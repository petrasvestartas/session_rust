use crate::tolerance::Tolerance;
use crate::tolerance::TOLERANCE;
use crate::Color;
use crate::Vector;
use crate::Xform;
use serde::Deserialize;
use serde::Serialize;
use std::fmt;
use std::ops::Add;
use std::ops::AddAssign;
use std::ops::Div;
use std::ops::DivAssign;
use std::ops::Index;
use std::ops::IndexMut;
use std::ops::Mul;
use std::ops::MulAssign;
use std::ops::Sub;
use std::ops::SubAssign;
use std::sync::OnceLock;

/// A 3D point with display width and color.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Point")]
pub struct Point {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: OnceLock<String>, // Lazily minted GUID.
    #[serde(rename = "x")]
    _x: f64, // X coordinate.
    #[serde(rename = "y")]
    _y: f64, // Y coordinate.
    #[serde(rename = "z")]
    _z: f64, // Z coordinate.
    pub name: String,      // Point name.
    pub width: f64,        // Display width.
    pub pointcolor: Color, // Display color.
}

impl Point {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from coordinates with the default name.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self::with_name(x, y, z, "my_point")
    }

    /// Construct from coordinates and a name.
    pub fn with_name(x: f64, y: f64, z: f64, name: &str) -> Self {
        Self {
            guid: OnceLock::new(),
            _x: x,
            _y: y,
            _z: z,
            name: name.to_string(),
            width: 1.0,
            pointcolor: Color::black(),
        }
    }

    /// Copy with a new guid and the same data.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = OnceLock::new();

        copy
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

    /// Set the guid.
    pub fn set_guid(&mut self, guid: String) {
        self.guid = OnceLock::from(guid);
    }

    /// Clear the guid so a fresh one mints lazily on the next read.
    pub fn refresh_guid(&mut self) {
        self.guid = OnceLock::new();
    }
}

impl Default for Point {
    /// Construct the origin.
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
impl Index<usize> for Point {
    type Output = f64;

    /// Return the coordinate by index (0=x, 1=y, 2=z).
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
    /// Return the mutable coordinate by index (0=x, 1=y, 2=z).
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self._x,
            1 => &mut self._y,
            2 => &mut self._z,
            _ => panic!("Index out of range"),
        }
    }
}

impl PartialEq for Point {
    /// Compare name, coordinates, width and color within rounding.
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && (self._x * 1000000.0).round() == (other._x * 1000000.0).round()
            && (self._y * 1000000.0).round() == (other._y * 1000000.0).round()
            && (self._z * 1000000.0).round() == (other._z * 1000000.0).round()
            && (self.width * 1000000.0).round() == (other.width * 1000000.0).round()
            && self.pointcolor == other.pointcolor
    }
}

impl MulAssign<f64> for Point {
    /// Scale in place.
    fn mul_assign(&mut self, factor: f64) {
        self._x *= factor;
        self._y *= factor;
        self._z *= factor;
    }
}

impl DivAssign<f64> for Point {
    /// Divide in place.
    fn div_assign(&mut self, factor: f64) {
        self._x /= factor;
        self._y /= factor;
        self._z /= factor;
    }
}

impl AddAssign<Vector> for Point {
    /// Translate in place.
    fn add_assign(&mut self, other: Vector) {
        self._x += other[0];
        self._y += other[1];
        self._z += other[2];
    }
}

impl SubAssign<Vector> for Point {
    /// Translate back in place.
    fn sub_assign(&mut self, other: Vector) {
        self._x -= other[0];
        self._y -= other[1];
        self._z -= other[2];
    }
}

impl Mul<f64> for Point {
    type Output = Point;

    /// Return a scaled copy.
    fn mul(self, factor: f64) -> Point {
        Point::new(self._x * factor, self._y * factor, self._z * factor)
    }
}

impl Div<f64> for Point {
    type Output = Point;

    /// Return a divided copy.
    fn div(self, factor: f64) -> Point {
        Point::new(self._x / factor, self._y / factor, self._z / factor)
    }
}

impl Add<Vector> for Point {
    type Output = Point;

    /// Return a translated copy.
    fn add(self, other: Vector) -> Point {
        Point::new(self._x + other[0], self._y + other[1], self._z + other[2])
    }
}

impl Sub<Vector> for Point {
    type Output = Point;

    /// Return a copy translated back.
    fn sub(self, other: Vector) -> Point {
        Point::new(self._x - other[0], self._y - other[1], self._z - other[2])
    }
}

impl Sub<Point> for Point {
    type Output = Vector;

    /// Return the vector from other to this point.
    fn sub(self, other: Point) -> Vector {
        Vector::new(self._x - other._x, self._y - other._y, self._z - other._z)
    }
}

impl Mul<f64> for &Point {
    type Output = Point;

    /// Return a scaled copy.
    fn mul(self, factor: f64) -> Point {
        Point::new(self._x * factor, self._y * factor, self._z * factor)
    }
}

impl Div<f64> for &Point {
    type Output = Point;

    /// Return a divided copy.
    fn div(self, factor: f64) -> Point {
        Point::new(self._x / factor, self._y / factor, self._z / factor)
    }
}

impl Add<Vector> for &Point {
    type Output = Point;

    /// Return a translated copy.
    fn add(self, other: Vector) -> Point {
        Point::new(self._x + other[0], self._y + other[1], self._z + other[2])
    }
}

impl Add<&Vector> for &Point {
    type Output = Point;

    /// Return a translated copy.
    fn add(self, other: &Vector) -> Point {
        Point::new(self._x + other[0], self._y + other[1], self._z + other[2])
    }
}

impl Sub<Vector> for &Point {
    type Output = Point;

    /// Return a copy translated back.
    fn sub(self, other: Vector) -> Point {
        Point::new(self._x - other[0], self._y - other[1], self._z - other[2])
    }
}

impl Sub<&Vector> for &Point {
    type Output = Point;

    /// Return a copy translated back.
    fn sub(self, other: &Vector) -> Point {
        Point::new(self._x - other[0], self._y - other[1], self._z - other[2])
    }
}

impl Sub<&Point> for &Point {
    type Output = Vector;

    /// Return the vector from other to this point.
    fn sub(self, other: &Point) -> Vector {
        Vector::new(self._x - other._x, self._y - other._y, self._z - other._z)
    }
}

impl Point {
    /// Return the coordinate-wise sum of two points.
    pub fn sum(p0: &Point, p1: &Point) -> Self {
        Point::new(p0[0] + p1[0], p0[1] + p1[1], p0[2] + p1[2])
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform in place.
    pub fn transform(&mut self, xform: &Xform) {
        let x = self._x;
        let y = self._y;
        let z = self._z;
        let m = &xform.m;
        let w = m[3] * x + m[7] * y + m[11] * z + m[15];
        let w_inv = if w.abs() > 1e-10 { 1.0 / w } else { 1.0 };

        self._x = (m[0] * x + m[4] * y + m[8] * z + m[12]) * w_inv;
        self._y = (m[1] * x + m[5] * y + m[9] * z + m[13]) * w_inv;
        self._z = (m[2] * x + m[6] * y + m[10] * z + m[14]) * w_inv;
    }

    /// Return a transformed copy.
    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut result = self.duplicate();
        result.transform(xform);

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether a, b, c turn counter-clockwise in the xy plane.
    pub fn is_ccw(a: &Point, b: &Point, c: &Point) -> bool {
        (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])
    }

    /// Return the mid point between a and b.
    pub fn mid_point(a: &Point, b: &Point) -> Point {
        Point::new(
            (a[0] + b[0]) / 2.0,
            (a[1] + b[1]) / 2.0,
            (a[2] + b[2]) / 2.0,
        )
    }

    /// Return the distance to p, scaled to stay finite for large coordinates.
    pub fn distance(&self, p: &Point, double_min: Option<f64>) -> f64 {
        let double_min = double_min.unwrap_or(1e-12);
        let mut dx = (self._x - p[0]).abs();
        let mut dy = (self._y - p[1]).abs();
        let mut dz = (self._z - p[2]).abs();

        if dy >= dx && dy >= dz {
            std::mem::swap(&mut dx, &mut dy);
        } else if dz >= dx && dz >= dy {
            std::mem::swap(&mut dx, &mut dz);
        }

        if dx > double_min {
            dy /= dx;
            dz /= dx;

            return dx * (1.0 + dy * dy + dz * dz).sqrt();
        }

        if dx > 0.0 && dx.is_finite() {
            return dx;
        }

        0.0
    }

    /// Return the squared distance to p, scaled to stay finite for large coordinates.
    pub fn squared_distance(&self, p: &Point, double_min: Option<f64>) -> f64 {
        let double_min = double_min.unwrap_or(1e-12);
        let mut dx = (self._x - p[0]).abs();
        let mut dy = (self._y - p[1]).abs();
        let mut dz = (self._z - p[2]).abs();

        if dy >= dx && dy >= dz {
            std::mem::swap(&mut dx, &mut dy);
        } else if dz >= dx && dz >= dy {
            std::mem::swap(&mut dx, &mut dz);
        }

        if dx > double_min {
            dy /= dx;
            dz /= dx;

            return dx * dx * (1.0 + dy * dy + dz * dz);
        }

        if dx > 0.0 && dx.is_finite() {
            return dx * dx;
        }

        0.0
    }

    /// Return the point at parameter t in [0, 1] between a and b.
    pub fn lerp(a: &Point, b: &Point, t: f64) -> Point {
        a + (b - a) * t
    }

    /// Return evenly spaced points between from and to (kind: 0=no endpoints, 1=both, 2=start only).
    pub fn interpolate(from: &Point, to: &Point, steps: usize, kind: usize) -> Vec<Point> {
        let mut points = Vec::new();

        if kind == 1 || kind == 2 {
            points.push(from.duplicate());
        }

        for i in 1..=steps {
            points.push(Point::lerp(from, to, i as f64 / (steps + 1) as f64));
        }

        if kind == 1 {
            points.push(to.duplicate());
        }

        points
    }

    /// Return the shoelace area of a polygon in the xy plane.
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

    /// Return the area-weighted centroid of a quadrilateral, or an error when not four vertices.
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
                (p0[0] * (p1[1] - p2[1]) + p1[0] * (p2[1] - p0[1]) + p2[0] * (p0[1] - p1[1])).abs()
                    / 2.0;
            let tri_centroid = Vector::new(
                (p0[0] + p1[0] + p2[0]) / 3.0,
                (p0[1] + p1[1] + p2[1]) / 3.0,
                (p0[2] + p1[2] + p2[2]) / 3.0,
            );

            total_area += tri_area;
            centroid_sum += tri_centroid * tri_area;
        }

        let result = centroid_sum / total_area;

        Ok(Point::new(result[0], result[1], result[2]))
    }

    /// Return the arithmetic mean of points; empty input returns the origin.
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

    /// Return the unsigned dihedral angle in degrees of edge pq between half-planes pqr and pqs.
    #[allow(clippy::approx_constant)]
    pub fn dihedral_angle_deg(p: &Point, q: &Point, r: &Point, s: &Point) -> f64 {
        let pq = q - p;
        let pr = r - p;
        let ps = s - p;
        let n1 = pq.cross(&pr);
        let n2 = pq.cross(&ps);
        let m1 = n1.magnitude();
        let m2 = n2.magnitude();

        if m1 < Tolerance::ZERO_TOLERANCE || m2 < Tolerance::ZERO_TOLERANCE {
            return 0.0;
        }

        let cos_t = (n1.dot(&n2) / (m1 * m2)).clamp(-1.0, 1.0);

        cos_t.acos() * (180.0 / 3.141592653589793)
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
        self.jsondump().expect("Failed to serialize Point JSON")
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).expect("Failed to parse Point JSON")
    }

    /// Write JSON to a file.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.jsondump()?)?;

        Ok(())
    }

    /// Read JSON from a file.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::Point {
        crate::proto::Point {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            x: self._x,
            y: self._y,
            z: self._z,
            width: self.width,
            pointcolor: Some(self.pointcolor.to_proto()),
        }
    }

    /// Construct from the protobuf message.
    pub fn from_proto(proto: crate::proto::Point) -> Self {
        let mut point = Self::new(proto.x, proto.y, proto.z);

        if !proto.guid.is_empty() {
            point.set_guid(proto.guid);
        }

        point.name = proto.name;
        point.width = proto.width;

        if let Some(color) = proto.pointcolor {
            point.pointcolor = Color::from_proto(color);
        }

        point
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Ok(Self::from_proto(crate::proto::Point::decode(data)?))
    }

    /// Write protobuf bytes to a file.
    pub fn pb_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.pb_dumps())?;

        Ok(())
    }

    /// Read protobuf bytes from a file.
    pub fn pb_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::pb_loads(&std::fs::read(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "x, y, z".
    pub fn str(&self) -> String {
        let prec = Tolerance::ROUNDING;

        format!(
            "{}, {}, {}",
            TOLERANCE.format_number(self._x, prec),
            TOLERANCE.format_number(self._y, prec),
            TOLERANCE.format_number(self._z, prec)
        )
    }

    /// Return "Point(name, x, y, z, Color(...), width)".
    pub fn repr(&self) -> String {
        let prec = Tolerance::ROUNDING;

        format!(
            "Point({}, {}, {}, {}, {}, {})",
            self.name,
            TOLERANCE.format_number(self._x, prec),
            TOLERANCE.format_number(self._y, prec),
            TOLERANCE.format_number(self._z, prec),
            self.pointcolor.repr(),
            TOLERANCE.format_number(self.width, prec)
        )
    }
}

impl fmt::Display for Point {
    /// Write the string representation to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SESSION_VIEWER
// ═══════════════════════════════════════════════════════════════════════════
impl Point {
    /// Return GPU-ready [x, y, z] as f32, the f64 to f32 boundary for wgpu upload.
    pub fn to_f32(&self) -> [f32; 3] {
        [self._x as f32, self._y as f32, self._z as f32]
    }
}
