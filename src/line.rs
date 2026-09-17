use crate::tolerance::Tolerance;
use crate::tolerance::TOLERANCE;
use crate::{Color, Point, Polyline, Vector, Xform};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};
use std::sync::OnceLock;

/// A 3D line segment with display width, dash pattern and color.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Line")]
pub struct Line {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: OnceLock<String>, // Lazy guid.
    pub name: String,     // Line name.
    pub width: f64,       // Display width.
    pub dash: Vec<f64>,   // Dash pattern lengths.
    pub linecolor: Color, // Display color.
    #[serde(rename = "x0")]
    _x0: f64, // Start x.
    #[serde(rename = "y0")]
    _y0: f64, // Start y.
    #[serde(rename = "z0")]
    _z0: f64, // Start z.
    #[serde(rename = "x1")]
    _x1: f64, // End x.
    #[serde(rename = "y1")]
    _y1: f64, // End y.
    #[serde(rename = "z1")]
    _z1: f64, // End z.
}

impl Default for Line {
    /// Constructs the unit segment from the origin along z.
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    }
}

impl Line {
    /// Constructs from start and end coordinates.
    pub fn new(x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) -> Self {
        Self {
            guid: OnceLock::new(),
            name: "my_line".to_string(),
            width: 1.0,
            dash: Vec::new(),
            linecolor: Color::black(),
            _x0: x0,
            _y0: y0,
            _z0: z0,
            _x1: x1,
            _y1: y1,
            _z1: z1,
        }
    }

    /// Copy (new guid, same data)
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = OnceLock::new();

        copy
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
    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Clears the guid so a fresh one mints lazily on the next read.
    pub fn refresh_guid(&mut self) {
        self.guid = OnceLock::new();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Constructs from two points.
    pub fn from_points(p1: &Point, p2: &Point) -> Self {
        Self::new(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2])
    }

    /// Constructs from point to point + vector.
    pub fn from_point_and_vector(point: &Point, vector: &Vector) -> Self {
        Self::new(
            point[0],
            point[1],
            point[2],
            point[0] + vector[0],
            point[1] + vector[1],
            point[2] + vector[2],
        )
    }

    /// Constructs from point along the normalized direction.
    pub fn from_point_direction_length(point: &Point, direction: &Vector, length: f64) -> Self {
        let d = direction.normalized();
        Self::new(
            point[0],
            point[1],
            point[2],
            point[0] + d[0] * length,
            point[1] + d[1] * length,
            point[2] + d[2] * length,
        )
    }

    /// Constructs the least-squares line through points by power-iteration PCA; length <= 0 spans the projected extent.
    pub fn fit_points(points: &[Point], length: Option<f64>) -> Self {
        if points.len() < 2 {
            panic!("At least 2 points are required for line fitting");
        }

        let n = points.len() as f64;
        let mut cx = 0.0;
        let mut cy = 0.0;
        let mut cz = 0.0;

        for p in points {
            cx += p[0];
            cy += p[1];
            cz += p[2];
        }

        cx /= n;
        cy /= n;
        cz /= n;
        let mut cxx = 0.0;
        let mut cyy = 0.0;
        let mut czz = 0.0;
        let mut cxy = 0.0;
        let mut cxz = 0.0;
        let mut cyz = 0.0;

        for p in points {
            let dx = p[0] - cx;
            let dy = p[1] - cy;
            let dz = p[2] - cz;
            cxx += dx * dx;
            cyy += dy * dy;
            czz += dz * dz;
            cxy += dx * dy;
            cxz += dx * dz;
            cyz += dy * dz;
        }

        let mut vx = 1.0;
        let mut vy = 0.0;
        let mut vz = 0.0;

        if cyy > cxx && cyy >= czz {
            vx = 0.0;
            vy = 1.0;
        } else if czz > cxx && czz > cyy {
            vx = 0.0;
            vz = 1.0;
        }

        for _ in 0..100 {
            let nx = cxx * vx + cxy * vy + cxz * vz;
            let ny = cxy * vx + cyy * vy + cyz * vz;
            let nz = cxz * vx + cyz * vy + czz * vz;
            let mag = (nx * nx + ny * ny + nz * nz).sqrt();

            if mag < 1e-15 {
                break;
            }

            vx = nx / mag;
            vy = ny / mag;
            vz = nz / mag;
        }

        let mut half = length.unwrap_or(0.0) / 2.0;

        if length.unwrap_or(0.0) <= 0.0 {
            let mut t_min: f64 = 0.0;
            let mut t_max: f64 = 0.0;

            for p in points {
                let t = (p[0] - cx) * vx + (p[1] - cy) * vy + (p[2] - cz) * vz;
                t_min = t_min.min(t);
                t_max = t_max.max(t);
            }

            half = t_min.abs().max(t_max.abs());

            if half < 1e-10 {
                half = 0.5;
            }
        }

        Self::new(
            cx - vx * half,
            cy - vy * half,
            cz - vz * half,
            cx + vx * half,
            cy + vy * half,
            cz + vz * half,
        )
    }

    /// Constructs a named line from coordinates.
    pub fn with_name(name: &str, x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) -> Self {
        let mut line = Self::new(x0, y0, z0, x1, y1, z1);
        line.name = name.to_string();

        line
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Transforms in place.
    pub fn transform(&mut self, xform: &Xform) {
        let mut start = Point::new(self._x0, self._y0, self._z0);
        let mut end = Point::new(self._x1, self._y1, self._z1);
        start.transform(xform);
        end.transform(xform);
        self._x0 = start[0];
        self._y0 = start[1];
        self._z0 = start[2];
        self._x1 = end[0];
        self._y1 = end[1];
        self._z1 = end[2];
    }

    /// Returns a transformed copy.
    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut result = self.clone();
        result.transform(xform);

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════

    /// Returns the length.
    pub fn length(&self) -> f64 {
        self.squared_length().sqrt()
    }

    /// Returns the squared length.
    pub fn squared_length(&self) -> f64 {
        let dx = self._x1 - self._x0;
        let dy = self._y1 - self._y0;
        let dz = self._z1 - self._z0;

        dx * dx + dy * dy + dz * dz
    }

    /// Returns the vector from start to end.
    pub fn to_vector(&self) -> Vector {
        Vector::new(
            self._x1 - self._x0,
            self._y1 - self._y0,
            self._z1 - self._z0,
        )
    }

    /// Returns the unit vector from start to end.
    pub fn to_direction(&self) -> Vector {
        self.to_vector().normalized()
    }

    /// Returns the start point.
    pub fn start(&self) -> Point {
        Point::new(self._x0, self._y0, self._z0)
    }

    /// Returns the end point.
    pub fn end(&self) -> Point {
        Point::new(self._x1, self._y1, self._z1)
    }

    /// Returns the midpoint.
    pub fn center(&self) -> Point {
        Point::new(
            (self._x0 + self._x1) * 0.5,
            (self._y0 + self._y1) * 0.5,
            (self._z0 + self._z1) * 0.5,
        )
    }

    /// Returns the point at parameter t (0 = start, 1 = end).
    pub fn point_at(&self, t: f64) -> Point {
        let s = 1.0 - t;
        Point::new(
            s * self._x0 + t * self._x1,
            s * self._y0 + t * self._y1,
            s * self._z0 + t * self._z1,
        )
    }

    /// Returns n evenly spaced points including both ends.
    pub fn subdivide(&self, n: usize) -> Vec<Point> {
        if n < 2 {
            panic!("n must be at least 2");
        }

        let mut points = Vec::with_capacity(n);

        for i in 0..n {
            points.push(self.point_at(i as f64 / (n - 1) as f64));
        }

        points
    }

    /// Returns points spaced approximately distance apart including both ends.
    pub fn subdivide_by_distance(&self, distance: f64) -> Vec<Point> {
        if distance <= 0.0 {
            panic!("distance must be positive");
        }

        let total = self.length();

        if total < 1e-10 {
            return vec![self.start(), self.end()];
        }

        let n = 2.max((total / distance + 0.5) as usize + 1);

        self.subdivide(n)
    }

    /// Returns the parameter and closest point; limited clamps t to [0, 1].
    pub fn closest_point(&self, point: &Point, limited: bool) -> (f64, Point) {
        let dx = self._x1 - self._x0;
        let dy = self._y1 - self._y0;
        let dz = self._z1 - self._z0;
        let len_sq = dx * dx + dy * dy + dz * dz;

        if len_sq < 1e-20 {
            return (0.0, self.start());
        }

        let mut t =
            ((point[0] - self._x0) * dx + (point[1] - self._y0) * dy + (point[2] - self._z0) * dz)
                / len_sq;

        if limited {
            t = t.clamp(0.0, 1.0);
        }

        (t, self.point_at(t))
    }

    /// Computes the line through the midpoints of the paired starts and ends.
    pub fn get_middle_line(
        line0_start: &Point,
        line0_end: &Point,
        line1_start: &Point,
        line1_end: &Point,
    ) -> (Point, Point) {
        let output_start = Point::new(
            (line0_start[0] + line1_start[0]) * 0.5,
            (line0_start[1] + line1_start[1]) * 0.5,
            (line0_start[2] + line1_start[2]) * 0.5,
        );
        let output_end = Point::new(
            (line0_end[0] + line1_end[0]) * 0.5,
            (line0_end[1] + line1_end[1]) * 0.5,
            (line0_end[2] + line1_end[2]) * 0.5,
        );

        (output_start, output_end)
    }

    /// Computes the extreme sub-segment of line spanned by the projected points; None when empty.
    pub fn from_projected_points(line: &Line, points: &[Point]) -> Option<Line> {
        let result = Polyline::line_from_projected_points(&line.start(), &line.end(), points)?;

        Some(Line::from_points(&result.0, &result.1))
    }

    /// Computes the collinear overlap with other; None when none or a single point.
    pub fn overlap(&self, other: &Line) -> Option<Line> {
        let result =
            Polyline::line_line_overlap(&self.start(), &self.end(), &other.start(), &other.end())?;

        Some(Line::from_points(&result.0, &result.1))
    }

    /// Computes the longer of the two midpoint pairings of overlap(other) and other.overlap(self); None when empty.
    pub fn overlap_average(&self, other: &Line) -> Option<Line> {
        let result = Polyline::line_line_overlap_average(
            &self.start(),
            &self.end(),
            &other.start(),
            &other.end(),
        );
        let out = Line::from_points(&result.0, &result.1);

        if out.squared_length() <= 0.0 {
            return None;
        }

        Some(out)
    }

    /// Grows start by ext_start and end by ext_end.
    pub fn extend(&mut self, ext_start: f64, ext_end: f64) {
        let mut s = self.start();
        let mut e = self.end();
        Polyline::extend_line_segment(&mut s, &mut e, ext_start, ext_end);
        self._x0 = s[0];
        self._y0 = s[1];
        self._z0 = s[2];
        self._x1 = e[0];
        self._y1 = e[1];
        self._z1 = e[2];
    }

    /// Grows both ends by dist, or by proportion of the length when non-zero.
    pub fn extend_equally(&mut self, dist: f64, proportion: f64) {
        if dist == 0.0 && proportion == 0.0 {
            return;
        }

        let mut s = self.start();
        let mut e = self.end();
        Polyline::extend_segment_equally_static(&mut s, &mut e, dist, proportion);
        self._x0 = s[0];
        self._y0 = s[1];
        self._z0 = s[2];
        self._x1 = e[0];
        self._y1 = e[1];
        self._z1 = e[2];
    }

    /// Shrinks both ends by dist as a fraction of the length.
    pub fn scale(&mut self, dist: f64) {
        let mut s = self.start();
        let mut e = self.end();
        Polyline::shrink_line_segment(&mut s, &mut e, dist);
        self._x0 = s[0];
        self._y0 = s[1];
        self._z0 = s[2];
        self._x1 = e[0];
        self._y1 = e[1];
        self._z1 = e[2];
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
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
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_default()
    }

    /// Writes to a JSON file.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.jsondump()?)?;

        Ok(())
    }

    /// Reads from a JSON file.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════

    /// Serializes to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserializes from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, prost::DecodeError> {
        use prost::Message;

        Ok(Self::from_proto(crate::proto::Line::decode(data)?))
    }

    /// Writes to a protobuf file.
    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    /// Reads from a protobuf file.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");

        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    /// Converts to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::Line {
        crate::proto::Line {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            width: self.width,
            coords: vec![self._x0, self._y0, self._z0, self._x1, self._y1, self._z1],
            dash: self.dash.clone(),
            linecolor_rgba: vec![
                self.linecolor.r,
                self.linecolor.g,
                self.linecolor.b,
                self.linecolor.a,
            ],
            linecolor_name: self.linecolor.name.clone(),
        }
    }

    /// Constructs from the protobuf message.
    pub fn from_proto(proto: crate::proto::Line) -> Self {
        let mut line = Self::default();

        if proto.coords.len() == 6 {
            line = Self::new(
                proto.coords[0],
                proto.coords[1],
                proto.coords[2],
                proto.coords[3],
                proto.coords[4],
                proto.coords[5],
            );
        }

        if !proto.guid.is_empty() {
            line.set_guid(proto.guid);
        }

        line.name = proto.name;

        if proto.width > 0.0 {
            line.width = proto.width;
        }

        line.dash = proto.dash;

        if proto.linecolor_rgba.len() == 4 {
            line.linecolor.r = proto.linecolor_rgba[0];
            line.linecolor.g = proto.linecolor_rgba[1];
            line.linecolor.b = proto.linecolor_rgba[2];
            line.linecolor.a = proto.linecolor_rgba[3];

            if !proto.linecolor_name.is_empty() {
                line.linecolor.name = proto.linecolor_name;
            }
        }

        line
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════

    /// "x0, y0, z0, x1, y1, z1"
    pub fn str(&self) -> String {
        let prec = Tolerance::ROUNDING;
        format!(
            "{}, {}, {}, {}, {}, {}",
            TOLERANCE.format_number(self._x0, prec),
            TOLERANCE.format_number(self._y0, prec),
            TOLERANCE.format_number(self._z0, prec),
            TOLERANCE.format_number(self._x1, prec),
            TOLERANCE.format_number(self._y1, prec),
            TOLERANCE.format_number(self._z1, prec)
        )
    }

    /// Returns "Line(name, x0, y0, z0, x1, y1, z1, Color(...), width)".
    pub fn repr(&self) -> String {
        let prec = Tolerance::ROUNDING;
        format!(
            "Line({}, {}, {}, {}, {}, {}, {}, {}, {})",
            self.name,
            TOLERANCE.format_number(self._x0, prec),
            TOLERANCE.format_number(self._y0, prec),
            TOLERANCE.format_number(self._z0, prec),
            TOLERANCE.format_number(self._x1, prec),
            TOLERANCE.format_number(self._y1, prec),
            TOLERANCE.format_number(self._z1, prec),
            self.linecolor.repr(),
            TOLERANCE.format_number(self.width, prec)
        )
    }
}

impl fmt::Display for Line {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

impl Index<usize> for Line {
    type Output = f64;

    /// Returns the coordinate by index (0=x0, 1=y0, 2=z0, 3=x1, 4=y1, 5=z1).
    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self._x0,
            1 => &self._y0,
            2 => &self._z0,
            3 => &self._x1,
            4 => &self._y1,
            5 => &self._z1,
            _ => panic!("Index out of bounds"),
        }
    }
}

impl IndexMut<usize> for Line {
    /// Returns the mutable coordinate by index (0=x0, 1=y0, 2=z0, 3=x1, 4=y1, 5=z1).
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self._x0,
            1 => &mut self._y0,
            2 => &mut self._z0,
            3 => &mut self._x1,
            4 => &mut self._y1,
            5 => &mut self._z1,
            _ => panic!("Index out of bounds"),
        }
    }
}

impl PartialEq for Line {
    /// Compares name, coordinates to 1e-6, width and linecolor; guid ignored.
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && (self._x0 * 1000000.0).round() == (other._x0 * 1000000.0).round()
            && (self._y0 * 1000000.0).round() == (other._y0 * 1000000.0).round()
            && (self._z0 * 1000000.0).round() == (other._z0 * 1000000.0).round()
            && (self._x1 * 1000000.0).round() == (other._x1 * 1000000.0).round()
            && (self._y1 * 1000000.0).round() == (other._y1 * 1000000.0).round()
            && (self._z1 * 1000000.0).round() == (other._z1 * 1000000.0).round()
            && (self.width * 1000000.0).round() == (other.width * 1000000.0).round()
            && self.linecolor == other.linecolor
    }
}

impl AddAssign<&Vector> for Line {
    /// Translates in place.
    fn add_assign(&mut self, other: &Vector) {
        self._x0 += other[0];
        self._y0 += other[1];
        self._z0 += other[2];
        self._x1 += other[0];
        self._y1 += other[1];
        self._z1 += other[2];
    }
}

impl SubAssign<&Vector> for Line {
    /// Translates back in place.
    fn sub_assign(&mut self, other: &Vector) {
        self._x0 -= other[0];
        self._y0 -= other[1];
        self._z0 -= other[2];
        self._x1 -= other[0];
        self._y1 -= other[1];
        self._z1 -= other[2];
    }
}

impl MulAssign<f64> for Line {
    /// Scales both ends in place.
    fn mul_assign(&mut self, factor: f64) {
        self._x0 *= factor;
        self._y0 *= factor;
        self._z0 *= factor;
        self._x1 *= factor;
        self._y1 *= factor;
        self._z1 *= factor;
    }
}

impl DivAssign<f64> for Line {
    /// Divides both ends in place.
    fn div_assign(&mut self, factor: f64) {
        self._x0 /= factor;
        self._y0 /= factor;
        self._z0 /= factor;
        self._x1 /= factor;
        self._y1 /= factor;
        self._z1 /= factor;
    }
}

impl Add<&Vector> for Line {
    type Output = Line;

    /// Returns a translated copy.
    fn add(self, other: &Vector) -> Line {
        let mut result = self;
        result += other;

        result
    }
}

impl Sub<&Vector> for Line {
    type Output = Line;

    /// Returns a copy translated back.
    fn sub(self, other: &Vector) -> Line {
        let mut result = self;
        result -= other;

        result
    }
}

impl Mul<f64> for Line {
    type Output = Line;

    /// Returns a copy with both ends scaled.
    fn mul(self, factor: f64) -> Line {
        let mut result = self;
        result *= factor;

        result
    }
}

impl Div<f64> for Line {
    type Output = Line;

    /// Returns a copy with both ends divided.
    fn div(self, factor: f64) -> Line {
        let mut result = self;
        result /= factor;

        result
    }
}

impl Neg for Line {
    type Output = Line;

    /// Returns a flipped copy (end to start).
    fn neg(self) -> Line {
        Line::new(self._x1, self._y1, self._z1, self._x0, self._y0, self._z0)
    }
}

impl Add<&Vector> for &Line {
    type Output = Line;

    /// Returns a translated copy.
    fn add(self, other: &Vector) -> Line {
        self.clone() + other
    }
}

impl Sub<&Vector> for &Line {
    type Output = Line;

    /// Returns a copy translated back.
    fn sub(self, other: &Vector) -> Line {
        self.clone() - other
    }
}

impl Mul<f64> for &Line {
    type Output = Line;

    /// Returns a copy with both ends scaled.
    fn mul(self, factor: f64) -> Line {
        self.clone() * factor
    }
}

impl Div<f64> for &Line {
    type Output = Line;

    /// Returns a copy with both ends divided.
    fn div(self, factor: f64) -> Line {
        self.clone() / factor
    }
}

impl Neg for &Line {
    type Output = Line;

    /// Returns a flipped copy (end to start).
    fn neg(self) -> Line {
        Line::new(self._x1, self._y1, self._z1, self._x0, self._y0, self._z0)
    }
}
