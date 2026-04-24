use crate::{Color, Point, Vector, Xform};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Line")]
pub struct Line {
    #[serde(serialize_with = "crate::guid_serde::serialize", deserialize_with = "crate::guid_serde::deserialize")]
    guid: std::sync::OnceLock<String>,
    pub name: String,
    #[serde(rename = "x0")]
    _x0: f64,
    #[serde(rename = "y0")]
    _y0: f64,
    #[serde(rename = "z0")]
    _z0: f64,
    #[serde(rename = "x1")]
    _x1: f64,
    #[serde(rename = "y1")]
    _y1: f64,
    #[serde(rename = "z1")]
    _z1: f64,
    pub width: f64,
    pub linecolor: Color,
    #[serde(default = "Xform::identity")]
    pub xform: Xform,
}

impl Default for Line {
    fn default() -> Self {
        Self {
            _x0: 0.0,
            _y0: 0.0,
            _z0: 0.0,
            _x1: 0.0,
            _y1: 0.0,
            _z1: 1.0,
            guid: std::sync::OnceLock::new(),
            name: "my_line".to_string(),
            linecolor: Color::black(),
            width: 1.0,
            xform: Xform::identity(),
        }
    }
}

impl Line {
    pub fn new(x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) -> Self {
        Self {
            _x0: x0,
            _y0: y0,
            _z0: z0,
            _x1: x1,
            _y1: y1,
            _z1: z1,
            ..Default::default()
        }
    }

    pub fn with_name(name: &str, x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) -> Self {
        Self {
            name: name.to_string(),
            _x0: x0,
            _y0: y0,
            _z0: z0,
            _x1: x1,
            _y1: y1,
            _z1: z1,
            ..Default::default()
        }
    }

    pub fn from_points(p1: &Point, p2: &Point) -> Self {
        Self::new(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2])
    }

    /// Fit a line to a set of points using least squares (PCA).
    ///
    /// Uses Principal Component Analysis to find the best-fit line
    /// that minimizes perpendicular distances to all points.
    ///
    /// # Arguments
    /// * `points` - Slice of points to fit (minimum 2 required)
    /// * `length` - Optional length of resulting line (None = auto from extent)
    ///
    /// # Panics
    /// Panics if fewer than 2 points are provided.
    pub fn fit_points(points: &[Point], length: Option<f64>) -> Self {
        if points.len() < 2 {
            panic!("At least 2 points are required for line fitting");
        }

        let n = points.len() as f64;

        // Compute centroid
        let (mut cx, mut cy, mut cz) = (0.0, 0.0, 0.0);
        for p in points {
            cx += p[0];
            cy += p[1];
            cz += p[2];
        }
        cx /= n;
        cy /= n;
        cz /= n;

        // Compute covariance matrix elements
        let (mut cxx, mut cyy, mut czz) = (0.0, 0.0, 0.0);
        let (mut cxy, mut cxz, mut cyz) = (0.0, 0.0, 0.0);
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

        // Power iteration to find dominant eigenvector
        let (mut vx, mut vy, mut vz) = (1.0, 0.0, 0.0);
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

        // Determine line extent from projected points
        let half_len = match length {
            Some(len) => len / 2.0,
            None => {
                let (mut t_min, mut t_max): (f64, f64) = (0.0, 0.0);
                for p in points {
                    let dx = p[0] - cx;
                    let dy = p[1] - cy;
                    let dz = p[2] - cz;
                    let t = dx * vx + dy * vy + dz * vz;
                    t_min = t_min.min(t);
                    t_max = t_max.max(t);
                }
                let hl = t_min.abs().max(t_max.abs());
                if hl < 1e-10 { 0.5 } else { hl }
            }
        };

        // Create line from centroid +/- direction * half_len
        Self::new(
            cx - vx * half_len, cy - vy * half_len, cz - vz * half_len,
            cx + vx * half_len, cy + vy * half_len, cz + vz * half_len,
        )
    }

    pub fn from_point_and_vector(point: &Point, vector: &Vector) -> Self {
        Self::new(
            point[0], point[1], point[2],
            point[0] + vector[0], point[1] + vector[1], point[2] + vector[2],
        )
    }

    pub fn from_point_direction_length(point: &Point, direction: &Vector, length: f64) -> Self {
        let d = direction.normalized();
        Self::new(
            point[0], point[1], point[2],
            point[0] + d[0] * length, point[1] + d[1] * length, point[2] + d[2] * length,
        )
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Create a duplicate with a new GUID.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = std::sync::OnceLock::new();
        copy
    }

    pub fn transform(&mut self) {
        let mut start = Point::new(self._x0, self._y0, self._z0);
        let mut end = Point::new(self._x1, self._y1, self._z1);

        start.xform = self.xform.clone();
        start.transform();
        end.xform = self.xform.clone();
        end.transform();

        self._x0 = start[0];
        self._y0 = start[1];
        self._z0 = start[2];
        self._x1 = end[0];
        self._y1 = end[1];
        self._z1 = end[2];
        self.xform = Xform::identity();
    }

    pub fn transformed(&self) -> Self {
        let mut result = self.clone();
        result.transform();
        result
    }

    pub fn length(&self) -> f64 {
        let dx = self._x1 - self._x0;
        let dy = self._y1 - self._y0;
        let dz = self._z1 - self._z0;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    pub fn squared_length(&self) -> f64 {
        let dx = self._x1 - self._x0;
        let dy = self._y1 - self._y0;
        let dz = self._z1 - self._z0;
        dx * dx + dy * dy + dz * dz
    }

    pub fn to_vector(&self) -> Vector {
        Vector::new(
            self._x1 - self._x0,
            self._y1 - self._y0,
            self._z1 - self._z0,
        )
    }

    pub fn to_direction(&self) -> Vector {
        self.to_vector().normalized()
    }

    pub fn point_at(&self, t: f64) -> Point {
        let s = 1.0 - t;
        Point::new(
            s * self._x0 + t * self._x1,
            s * self._y0 + t * self._y1,
            s * self._z0 + t * self._z1,
        )
    }

    /// Extend both endpoints of this line by `distance` along its tangent.
    /// Negative `distance` shortens the line; if the requested shortening
    /// would collapse the line to zero length, the line is left unchanged.
    /// Mirrors the wood-library helper `cgal::polyline_util::extend_equally`.
    pub fn extend_equally(&mut self, distance: f64) {
        let len = self.length();
        if len < crate::tolerance::Tolerance::ZERO_TOLERANCE {
            return;
        }
        // Don't allow the line to collapse to zero or invert.
        if distance < 0.0 && (-distance) * 2.0 >= len {
            return;
        }
        let inv_len = 1.0 / len;
        let dx = (self._x1 - self._x0) * inv_len * distance;
        let dy = (self._y1 - self._y0) * inv_len * distance;
        let dz = (self._z1 - self._z0) * inv_len * distance;
        self._x0 -= dx;
        self._y0 -= dy;
        self._z0 -= dz;
        self._x1 += dx;
        self._y1 += dy;
        self._z1 += dz;
    }

    /// Subdivide line into n points.
    pub fn subdivide(&self, n: usize) -> Vec<Point> {
        if n < 2 {
            panic!("n must be at least 2");
        }
        let mut points = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f64 / (n - 1) as f64;
            points.push(self.point_at(t));
        }
        points
    }

    /// Subdivide line by approximate distance between points.
    pub fn subdivide_by_distance(&self, distance: f64) -> Vec<Point> {
        if distance <= 0.0 {
            panic!("distance must be positive");
        }
        let len = self.length();
        if len < 1e-10 {
            return vec![self.start(), self.end()];
        }
        let n = 2.max((len / distance + 0.5) as usize + 1);
        self.subdivide(n)
    }

    pub fn start(&self) -> Point {
        Point::new(self._x0, self._y0, self._z0)
    }

    pub fn end(&self) -> Point {
        Point::new(self._x1, self._y1, self._z1)
    }

    pub fn center(&self) -> Point {
        Point::new(
            (self._x0 + self._x1) * 0.5,
            (self._y0 + self._y1) * 0.5,
            (self._z0 + self._z1) * 0.5,
        )
    }

    pub fn closest_point(&self, point: &Point, limited: bool) -> (f64, Point) {
        let dx = self._x1 - self._x0;
        let dy = self._y1 - self._y0;
        let dz = self._z1 - self._z0;
        let len_sq = dx * dx + dy * dy + dz * dz;
        if len_sq < 1e-20 {
            return (0.0, self.start());
        }
        let mut t = ((point[0] - self._x0) * dx + (point[1] - self._y0) * dy + (point[2] - self._z0) * dz) / len_sq;
        if limited {
            t = t.clamp(0.0, 1.0);
        }
        (t, self.point_at(t))
    }

    /// Calculate middle line between two line segments
    pub fn get_middle_line(
        line0_start: &Point,
        line0_end: &Point,
        line1_start: &Point,
        line1_end: &Point,
    ) -> (Point, Point) {
        let p0 = Point::new(
            (line0_start[0] + line1_start[0]) * 0.5,
            (line0_start[1] + line1_start[1]) * 0.5,
            (line0_start[2] + line1_start[2]) * 0.5,
        );
        let p1 = Point::new(
            (line0_end[0] + line1_end[0]) * 0.5,
            (line0_end[1] + line1_end[1]) * 0.5,
            (line0_end[2] + line1_end[2]) * 0.5,
        );
        (p0, p1)
    }

    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_else(|_| Self::default())
    }

    /// Serialize to JSON file.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    /// Deserialize from JSON file.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to protobuf binary format.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        let proto = crate::proto::Line {
            start: Some(crate::proto::Point {
                x: self._x0,
                y: self._y0,
                z: self._z0,
                guid: String::new(),
                name: String::new(),
                width: 1.0,
                pointcolor: None,
                xform: None,
            }),
            end: Some(crate::proto::Point {
                x: self._x1,
                y: self._y1,
                z: self._z1,
                guid: String::new(),
                name: String::new(),
                width: 1.0,
                pointcolor: None,
                xform: None,
            }),
            guid: self.guid().to_string(),
            name: self.name.clone(),
            xform: None,
        };
        proto.encode_to_vec()
    }

    /// Create from protobuf binary data.
    pub fn pb_loads(data: &[u8]) -> Result<Self, prost::DecodeError> {
        use prost::Message;
        let proto = crate::proto::Line::decode(data)?;
        let start = proto.start.unwrap_or_default();
        let end = proto.end.unwrap_or_default();
        let mut line = Self::new(start.x, start.y, start.z, end.x, end.y, end.z);
        line.set_guid(proto.guid);
        line.name = proto.name;
        Ok(line)
    }

    /// Write protobuf to file.
    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    /// Read protobuf from file.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    /// Short string representation.
    pub fn str(&self) -> String {
        use crate::tolerance::TOLERANCE;
        let prec = crate::tolerance::Tolerance::ROUNDING;
        format!(
            "{}, {}, {}, {}, {}, {}",
            TOLERANCE.format_number(self._x0, prec),
            TOLERANCE.format_number(self._y0, prec),
            TOLERANCE.format_number(self._z0, prec),
            TOLERANCE.format_number(self._x1, prec),
            TOLERANCE.format_number(self._y1, prec),
            TOLERANCE.format_number(self._z1, prec),
        )
    }

    /// Detailed string representation (like Python __repr__).
    pub fn repr(&self) -> String {
        use crate::tolerance::TOLERANCE;
        let prec = crate::tolerance::Tolerance::ROUNDING;
        format!(
            "Line({}, {}, {}, {}, {}, {}, {}, Color({}, {}, {}, {}), {})",
            self.name,
            TOLERANCE.format_number(self._x0, prec),
            TOLERANCE.format_number(self._y0, prec),
            TOLERANCE.format_number(self._z0, prec),
            TOLERANCE.format_number(self._x1, prec),
            TOLERANCE.format_number(self._y1, prec),
            TOLERANCE.format_number(self._z1, prec),
            self.linecolor.r,
            self.linecolor.g,
            self.linecolor.b,
            self.linecolor.a,
            TOLERANCE.format_number(self.width, prec),
        )
    }

    /// Return the overlapping segment between this line and `other`.
    ///
    /// Returns `None` if the segments do not overlap (or overlap at a point).
    pub fn overlap(&self, other: &Line) -> Option<Line> {
        let s0 = self.start();
        let e0 = self.end();
        let s1 = other.start();
        let e1 = other.end();
        let r = crate::polyline::Polyline::line_line_overlap(&s0, &e0, &s1, &e1)?;
        Some(Line::from_points(&r.0, &r.1))
    }

    /// Return the average of the two reciprocal overlaps between this line and `other`.
    ///
    /// Returns `None` if the resulting overlap collapses to a point.
    pub fn overlap_average(&self, other: &Line) -> Option<Line> {
        let s0 = self.start();
        let e0 = self.end();
        let s1 = other.start();
        let e1 = other.end();
        let (a, b) = crate::polyline::Polyline::line_line_overlap_average(&s0, &e0, &s1, &e1);
        let out = Line::from_points(&a, &b);
        if out.squared_length() > 0.0 { Some(out) } else { None }
    }

    /// Extend this line in place by `ext_start` at the start end and
    /// `ext_end` at the end.
    pub fn extend(&mut self, ext_start: f64, ext_end: f64) {
        let s = self.start();
        let e = self.end();
        let mut v = e.clone() - s.clone();
        v.normalize_self();
        let new_s = s - (v.clone() * ext_start);
        let new_e = e + (v * ext_end);
        self._x0 = new_s[0]; self._y0 = new_s[1]; self._z0 = new_s[2];
        self._x1 = new_e[0]; self._y1 = new_e[1]; self._z1 = new_e[2];
    }

}

impl Index<usize> for Line {
    type Output = f64;

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

impl AddAssign<&Vector> for Line {
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

    fn add(self, other: &Vector) -> Line {
        let mut result = self;
        result += other;
        result
    }
}

impl Sub<&Vector> for Line {
    type Output = Line;

    fn sub(self, other: &Vector) -> Line {
        let mut result = self;
        result -= other;
        result
    }
}

impl Mul<f64> for Line {
    type Output = Line;

    fn mul(self, factor: f64) -> Line {
        let mut result = self;
        result *= factor;
        result
    }
}

impl Div<f64> for Line {
    type Output = Line;

    fn div(self, factor: f64) -> Line {
        let mut result = self;
        result /= factor;
        result
    }
}

impl Neg for Line {
    type Output = Line;

    fn neg(self) -> Line {
        Line::new(self._x1, self._y1, self._z1, self._x0, self._y0, self._z0)
    }
}

impl Add<&Vector> for &Line {
    type Output = Line;

    fn add(self, other: &Vector) -> Line {
        self.clone() + other
    }
}

impl Sub<&Vector> for &Line {
    type Output = Line;

    fn sub(self, other: &Vector) -> Line {
        self.clone() - other
    }
}

impl Mul<f64> for &Line {
    type Output = Line;

    fn mul(self, factor: f64) -> Line {
        self.clone() * factor
    }
}

impl Div<f64> for &Line {
    type Output = Line;

    fn div(self, factor: f64) -> Line {
        self.clone() / factor
    }
}

impl Neg for &Line {
    type Output = Line;

    fn neg(self) -> Line {
        Line::new(self._x1, self._y1, self._z1, self._x0, self._y0, self._z0)
    }
}

impl fmt::Display for Line {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Line({}, {}, {}, {}, {}, {})",
            self._x0, self._y0, self._z0, self._x1, self._y1, self._z1
        )
    }
}

impl PartialEq for Line {
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



#[path = "line_test.rs"]
#[cfg(test)]
mod line_test;
