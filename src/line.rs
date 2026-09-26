use crate::tolerance::Tolerance;
use crate::tolerance::TOLERANCE;
use crate::Color;
use crate::Point;
use crate::Polyline;
use crate::Vector;
use crate::Xform;
use serde::Deserialize;
use serde::Serialize;
use std::collections::BTreeSet;
use std::fmt;
use std::ops::Add;
use std::ops::AddAssign;
use std::ops::Div;
use std::ops::DivAssign;
use std::ops::Index;
use std::ops::IndexMut;
use std::ops::Mul;
use std::ops::MulAssign;
use std::ops::Neg;
use std::ops::Sub;
use std::ops::SubAssign;
use std::sync::OnceLock;

/// A 3D line segment with display width, dash pattern and color.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Line")]
pub struct Line {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: OnceLock<String>, // Lazily minted GUID.
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
    pub name: String,     // Line name.
    pub width: f64,       // Display width.
    pub dash: Vec<f64>,   // Dash pattern lengths.
    pub linecolor: Color, // Display color.
}

impl Line {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from start and end coordinates.
    pub fn new(x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) -> Self {
        Self {
            guid: OnceLock::new(),
            _x0: x0,
            _y0: y0,
            _z0: z0,
            _x1: x1,
            _y1: y1,
            _z1: z1,
            name: "my_line".to_string(),
            width: 1.0,
            dash: Vec::new(),
            linecolor: Color::black(),
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

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from two points.
    pub fn from_points(p1: &Point, p2: &Point) -> Self {
        Self::new(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2])
    }

    /// Construct from point to point + vector.
    pub fn from_point_and_vector(point: &Point, vector: &Vector) -> Self {
        Self::from_points(point, &(point + vector))
    }

    /// Construct from point along the normalized direction.
    pub fn from_point_direction_length(point: &Point, direction: &Vector, length: f64) -> Self {
        Self::from_points(point, &(point + &(&direction.normalized() * length)))
    }

    /// Power iteration on the covariance rows from seed: the unit axis and its eigenvalue estimate.
    fn fit_points_power(
        row0: &Vector,
        row1: &Vector,
        row2: &Vector,
        seed: Vector,
    ) -> (Vector, f64) {
        let mut axis = seed;
        let mut eigen = 0.0;

        for _ in 0..100 {
            let next = Vector::new(row0.dot(&axis), row1.dot(&axis), row2.dot(&axis));
            eigen = next.magnitude_squared().sqrt();

            if eigen < 1e-15 {
                break;
            }

            axis = next / eigen;
        }

        (axis, eigen)
    }

    /// Principal direction of the points about center: power iteration from each of X, Y and Z, largest eigenvalue kept.
    fn fit_points_axis(points: &[Point], center: &Point) -> Vector {
        let mut cxx = 0.0;
        let mut cyy = 0.0;
        let mut czz = 0.0;
        let mut cxy = 0.0;
        let mut cxz = 0.0;
        let mut cyz = 0.0;

        for p in points {
            let d = p - center;
            cxx += d[0] * d[0];
            cyy += d[1] * d[1];
            czz += d[2] * d[2];
            cxy += d[0] * d[1];
            cxz += d[0] * d[2];
            cyz += d[1] * d[2];
        }

        let row0 = Vector::new(cxx, cxy, cxz);
        let row1 = Vector::new(cxy, cyy, cyz);
        let row2 = Vector::new(cxz, cyz, czz);
        let mut first = 0;

        if cyy > cxx && cyy >= czz {
            first = 1;
        } else if czz > cxx && czz > cyy {
            first = 2;
        }

        let mut axis = Vector::new(1.0, 0.0, 0.0);
        let mut best = -1.0;

        for k in 0..3 {
            let mut seed = Vector::new(0.0, 0.0, 0.0);
            seed[(first + k) % 3] = 1.0;
            let power = Self::fit_points_power(&row0, &row1, &row2, seed);

            if power.1 > best * (1.0 + Tolerance::RELATIVE) {
                axis = power.0;
                best = power.1;
            }
        }

        axis
    }

    /// Parameter range of the fitted line along axis: +-length / 2, or the projected extent when length <= 0.
    fn fit_points_extent(
        points: &[Point],
        center: &Point,
        axis: &Vector,
        length: f64,
    ) -> (f64, f64) {
        if length > 0.0 {
            return (-length / 2.0, length / 2.0);
        }

        let mut t_min: f64 = 0.0;
        let mut t_max: f64 = 0.0;

        for p in points {
            let t = (p - center).dot(axis);
            t_min = t_min.min(t);
            t_max = t_max.max(t);
        }

        if t_max - t_min < 1e-10 {
            return (-0.5, 0.5);
        }

        (t_min, t_max)
    }

    /// Construct the least-squares line through points by power-iteration PCA; length <= 0 spans the projected extent.
    pub fn fit_points(points: &[Point], length: Option<f64>) -> Self {
        if points.len() < 2 {
            panic!("At least 2 points are required for line fitting");
        }

        let length = length.unwrap_or(0.0);
        let center = Point::centroid(points);
        let axis = Self::fit_points_axis(points, &center);
        let extent = Self::fit_points_extent(points, &center, &axis, length);

        Self::from_points(
            &(&center + &(&axis * extent.0)),
            &(&center + &(&axis * extent.1)),
        )
    }

    /// Construct a named line from coordinates.
    pub fn with_name(name: &str, x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) -> Self {
        let mut line = Self::new(x0, y0, z0, x1, y1, z1);
        line.name = name.to_string();

        line
    }
}

impl Default for Line {
    /// Construct the unit segment from the origin along z.
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
impl Index<usize> for Line {
    type Output = f64;

    /// Return the coordinate by index (0=x0, 1=y0, 2=z0, 3=x1, 4=y1, 5=z1).
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
    /// Return the mutable coordinate by index (0=x0, 1=y0, 2=z0, 3=x1, 4=y1, 5=z1).
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
    /// Compare name, coordinates to 1e-6, width and linecolor; guid ignored.
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
    /// Translate in place.
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
    /// Translate back in place.
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
    /// Scale both ends in place.
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
    /// Divide both ends in place.
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

    /// Return a translated copy.
    fn add(self, other: &Vector) -> Line {
        let mut result = self;
        result += other;

        result
    }
}

impl Sub<&Vector> for Line {
    type Output = Line;

    /// Return a copy translated back.
    fn sub(self, other: &Vector) -> Line {
        let mut result = self;
        result -= other;

        result
    }
}

impl Mul<f64> for Line {
    type Output = Line;

    /// Return a copy with both ends scaled.
    fn mul(self, factor: f64) -> Line {
        let mut result = self;
        result *= factor;

        result
    }
}

impl Div<f64> for Line {
    type Output = Line;

    /// Return a copy with both ends divided.
    fn div(self, factor: f64) -> Line {
        let mut result = self;
        result /= factor;

        result
    }
}

impl Neg for Line {
    type Output = Line;

    /// Return a flipped copy (end to start).
    fn neg(self) -> Line {
        Line::new(self._x1, self._y1, self._z1, self._x0, self._y0, self._z0)
    }
}

impl Add<&Vector> for &Line {
    type Output = Line;

    /// Return a translated copy.
    fn add(self, other: &Vector) -> Line {
        self.duplicate() + other
    }
}

impl Sub<&Vector> for &Line {
    type Output = Line;

    /// Return a copy translated back.
    fn sub(self, other: &Vector) -> Line {
        self.duplicate() - other
    }
}

impl Mul<f64> for &Line {
    type Output = Line;

    /// Return a copy with both ends scaled.
    fn mul(self, factor: f64) -> Line {
        self.duplicate() * factor
    }
}

impl Div<f64> for &Line {
    type Output = Line;

    /// Return a copy with both ends divided.
    fn div(self, factor: f64) -> Line {
        self.duplicate() / factor
    }
}

impl Neg for &Line {
    type Output = Line;

    /// Return a flipped copy (end to start).
    fn neg(self) -> Line {
        Line::new(self._x1, self._y1, self._z1, self._x0, self._y0, self._z0)
    }
}

impl Line {
    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform in place.
    pub fn transform(&mut self, xform: &Xform) {
        let s = self.start().transformed(xform);
        let e = self.end().transformed(xform);

        self._x0 = s[0];
        self._y0 = s[1];
        self._z0 = s[2];
        self._x1 = e[0];
        self._y1 = e[1];
        self._z1 = e[2];
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
    /// Return the length.
    pub fn length(&self) -> f64 {
        self.squared_length().sqrt()
    }

    /// Return the squared length.
    pub fn squared_length(&self) -> f64 {
        self.to_vector().magnitude_squared()
    }

    /// Return the vector from start to end.
    pub fn to_vector(&self) -> Vector {
        Vector::new(
            self._x1 - self._x0,
            self._y1 - self._y0,
            self._z1 - self._z0,
        )
    }

    /// Return the unit vector from start to end.
    pub fn to_direction(&self) -> Vector {
        self.to_vector().normalized()
    }

    /// Return the start point.
    pub fn start(&self) -> Point {
        Point::new(self._x0, self._y0, self._z0)
    }

    /// Return the end point.
    pub fn end(&self) -> Point {
        Point::new(self._x1, self._y1, self._z1)
    }

    /// Return the midpoint.
    pub fn center(&self) -> Point {
        Point::new(
            (self._x0 + self._x1) * 0.5,
            (self._y0 + self._y1) * 0.5,
            (self._z0 + self._z1) * 0.5,
        )
    }

    /// Return the point at parameter t (0 = start, 1 = end).
    pub fn point_at(&self, t: f64) -> Point {
        let s = 1.0 - t;

        Point::new(
            s * self._x0 + t * self._x1,
            s * self._y0 + t * self._y1,
            s * self._z0 + t * self._z1,
        )
    }

    /// Return n evenly spaced points including both ends.
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

    /// Return points spaced approximately distance apart including both ends.
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

    /// Return the parameter and closest point; limited clamps t to [0, 1].
    pub fn closest_point(&self, point: &Point, limited: bool) -> (f64, Point) {
        let d = self.to_vector();
        let len_sq = d.magnitude_squared();

        if len_sq < 1e-20 {
            return (0.0, self.start());
        }

        let mut t = (point - &self.start()).dot(&d) / len_sq;

        if limited {
            t = t.clamp(0.0, 1.0);
        }

        (t, self.point_at(t))
    }

    /// Compute the midpoints of the paired starts and ends.
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

    /// Compute the extreme sub-segment of line spanned by the projected points; None when empty.
    pub fn from_projected_points(line: &Line, points: &[Point]) -> Option<Line> {
        let (output_start, output_end) =
            Polyline::line_from_projected_points(&line.start(), &line.end(), points)?;

        Some(Line::from_points(&output_start, &output_end))
    }

    /// Split lines and boundary lines in xy at every crossing within tolerance, lines shorter than tolerance dropped, a collinear overlap kept by the boundary, else by the earlier line; split points within merge welded onto boundary ends, then boundary crossings, then the rest; dangling pieces dropped. Returns the pieces, a piece two lines share kept once, and the index of the line of each, boundary lines numbered after lines.
    pub fn split_at_crossings(
        lines: &[Line],
        boundary: &[Line],
        tolerance: f64,
        merge: f64,
    ) -> (Vec<Line>, Vec<usize>) {
        let mut segments = Vec::new();

        for i in 0..lines.len() + boundary.len() {
            let line = if i < lines.len() {
                &lines[i]
            } else {
                &boundary[i - lines.len()]
            };
            let start = Point::new(line.start()[0], line.start()[1], 0.0);
            let end = Point::new(line.end()[0], line.end()[1], 0.0);

            if split_distance(&start, &end) >= tolerance {
                segments.push(SplitSegment {
                    start,
                    end,
                    source: i,
                    boundary: i >= lines.len(),
                    alive: true,
                    stops: Vec::new(),
                });
            }
        }

        split_overlaps(&mut segments, tolerance);
        let stops = split_stops(&mut segments, tolerance);
        let (points, kept) = split_welds(&stops, merge);
        let (pairs, sources) = split_pieces(&mut segments, &kept);
        let (pairs, sources) = split_pruned(pairs, sources, points.len());

        let mut pieces = Vec::new();

        for pair in &pairs {
            pieces.push(Line::from_points(&points[pair.0], &points[pair.1]));
        }

        (pieces, sources)
    }

    /// Compute the collinear overlap with other; None when none or a single point.
    pub fn overlap(&self, other: &Line) -> Option<Line> {
        let (output_start, output_end) =
            Polyline::line_line_overlap(&self.start(), &self.end(), &other.start(), &other.end())?;

        Some(Line::from_points(&output_start, &output_end))
    }

    /// Compute the longer of the two midpoint pairings of overlap(other) and other.overlap(self); None when empty.
    pub fn overlap_average(&self, other: &Line) -> Option<Line> {
        let (output_start, output_end) = Polyline::line_line_overlap_average(
            &self.start(),
            &self.end(),
            &other.start(),
            &other.end(),
        );

        let out = Line::from_points(&output_start, &output_end);

        if out.squared_length() <= 0.0 {
            return None;
        }

        Some(out)
    }

    /// Grow start by ext_start and end by ext_end.
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

    /// Grow both ends by dist, or by proportion of the length when non-zero.
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

    /// Shrink both ends by dist as a fraction of the length.
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
    /// Serialize to a sorted JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::file_json_dumps(self, false)
    }

    /// Deserialize from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().expect("Failed to serialize Line JSON")
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).expect("Failed to parse Line JSON")
    }

    /// Write JSON to a file.
    pub fn file_json_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        crate::file_encoders::file_json_dump(self, filename, true)
    }

    /// Read JSON from a file.
    pub fn file_json_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filename)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
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

    /// Construct from the protobuf message.
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

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Ok(Self::from_proto(crate::proto::Line::decode(data)?))
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filename, self.pb_dumps())?;

        Ok(())
    }

    /// Read from a protobuf file.
    pub fn pb_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::pb_loads(&std::fs::read(filename)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "x0, y0, z0, x1, y1, z1".
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

    /// Return "Line(name, x0, y0, z0, x1, y1, z1, Color(...), width)".
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
    /// Write the line string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Crossings
// ═══════════════════════════════════════════════════════════════════════════
/// A piece of an input line while the crossings are computed.
#[derive(Clone)]
struct SplitSegment {
    start: Point,             // Start at z 0.
    end: Point,               // End at z 0.
    source: usize,            // Index of the input line, boundary lines after lines.
    boundary: bool,           // True for a boundary line.
    alive: bool,              // False once a stronger collinear segment took it.
    stops: Vec<(f64, usize)>, // Distance along it and index of each of its stops.
}

/// A point every piece ends on: a segment end or a crossing.
struct SplitStop {
    point: Point, // At z 0.
    order: i32,   // Weld order: 0 boundary end, 1 boundary crossing, 2 rest.
}

/// Unit xy direction from start to end.
fn split_direction(start: &Point, end: &Point) -> Vector {
    Vector::new(end[0] - start[0], end[1] - start[1], 0.0).normalized()
}

/// Distance in xy from start to end.
fn split_distance(start: &Point, end: &Point) -> f64 {
    (end[0] - start[0]).hypot(end[1] - start[1])
}

/// True when segment strong takes a collinear overlap from weak: the boundary first, then the earlier line.
fn split_is_stronger(strong: &SplitSegment, weak: &SplitSegment) -> bool {
    if strong.boundary != weak.boundary {
        return strong.boundary;
    }

    strong.source < weak.source
}

/// Distance along a segment from its start to the foot of a point.
fn split_parameter(segment: &SplitSegment, point: &Point) -> f64 {
    (point - &segment.start).dot(&split_direction(&segment.start, &segment.end))
}

/// The weak segment of a collinear pair loses the stretch the strong one covers and keeps the rest as new pieces.
fn split_overlap(segments: &mut Vec<SplitSegment>, strong: usize, weak: usize, tolerance: f64) {
    let along = split_direction(&segments[strong].start, &segments[strong].end);
    let direction = split_direction(&segments[weak].start, &segments[weak].end);

    if along.cross(&direction)[2].abs() > Tolerance::ANGULAR
        || (&segments[weak].start - &segments[strong].start).cross(&along)[2].abs() > tolerance
    {
        return;
    }

    let length = split_distance(&segments[weak].start, &segments[weak].end);
    let low = 0.0f64.max(
        split_parameter(&segments[weak], &segments[strong].start)
            .min(split_parameter(&segments[weak], &segments[strong].end)),
    );
    let high = length.min(
        split_parameter(&segments[weak], &segments[strong].start)
            .max(split_parameter(&segments[weak], &segments[strong].end)),
    );

    if high - low <= tolerance {
        return;
    }

    let loser = segments[weak].clone();
    segments[weak].alive = false;

    if low > tolerance {
        segments.push(SplitSegment {
            start: loser.start.clone(),
            end: &loser.start + &(&direction * low),
            source: loser.source,
            boundary: loser.boundary,
            alive: true,
            stops: Vec::new(),
        });
    }

    if length - high > tolerance {
        segments.push(SplitSegment {
            start: &loser.start + &(&direction * high),
            end: loser.end.clone(),
            source: loser.source,
            boundary: loser.boundary,
            alive: true,
            stops: Vec::new(),
        });
    }
}

/// Collinear overlaps resolved over every pair, the weaker segment of each giving way.
fn split_overlaps(segments: &mut Vec<SplitSegment>, tolerance: f64) {
    let mut i = 0;

    while i < segments.len() {
        let mut j = 0;

        while j < segments.len() {
            if i != j
                && segments[i].alive
                && segments[j].alive
                && !split_is_stronger(&segments[j], &segments[i])
            {
                split_overlap(segments, i, j, tolerance);
            }

            j += 1;
        }

        i += 1;
    }
}

/// The crossing of segments first and second as a stop on both, when they cross within tolerance.
fn split_crossing(
    segments: &mut [SplitSegment],
    first: usize,
    second: usize,
    tolerance: f64,
    stops: &mut Vec<SplitStop>,
) {
    let along = &segments[first].end - &segments[first].start;
    let across = &segments[second].end - &segments[second].start;
    let denominator = along.cross(&across)[2];

    if denominator.abs() < Tolerance::ABSOLUTE * along.magnitude() * across.magnitude() {
        return;
    }

    let offset = &segments[second].start - &segments[first].start;
    let on_first = offset.cross(&across)[2] / denominator;
    let on_second = offset.cross(&along)[2] / denominator;

    if on_first < -tolerance / along.magnitude()
        || on_first > 1.0 + tolerance / along.magnitude()
        || on_second < -tolerance / across.magnitude()
        || on_second > 1.0 + tolerance / across.magnitude()
    {
        return;
    }

    let order = if segments[first].boundary || segments[second].boundary {
        1
    } else {
        2
    };
    segments[first]
        .stops
        .push((on_first.clamp(0.0, 1.0) * along.magnitude(), stops.len()));
    segments[second]
        .stops
        .push((on_second.clamp(0.0, 1.0) * across.magnitude(), stops.len()));
    stops.push(SplitStop {
        point: &segments[first].start + &(&along * on_first.clamp(0.0, 1.0)),
        order,
    });
}

/// The stops of every live segment: its ends, then every crossing with a later one.
fn split_stops(segments: &mut [SplitSegment], tolerance: f64) -> Vec<SplitStop> {
    let mut stops = Vec::new();

    for segment in segments.iter_mut() {
        if !segment.alive {
            continue;
        }

        let order = if segment.boundary { 0 } else { 2 };
        segment.stops.push((0.0, stops.len()));
        stops.push(SplitStop {
            point: segment.start.clone(),
            order,
        });
        segment
            .stops
            .push((split_distance(&segment.start, &segment.end), stops.len()));
        stops.push(SplitStop {
            point: segment.end.clone(),
            order,
        });
    }

    for i in 0..segments.len() {
        for j in i + 1..segments.len() {
            if segments[i].alive && segments[j].alive {
                split_crossing(segments, i, j, tolerance, &mut stops);
            }
        }
    }

    stops
}

/// Stops within merge of an earlier stop in priority order welded onto it: the kept points and the kept index of every stop.
fn split_welds(stops: &[SplitStop], merge: f64) -> (Vec<Point>, Vec<usize>) {
    let mut points: Vec<Point> = Vec::new();
    let mut kept = vec![0; stops.len()];

    for order in 0..3 {
        for index in 0..stops.len() {
            if stops[index].order != order {
                continue;
            }

            let mut found = points.len();

            for (k, point) in points.iter().enumerate() {
                if split_distance(point, &stops[index].point) <= merge {
                    found = k;
                    break;
                }
            }

            if found == points.len() {
                points.push(stops[index].point.clone());
            }

            kept[index] = found;
        }
    }

    (points, kept)
}

/// Pieces of every live segment between consecutive stops as welded vertex pairs with their source, each pair once.
fn split_pieces(
    segments: &mut [SplitSegment],
    canonical: &[usize],
) -> (Vec<(usize, usize)>, Vec<usize>) {
    let mut seen: BTreeSet<(usize, usize)> = BTreeSet::new();
    let mut pairs: Vec<(usize, usize)> = Vec::new();
    let mut sources: Vec<usize> = Vec::new();

    for segment in segments.iter_mut() {
        segment
            .stops
            .sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));

        if !segment.alive {
            continue;
        }

        for k in 0..segment.stops.len().saturating_sub(1) {
            let first = canonical[segment.stops[k].1];
            let second = canonical[segment.stops[k + 1].1];
            let piece = (first.min(second), first.max(second));

            if piece.0 != piece.1 && seen.insert(piece) {
                pairs.push(piece);
                sources.push(segment.source);
            }
        }
    }

    (pairs, sources)
}

/// Pieces with a dangling end removed until every end is shared.
fn split_pruned(
    mut pairs: Vec<(usize, usize)>,
    mut sources: Vec<usize>,
    vertices: usize,
) -> (Vec<(usize, usize)>, Vec<usize>) {
    for _ in 0..=pairs.len() {
        let mut degree = vec![0; vertices];

        for piece in &pairs {
            degree[piece.0] += 1;
            degree[piece.1] += 1;
        }

        let mut kept_pairs = Vec::new();
        let mut kept_sources = Vec::new();

        for k in 0..pairs.len() {
            if degree[pairs[k].0] >= 2 && degree[pairs[k].1] >= 2 {
                kept_pairs.push(pairs[k]);
                kept_sources.push(sources[k]);
            }
        }

        let pruned = kept_pairs.len() != pairs.len();
        pairs = kept_pairs;
        sources = kept_sources;

        if !pruned {
            break;
        }
    }

    (pairs, sources)
}
