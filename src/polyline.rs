use crate::{Color, Plane, Point, Tolerance, Vector, Xform};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde_json::Value;
use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};
use uuid::Uuid;

/// A polyline defined by a collection of coordinates with an associated plane.
///
/// Internally stores coordinates as a flat array [x0, y0, z0, x1, y1, z1, ...] for
/// efficient serialization. Provides Point-based API for compatibility.
#[derive(Debug, Clone)]
pub struct Polyline {
    pub guid: String,
    pub name: String,
    /// Flat coordinate array [x0, y0, z0, x1, y1, z1, ...]
    pub coords: Vec<f64>,
    pub plane: Plane,
    pub width: f64,
    pub linecolor: Color,
    pub xform: Xform,
}

impl Default for Polyline {
    fn default() -> Self {
        Self {
            guid: Uuid::new_v4().to_string(),
            name: "my_polyline".to_string(),
            coords: Vec::new(),
            plane: Plane::default(),
            width: 1.0,
            linecolor: Color::white(),
            xform: Xform::identity(),
        }
    }
}

impl Polyline {
    /// Creates a new `Polyline` with default guid and name.
    ///
    /// # Arguments
    ///
    /// * `points` - The collection of points (converted to flat coords internally).
    pub fn new(points: Vec<Point>) -> Self {
        // Convert points to flat coords
        let mut coords = Vec::with_capacity(points.len() * 3);
        for p in &points {
            coords.push(p[0]);
            coords.push(p[1]);
            coords.push(p[2]);
        }
        
        // Delegate plane computation to Plane::from_points
        let plane = if points.len() >= 3 {
            Plane::from_points(points)
        } else {
            Plane::default()
        };

        Self {
            guid: Uuid::new_v4().to_string(),
            name: "my_polyline".to_string(),
            coords,
            plane,
            width: 1.0,
            linecolor: Color::white(),
            xform: Xform::identity(),
        }
    }

    /// Creates a Polyline from a flat coordinate array.
    pub fn from_coords(coords: Vec<f64>) -> Self {
        let mut pl = Self {
            guid: Uuid::new_v4().to_string(),
            name: "my_polyline".to_string(),
            coords,
            plane: Plane::default(),
            width: 1.0,
            linecolor: Color::white(),
            xform: Xform::identity(),
        };
        pl.recompute_plane_if_needed();
        pl
    }

    /// Returns detailed string representation (like Python __repr__).
    pub fn repr(&self) -> String {
        format!("Polyline({}, {} points)", self.name, self.point_count())
    }

    /// Creates a deep copy with a new GUID.
    pub fn duplicate(&self) -> Self {
        Self {
            guid: Uuid::new_v4().to_string(),
            name: self.name.clone(),
            coords: self.coords.clone(),
            plane: self.plane.clone(),
            width: self.width,
            linecolor: self.linecolor.clone(),
            xform: self.xform.clone(),
        }
    }

    /// Returns the number of points in the polyline.
    pub fn point_count(&self) -> usize {
        self.coords.len() / 3
    }

    /// Returns the number of points in the polyline (alias for point_count).
    pub fn len(&self) -> usize {
        self.point_count()
    }

    /// Returns true if the polyline has no points.
    pub fn is_empty(&self) -> bool {
        self.coords.is_empty()
    }

    /// Returns the number of segments in the polyline.
    /// A polyline with n points has n-1 segments.
    pub fn segment_count(&self) -> usize {
        let n = self.point_count();
        if n > 1 { n - 1 } else { 0 }
    }

    /// Returns all points as Point objects.
    pub fn get_points(&self) -> Vec<Point> {
        let mut points = Vec::with_capacity(self.point_count());
        for i in 0..self.point_count() {
            let idx = i * 3;
            points.push(Point::new(
                self.coords[idx],
                self.coords[idx + 1],
                self.coords[idx + 2],
            ));
        }
        points
    }

    /// Calculates the total length of the polyline.
    pub fn length(&self) -> f64 {
        let mut total_length = 0.0;
        for i in 0..self.segment_count() {
            let idx0 = i * 3;
            let idx1 = (i + 1) * 3;
            let dx = self.coords[idx1] - self.coords[idx0];
            let dy = self.coords[idx1 + 1] - self.coords[idx0 + 1];
            let dz = self.coords[idx1 + 2] - self.coords[idx0 + 2];
            total_length += (dx * dx + dy * dy + dz * dz).sqrt();
        }
        total_length
    }

    /// Returns a copy of the point at the given index.
    pub fn get_point(&self, index: usize) -> Option<Point> {
        if index < self.point_count() {
            let idx = index * 3;
            Some(Point::new(
                self.coords[idx],
                self.coords[idx + 1],
                self.coords[idx + 2],
            ))
        } else {
            None
        }
    }

    /// Sets the point at the given index.
    pub fn set_point(&mut self, index: usize, point: &Point) {
        if index < self.point_count() {
            let idx = index * 3;
            self.coords[idx] = point[0];
            self.coords[idx + 1] = point[1];
            self.coords[idx + 2] = point[2];
        }
    }

    /// Adds a point to the end of the polyline.
    pub fn add_point(&mut self, point: Point) {
        self.coords.push(point[0]);
        self.coords.push(point[1]);
        self.coords.push(point[2]);
        // Recompute plane if we have at least 3 points
        if self.point_count() == 3 {
            self.recompute_plane_if_needed();
        }
    }

    /// Inserts a point at the specified index.
    pub fn insert_point(&mut self, index: usize, point: Point) {
        let idx = index * 3;
        if idx <= self.coords.len() {
            self.coords.insert(idx, point[2]);
            self.coords.insert(idx, point[1]);
            self.coords.insert(idx, point[0]);
            // Recompute plane if we have at least 3 points
            if self.point_count() == 3 {
                self.recompute_plane_if_needed();
            }
        }
    }

    /// Removes and returns the point at the specified index.
    pub fn remove_point(&mut self, index: usize) -> Option<Point> {
        if index < self.point_count() {
            let idx = index * 3;
            let z = self.coords.remove(idx + 2);
            let y = self.coords.remove(idx + 1);
            let x = self.coords.remove(idx);
            // Recompute plane if we still have at least 3 points
            if self.point_count() == 3 {
                self.recompute_plane_if_needed();
            }
            Some(Point::new(x, y, z))
        } else {
            None
        }
    }

    /// Reverses the order of points in the polyline.
    pub fn reverse(&mut self) {
        let n = self.point_count();
        if n <= 1 {
            return;
        }
        // Reverse in groups of 3
        let mut new_coords = Vec::with_capacity(self.coords.len());
        for i in (0..n).rev() {
            let idx = i * 3;
            new_coords.push(self.coords[idx]);
            new_coords.push(self.coords[idx + 1]);
            new_coords.push(self.coords[idx + 2]);
        }
        self.coords = new_coords;
        self.plane.reverse();
    }

    /// Returns a new polyline with reversed point order.
    pub fn reversed(&self) -> Self {
        let mut reversed = self.clone();
        reversed.reverse();
        reversed
    }

    pub fn transform(&mut self) {
        let xform = self.xform.clone();
        let points = self.get_points();
        self.coords.clear();
        for mut pt in points {
            xform.transform_point(&mut pt);
            self.coords.push(pt[0]);
            self.coords.push(pt[1]);
            self.coords.push(pt[2]);
        }
        self.xform = Xform::identity();
    }

    pub fn transformed(&self) -> Self {
        let mut result = self.clone();
        result.transform();
        result
    }

     /// Recompute plane if we have at least 3 points
     fn recompute_plane_if_needed(&mut self) {
         if self.point_count() >= 3 {
             self.plane = Plane::from_points(self.get_points());
         }
     }

 }

 impl Serialize for Polyline {
     fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
     where
         S: Serializer,
     {
         use serde::ser::SerializeMap;
         let mut map = serializer.serialize_map(Some(7))?;
         map.serialize_entry("type", "Polyline")?;
         map.serialize_entry("guid", &self.guid)?;
         map.serialize_entry("name", &self.name)?;
         map.serialize_entry("coords", &self.coords)?;
         map.serialize_entry("width", &self.width)?;
         map.serialize_entry("linecolor", &self.linecolor)?;
         map.serialize_entry("xform", &self.xform)?;
         map.end()
     }
 }

 impl<'de> Deserialize<'de> for Polyline {
     fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
     where
         D: Deserializer<'de>,
     {
         let value: Value = Value::deserialize(deserializer)?;

         let guid = value
             .get("guid")
             .and_then(|v| v.as_str())
             .map(|s| s.to_string())
             .unwrap_or_else(|| Uuid::new_v4().to_string());

         let name = value
             .get("name")
             .and_then(|v| v.as_str())
             .map(|s| s.to_string())
             .unwrap_or_else(|| "my_polyline".to_string());

         // Support both coords format and legacy points format
         let coords = if let Some(coords_val) = value.get("coords") {
             coords_val
                 .as_array()
                 .map(|arr| arr.iter().filter_map(|v| v.as_f64()).collect())
                 .unwrap_or_default()
         } else if let Some(points_val) = value.get("points") {
             // Legacy format with full Point objects
             let mut coords = Vec::new();
             if let Some(arr) = points_val.as_array() {
                 for pt_val in arr {
                     if let (Some(x), Some(y), Some(z)) = (
                         pt_val.get("x").or_else(|| pt_val.get("_x")).and_then(|v| v.as_f64()),
                         pt_val.get("y").or_else(|| pt_val.get("_y")).and_then(|v| v.as_f64()),
                         pt_val.get("z").or_else(|| pt_val.get("_z")).and_then(|v| v.as_f64()),
                     ) {
                         coords.push(x);
                         coords.push(y);
                         coords.push(z);
                     }
                 }
             }
             coords
         } else {
             Vec::new()
         };

         let width = value.get("width").and_then(|v| v.as_f64()).unwrap_or(1.0);

         let linecolor = value
             .get("linecolor")
             .map(|v| serde_json::from_value(v.clone()).unwrap_or_else(|_| Color::white()))
             .unwrap_or_else(Color::white);

         let xform = value
             .get("xform")
             .map(|v| serde_json::from_value(v.clone()).unwrap_or_else(|_| Xform::identity()))
             .unwrap_or_else(Xform::identity);

         let mut polyline = Polyline {
             guid,
             name,
             coords,
             plane: Plane::default(),
             width,
             linecolor,
             xform,
         };
         polyline.recompute_plane_if_needed();
         Ok(polyline)
     }
 }

 impl Polyline {

     /// Serializes the Polyline to a JSON string.
     pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
         let mut buf = Vec::new();
         let formatter = serde_json::ser::PrettyFormatter::with_indent(b"    ");
         let mut ser = serde_json::Serializer::with_formatter(&mut buf, formatter);
         self.serialize(&mut ser)?;
         Ok(String::from_utf8(buf)?)
     }

    /// Deserializes a Polyline from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serializes the Polyline to a JSON file.
    pub fn to_json(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    /// Deserializes a Polyline from a JSON file.
    pub fn from_json(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    #[cfg(feature = "protobuf")]
    /// Convert to protobuf binary format.
    ///
    /// # Returns
    ///
    /// A Vec<u8> containing the serialized protobuf data.
    pub fn to_protobuf(&self) -> Vec<u8> {
        use prost::Message;

        let proto = crate::proto::Polyline {
            guid: self.guid.clone(),
            name: self.name.clone(),
            coords: self.coords.clone(),
            width: self.width,
            linecolor: Some(crate::proto::Color {
                guid: self.linecolor.guid.clone(),
                name: self.linecolor.name.clone(),
                r: self.linecolor.r as i32,
                g: self.linecolor.g as i32,
                b: self.linecolor.b as i32,
                a: self.linecolor.a as i32,
            }),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid.clone(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
        };
        proto.encode_to_vec()
    }

    #[cfg(feature = "protobuf")]
    /// Create Polyline from protobuf binary data.
    ///
    /// # Arguments
    ///
    /// * `data` - Byte slice containing protobuf-encoded polyline data.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized Polyline or an error.
    pub fn from_protobuf(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        let proto = crate::proto::Polyline::decode(data)?;

        let mut pl = Self::from_coords(proto.coords);
        pl.guid = proto.guid;
        pl.name = proto.name;
        pl.width = proto.width;

        if let Some(color) = proto.linecolor {
            pl.linecolor.guid = color.guid;
            pl.linecolor.name = color.name;
            pl.linecolor.r = color.r as u8;
            pl.linecolor.g = color.g as u8;
            pl.linecolor.b = color.b as u8;
            pl.linecolor.a = color.a as u8;
        }

        if let Some(xform) = proto.xform {
            pl.xform.guid = xform.guid;
            pl.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 {
                    pl.xform.m[i] = *val;
                }
            }
        }

        Ok(pl)
    }

    #[cfg(feature = "protobuf")]
    /// Write protobuf to file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the output file.
    pub fn protobuf_dump(&self, filepath: &str) {
        let data = self.to_protobuf();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    #[cfg(feature = "protobuf")]
    /// Read protobuf from file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the protobuf file.
    ///
    /// # Returns
    ///
    /// The deserialized Polyline.
    pub fn protobuf_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::from_protobuf(&data).expect("Failed to parse protobuf")
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Geometric Utilities
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Shift polyline points by specified number of positions
    pub fn shift(&mut self, times: i32) {
        if self.coords.is_empty() {
            return;
        }
        let n = self.point_count();
        let shift_amount = ((times % n as i32) + n as i32) % n as i32;
        // Rotate coords in groups of 3
        let mut new_coords = Vec::with_capacity(self.coords.len());
        for i in 0..n {
            let src_idx = ((i + shift_amount as usize) % n) * 3;
            new_coords.push(self.coords[src_idx]);
            new_coords.push(self.coords[src_idx + 1]);
            new_coords.push(self.coords[src_idx + 2]);
        }
        self.coords = new_coords;
    }

    /// Calculate squared length of polyline (faster, no sqrt)
    pub fn magnitude_squared(&self) -> f64 {
        let mut length = 0.0f64;
        for i in 0..self.segment_count() {
            let idx0 = i * 3;
            let idx1 = (i + 1) * 3;
            let dx = self.coords[idx1] - self.coords[idx0];
            let dy = self.coords[idx1 + 1] - self.coords[idx0 + 1];
            let dz = self.coords[idx1 + 2] - self.coords[idx0 + 2];
            length += dx * dx + dy * dy + dz * dz;
        }
        length
    }

    /// Get point at parameter t along a line segment (t=0 is start, t=1 is end)
    pub fn point_at(start: &Point, end: &Point, t: f64) -> Point {
        let s = 1.0 - t;
        let t_f32 = t;
        let s_f32 = s;
        Point::new(
            if start[0] == end[0] {
                start[0]
            } else {
                s_f32 * start[0] + t_f32 * end[0]
            },
            if start[1] == end[1] {
                start[1]
            } else {
                s_f32 * start[1] + t_f32 * end[1]
            },
            if start[2] == end[2] {
                start[2]
            } else {
                s_f32 * start[2] + t_f32 * end[2]
            },
        )
    }

    /// Find closest point on line segment to given point, returns parameter t
    pub fn closest_point_to_line(point: &Point, line_start: &Point, line_end: &Point) -> f64 {
        let d = line_end.clone() - line_start.clone();
        let dod = d.magnitude_squared();

        if dod > 0.0 {
            if (point.clone() - line_start.clone()).magnitude_squared()
                <= (point.clone() - line_end.clone()).magnitude_squared()
            {
                (point.clone() - line_start.clone()).dot(&d) / dod
            } else {
                1.0 + (point.clone() - line_end.clone()).dot(&d) / dod
            }
        } else {
            0.0
        }
    }

    /// Check if two line segments overlap and return the overlapping segment
    pub fn line_line_overlap(
        line0_start: &Point,
        line0_end: &Point,
        line1_start: &Point,
        line1_end: &Point,
    ) -> Option<(Point, Point)> {
        let mut t = [0.0, 1.0, 0.0, 0.0];
        t[2] = Self::closest_point_to_line(line1_start, line0_start, line0_end);
        t[3] = Self::closest_point_to_line(line1_end, line0_start, line0_end);

        let do_overlap = !((t[2] < 0.0 && t[3] < 0.0) || (t[2] > 1.0 && t[3] > 1.0));
        t.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let overlap_valid = (t[2] - t[1]).abs() > Tolerance::ZERO_TOLERANCE;

        if do_overlap && overlap_valid {
            Some((
                Self::point_at(line0_start, line0_end, t[1]),
                Self::point_at(line0_start, line0_end, t[2]),
            ))
        } else {
            None
        }
    }

    /// Calculate average of two line segments
    pub fn line_line_average(
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

    /// Calculate overlap average of two line segments
    pub fn line_line_overlap_average(
        line0_start: &Point,
        line0_end: &Point,
        line1_start: &Point,
        line1_end: &Point,
    ) -> (Point, Point) {
        let line_a = Self::line_line_overlap(line0_start, line0_end, line1_start, line1_end);
        let line_b = Self::line_line_overlap(line1_start, line1_end, line0_start, line0_end);

        if let (Some((line_a_start, line_a_end)), Some((line_b_start, line_b_end))) =
            (line_a, line_b)
        {
            let mid_line0_start = Point::new(
                (line_a_start[0] + line_b_start[0]) * 0.5,
                (line_a_start[1] + line_b_start[1]) * 0.5,
                (line_a_start[2] + line_b_start[2]) * 0.5,
            );
            let mid_line0_end = Point::new(
                (line_a_end[0] + line_b_end[0]) * 0.5,
                (line_a_end[1] + line_b_end[1]) * 0.5,
                (line_a_end[2] + line_b_end[2]) * 0.5,
            );
            let mid_line1_start = Point::new(
                (line_a_start[0] + line_b_end[0]) * 0.5,
                (line_a_start[1] + line_b_end[1]) * 0.5,
                (line_a_start[2] + line_b_end[2]) * 0.5,
            );
            let mid_line1_end = Point::new(
                (line_a_end[0] + line_b_start[0]) * 0.5,
                (line_a_end[1] + line_b_start[1]) * 0.5,
                (line_a_end[2] + line_b_start[2]) * 0.5,
            );

            let mid0_vec = mid_line0_end.clone() - mid_line0_start.clone();
            let mid1_vec = mid_line1_end.clone() - mid_line1_start.clone();

            if mid0_vec.magnitude_squared() > mid1_vec.magnitude_squared() {
                (mid_line0_start, mid_line0_end)
            } else {
                (mid_line1_start, mid_line1_end)
            }
        } else {
            Self::line_line_average(line0_start, line0_end, line1_start, line1_end)
        }
    }

    /// Create line from projected points onto a base line
    pub fn line_from_projected_points(
        line_start: &Point,
        line_end: &Point,
        points: &[Point],
    ) -> Option<(Point, Point)> {
        if points.is_empty() {
            return None;
        }

        let mut t_values: Vec<f64> = points
            .iter()
            .map(|p| Self::closest_point_to_line(p, line_start, line_end))
            .collect();

        t_values.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let output_start = Self::point_at(line_start, line_end, t_values[0]);
        let output_end =
            Self::point_at(line_start, line_end, t_values[t_values.len() - 1]);

        if (t_values[0] - t_values[t_values.len() - 1]).abs() > Tolerance::ZERO_TOLERANCE {
            Some((output_start, output_end))
        } else {
            None
        }
    }

    /// Find closest distance and point from a point to this polyline
    pub fn closest_distance_and_point(&self, point: &Point) -> (f64, usize, Point) {
        let mut edge_id = 0;
        let mut closest_distance = f64::MAX;
        let mut best_t = 0.0;
        let points = self.get_points();

        for i in 0..self.segment_count() {
            let t = Self::closest_point_to_line(point, &points[i], &points[i + 1]);
            let point_on_segment = Self::point_at(&points[i], &points[i + 1], t);
            let distance = point.distance(&point_on_segment, None);

            if distance < closest_distance {
                closest_distance = distance;
                edge_id = i;
                best_t = t;
            }

            if closest_distance < Tolerance::ZERO_TOLERANCE {
                break;
            }
        }

        let closest_point = Self::point_at(&points[edge_id], &points[edge_id + 1], best_t);
        (closest_distance, edge_id, closest_point)
    }

    /// Check if polyline is closed (first and last points are the same)
    pub fn is_closed(&self) -> bool {
        let n = self.point_count();
        if n < 2 {
            return false;
        }
        let first = self.get_point(0).unwrap();
        let last = self.get_point(n - 1).unwrap();
        first.distance(&last, None) < Tolerance::ZERO_TOLERANCE
    }

    /// Calculate center point of polyline
    pub fn center(&self) -> Point {
        if self.coords.is_empty() {
            return Point::new(0.0, 0.0, 0.0);
        }

        let total = self.point_count();
        let n = if self.is_closed() && total > 1 { total - 1 } else { total };

        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_z = 0.0;

        for i in 0..n {
            let idx = i * 3;
            sum_x += self.coords[idx];
            sum_y += self.coords[idx + 1];
            sum_z += self.coords[idx + 2];
        }

        Point::new(sum_x / n as f64, sum_y / n as f64, sum_z / n as f64)
    }

    /// Calculate center as vector
    pub fn center_vec(&self) -> Vector {
        let center = self.center();
        Vector::new(center[0], center[1], center[2])
    }

    /// Get average plane from polyline points
    pub fn get_average_plane(&self) -> (Point, Vector, Vector, Vector) {
        let origin = self.center();
        let points = self.get_points();

        let x_axis = if points.len() >= 2 {
            let mut x = points[1].clone() - points[0].clone();
            x.normalize();
            x
        } else {
            Vector::new(1.0, 0.0, 0.0)
        };

        let z_axis = self.average_normal();
        let mut y_axis = z_axis.cross(&x_axis);
        y_axis.normalize();

        (origin, x_axis, y_axis, z_axis)
    }

    /// Get fast plane calculation from polyline
    pub fn get_fast_plane(&self) -> (Point, Plane) {
        let origin = if !self.coords.is_empty() {
            self.get_point(0).unwrap()
        } else {
            Point::new(0.0, 0.0, 0.0)
        };

        let average_normal = self.average_normal();
        let plane = Plane::from_point_normal(origin.clone(), average_normal);
        (origin, plane)
    }

    /// Extend line segment by specified distances at both ends
    pub fn extend_line(
        line_start: &mut Point,
        line_end: &mut Point,
        distance0: f64,
        distance1: f64,
    ) {
        let mut v = line_end.clone() - line_start.clone();
        v.normalize();

        *line_start = line_start.clone() - (v.clone() * distance0);
        *line_end = line_end.clone() + (v * distance1);
    }

    /// Scale line segment inward by specified distance
    pub fn scale_line(line_start: &mut Point, line_end: &mut Point, distance: f64) {
        let v = line_end.clone() - line_start.clone();
        *line_start = line_start.clone() + (v.clone() * distance);
        *line_end = line_end.clone() - (v * distance);
    }

    /// Extend polyline segment
    pub fn extend_segment(
        &mut self,
        segment_id: usize,
        dist0: f64,
        dist1: f64,
        proportion0: f64,
        proportion1: f64,
    ) {
        if segment_id >= self.segment_count() {
            return;
        }

        let mut p0 = self.get_point(segment_id).unwrap();
        let mut p1 = self.get_point(segment_id + 1).unwrap();
        let v = p1.clone() - p0.clone();

        if proportion0 != 0.0 || proportion1 != 0.0 {
            p0 -= v.clone() * proportion0;
            p1 += v * proportion1;
        } else {
            let v_norm = v.normalized();
            p0 -= v_norm.clone() * dist0;
            p1 += v_norm * dist1;
        }

        self.set_point(segment_id, &p0);
        self.set_point(segment_id + 1, &p1);

        if self.is_closed() {
            let len = self.point_count();
            if segment_id == 0 {
                let first = self.get_point(0).unwrap();
                self.set_point(len - 1, &first);
            } else if segment_id + 1 == len - 1 {
                let last = self.get_point(len - 1).unwrap();
                self.set_point(0, &last);
            }
        }
    }

    /// Extend segment equally on both ends (static utility)
    pub fn extend_segment_equally_static(
        segment_start: &mut Point,
        segment_end: &mut Point,
        dist: f64,
        proportion: f64,
    ) {
        if dist == 0.0 && proportion == 0.0 {
            return;
        }

        let v = segment_end.clone() - segment_start.clone();

        if proportion != 0.0 {
            *segment_start = segment_start.clone() - (v.clone() * proportion);
            *segment_end = segment_end.clone() + (v * proportion);
        } else {
            let mut v_norm = v;
            v_norm.normalize();
            *segment_start = segment_start.clone() - (v_norm.clone() * dist);
            *segment_end = segment_end.clone() + (v_norm * dist);
        }
    }

    /// Extend polyline segment equally
    pub fn extend_segment_equally(&mut self, segment_id: usize, dist: f64, proportion: f64) {
        if segment_id >= self.segment_count() {
            return;
        }

        let mut start = self.get_point(segment_id).unwrap();
        let mut end = self.get_point(segment_id + 1).unwrap();
        Self::extend_segment_equally_static(&mut start, &mut end, dist, proportion);
        self.set_point(segment_id, &start);
        self.set_point(segment_id + 1, &end);

        if self.point_count() > 2 && self.is_closed() {
            let len = self.point_count();
            if segment_id == 0 {
                let first = self.get_point(0).unwrap();
                self.set_point(len - 1, &first);
            } else if segment_id + 1 == len - 1 {
                let last = self.get_point(len - 1).unwrap();
                self.set_point(0, &last);
            }
        }
    }

    /// Move polyline by direction vector
    pub fn move_by(&mut self, direction: &Vector) {
        for i in 0..self.point_count() {
            let idx = i * 3;
            self.coords[idx] += direction[0];
            self.coords[idx + 1] += direction[1];
            self.coords[idx + 2] += direction[2];
        }
    }

    /// Check if polyline is clockwise oriented
    pub fn is_clockwise(&self, _plane: &Plane) -> bool {
        let total = self.point_count();
        if total < 3 {
            return false;
        }

        let mut sum = 0.0;
        let n = if self.is_closed() { total - 1 } else { total };

        for i in 0..n {
            let idx_curr = i * 3;
            let idx_next = ((i + 1) % n) * 3;
            sum += (self.coords[idx_next] - self.coords[idx_curr])
                * (self.coords[idx_next + 1] + self.coords[idx_curr + 1]);
        }

        sum > 0.0
    }

    /// Flip polyline direction (reverse point order)
    pub fn flip(&mut self) {
        self.reverse();
    }

    /// Get convex/concave corners of polyline
    pub fn get_convex_corners(&self) -> Vec<bool> {
        let total = self.point_count();
        if total < 3 {
            return Vec::new();
        }

        let closed = self.is_closed();
        let normal = self.average_normal();
        let n = if closed { total - 1 } else { total };
        let mut convex_corners = Vec::with_capacity(n);
        let points = self.get_points();

        for current in 0..n {
            let prev = if current == 0 { n - 1 } else { current - 1 };
            let next = if current == n - 1 { 0 } else { current + 1 };

            let mut dir0 = points[current].clone() - points[prev].clone();
            dir0.normalize();

            let mut dir1 = points[next].clone() - points[current].clone();
            dir1.normalize();

            let mut cross = dir0.cross(&dir1);
            cross.normalize();

            let dot = cross.dot(&normal);
            let is_convex = dot >= 0.0;
            convex_corners.push(is_convex);
        }

        convex_corners
    }

    /// Interpolate between two polylines
    pub fn tween_two_polylines(
        polyline0: &Polyline,
        polyline1: &Polyline,
        weight: f64,
    ) -> Polyline {
        if polyline0.point_count() != polyline1.point_count() {
            return polyline0.clone();
        }

        let mut result = Polyline::default();
        result.coords.reserve(polyline0.coords.len());

        for i in 0..polyline0.point_count() {
            let idx = i * 3;
            let x = polyline0.coords[idx] + (polyline1.coords[idx] - polyline0.coords[idx]) * weight;
            let y = polyline0.coords[idx + 1] + (polyline1.coords[idx + 1] - polyline0.coords[idx + 1]) * weight;
            let z = polyline0.coords[idx + 2] + (polyline1.coords[idx + 2] - polyline0.coords[idx + 2]) * weight;
            result.coords.push(x);
            result.coords.push(y);
            result.coords.push(z);
        }

        result
    }

    /// Calculate average normal from polyline points
    fn average_normal(&self) -> Vector {
        let total = self.point_count();
        if total < 3 {
            return Vector::new(0.0, 0.0, 1.0);
        }

        let closed = self.is_closed();
        let n = if closed && total > 1 { total - 1 } else { total };
        let points = self.get_points();

        let mut average_normal = Vector::new(0.0, 0.0, 0.0);

        for i in 0..n {
            let prev = if i == 0 { n - 1 } else { i - 1 };
            let next = (i + 1) % n;

            let v1 = points[prev].clone() - points[i].clone();
            let v2 = points[i].clone() - points[next].clone();
            let cross = v1.cross(&v2);
            average_normal += &cross;
        }

        average_normal.normalize();
        average_normal
    }
}

impl AddAssign<&Vector> for Polyline {
    /// Translates all points in the polyline by a vector.
    fn add_assign(&mut self, other: &Vector) {
        for i in 0..self.point_count() {
            let idx = i * 3;
            self.coords[idx] += other[0];
            self.coords[idx + 1] += other[1];
            self.coords[idx + 2] += other[2];
        }
        // Update plane origin
        self.plane = Plane::new(
            self.plane.origin() + other.clone(),
            self.plane.x_axis(),
            self.plane.y_axis(),
        );
    }
}

impl Add<&Vector> for Polyline {
    type Output = Polyline;

    /// Translates the polyline by a vector and returns a new polyline.
    fn add(self, other: &Vector) -> Polyline {
        let mut result = self.clone();
        result += other;
        result
    }
}

impl SubAssign<&Vector> for Polyline {
    /// Translates all points in the polyline by the negative of a vector.
    fn sub_assign(&mut self, other: &Vector) {
        for i in 0..self.point_count() {
            let idx = i * 3;
            self.coords[idx] -= other[0];
            self.coords[idx + 1] -= other[1];
            self.coords[idx + 2] -= other[2];
        }
        // Update plane origin
        self.plane = Plane::new(
            self.plane.origin() - other.clone(),
            self.plane.x_axis(),
            self.plane.y_axis(),
        );
    }
}

impl Sub<&Vector> for Polyline {
    type Output = Polyline;

    /// Translates the polyline by the negative of a vector and returns a new polyline.
    fn sub(self, other: &Vector) -> Polyline {
        let mut result = self.clone();
        result -= other;
        result
    }
}

impl MulAssign<f64> for Polyline {
    /// Multiply all coordinates by scalar in place.
    fn mul_assign(&mut self, factor: f64) {
        for coord in self.coords.iter_mut() {
            *coord *= factor;
        }
    }
}

impl Mul<f64> for Polyline {
    type Output = Polyline;

    /// Multiply polyline by scalar and return new polyline.
    fn mul(self, factor: f64) -> Polyline {
        let mut result = self.clone();
        result *= factor;
        result
    }
}

impl DivAssign<f64> for Polyline {
    /// Divide all coordinates by scalar in place.
    fn div_assign(&mut self, factor: f64) {
        for coord in self.coords.iter_mut() {
            *coord /= factor;
        }
    }
}

impl Div<f64> for Polyline {
    type Output = Polyline;

    /// Divide polyline by scalar and return new polyline.
    fn div(self, factor: f64) -> Polyline {
        let mut result = self.clone();
        result /= factor;
        result
    }
}

impl Neg for Polyline {
    type Output = Polyline;

    /// Negate polyline (reverse point order).
    fn neg(self) -> Polyline {
        self.reversed()
    }
}

impl fmt::Display for Polyline {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Polyline(guid={}, name={}, points={})",
            self.guid,
            self.name,
            self.point_count()
        )
    }
}

#[cfg(test)]
#[path = "polyline_test.rs"]
mod polyline_test;
