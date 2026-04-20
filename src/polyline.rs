use crate::tolerance::PI;
use crate::{Color, Line, Plane, Point, Tolerance, Vector, Xform};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde_json::Value;
use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign};

/// A polyline defined by a collection of coordinates with an associated plane.
///
/// Internally stores coordinates as a flat array [x0, y0, z0, x1, y1, z1, ...] for
/// efficient serialization. Provides Point-based API for compatibility.
#[derive(Debug, Clone)]
pub struct Polyline {
    guid: std::sync::OnceLock<String>,
    pub name: String,
    /// Flat coordinate array [x0, y0, z0, x1, y1, z1, ...]
    pub coords: Vec<f64>,
    pub plane: Plane,
    plane_dirty: bool,
    pub width: f64,
    pub linecolor: Color,
    pub xform: Xform,
}

impl Default for Polyline {
    fn default() -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: "my_polyline".to_string(),
            coords: Vec::new(),
            plane: Plane::default(),
            plane_dirty: true,
            width: 1.0,
            linecolor: Color::black(),
            xform: Xform::identity(),
        }
    }
}

impl Polyline {
    pub fn new(points: Vec<Point>) -> Self {
        let mut coords = Vec::with_capacity(points.len() * 3);
        for p in &points {
            coords.push(p[0]);
            coords.push(p[1]);
            coords.push(p[2]);
        }
        Self {
            guid: std::sync::OnceLock::new(),
            name: "my_polyline".to_string(),
            coords,
            plane: Plane::default(),
            plane_dirty: true,
            width: 1.0,
            linecolor: Color::black(),
            xform: Xform::identity(),
        }
    }

    /// Get plane (lazy — computed on first access from first non-collinear triple).
    pub fn get_plane(&mut self) -> &Plane {
        if self.plane_dirty && self.point_count() >= 3 {
            let n = self.point_count();
            let p0 = Point::new(self.coords[0], self.coords[1], self.coords[2]);
            let mut found = false;
            for i in 1..n {
                let v1 = Vector::new(self.coords[i*3]-p0[0], self.coords[i*3+1]-p0[1], self.coords[i*3+2]-p0[2]);
                if v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2] < 1e-20 { continue; }
                for j in (i+1)..n {
                    let v2 = Vector::new(self.coords[j*3]-p0[0], self.coords[j*3+1]-p0[1], self.coords[j*3+2]-p0[2]);
                    let mut normal = v1.cross(&v2);
                    if normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2] < 1e-20 { continue; }
                    normal.normalize_self();
                    let mut v1n = v1.clone();
                    v1n.normalize_self();
                    let mut yax = normal.cross(&v1n);
                    yax.normalize_self();
                    self.plane = Plane::new(p0.clone(), v1n, yax);
                    found = true;
                    break;
                }
                if found { break; }
            }
            self.plane_dirty = false;
        }
        &self.plane
    }

    /// Create a regular polygon with given number of sides and radius.
    pub fn from_sides(sides: usize, radius: f64, close: bool) -> Self {
        let cap = if close { sides + 1 } else { sides };
        let mut coords: Vec<f64> = Vec::with_capacity(cap * 3);
        for i in 0..sides {
            let angle = 2.0 * PI * i as f64 / sides as f64;
            coords.push(radius * angle.cos());
            coords.push(radius * angle.sin());
            coords.push(0.0);
        }
        if close {
            coords.push(coords[0]);
            coords.push(coords[1]);
            coords.push(coords[2]);
        }
        Self::from_coords(coords)
    }

    /// Creates a Polyline from a flat coordinate array.
    pub fn from_coords(coords: Vec<f64>) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: "my_polyline".to_string(),
            coords,
            plane: Plane::default(),
            plane_dirty: true,
            width: 1.0,
            linecolor: Color::black(),
            xform: Xform::identity(),
        }
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Returns detailed string representation (like Python __repr__).
    pub fn repr(&self) -> String {
        format!("Polyline({}, {} points)", self.name, self.point_count())
    }

    /// Creates a deep copy with a new GUID.
    pub fn duplicate(&self) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: self.name.clone(),
            coords: self.coords.clone(),
            plane: self.plane.clone(),
            plane_dirty: self.plane_dirty,
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

    /// Returns all segments as Line objects.
    pub fn get_lines(&self) -> Vec<Line> {
        let mut result = Vec::with_capacity(self.segment_count());
        for i in 0..self.segment_count() {
            let idx0 = i * 3;
            let idx1 = (i + 1) * 3;
            result.push(Line::new(
                self.coords[idx0], self.coords[idx0 + 1], self.coords[idx0 + 2],
                self.coords[idx1], self.coords[idx1 + 1], self.coords[idx1 + 2],
            ));
        }
        result
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
        // Transform coordinates in-place without creating Point objects
        for i in 0..self.point_count() {
            let idx = i * 3;
            let mut pt = Point::new(self.coords[idx], self.coords[idx + 1], self.coords[idx + 2]);
            pt.xform = self.xform.clone();
            pt.transform();
            self.coords[idx] = pt[0];
            self.coords[idx + 1] = pt[1];
            self.coords[idx + 2] = pt[2];
        }
        self.xform = Xform::identity();
    }

    pub fn transformed(&self) -> Self {
        let mut result = self.clone();
        result.transform();
        result
    }

    /// Return a copy of this polyline with `xf` applied to every point.
    /// Mirrors C++ `Polyline::transformed_xform`.
    pub fn transformed_xform(&self, xf: &Xform) -> Polyline {
        let m = &xf.m;
        let mut new_pts = Vec::with_capacity(self.point_count());
        for i in 0..self.point_count() {
            let p = self.get_point(i).unwrap();
            let x = m[0]*p[0] + m[4]*p[1] + m[8]*p[2]  + m[12];
            let y = m[1]*p[0] + m[5]*p[1] + m[9]*p[2]  + m[13];
            let z = m[2]*p[0] + m[6]*p[1] + m[10]*p[2] + m[14];
            new_pts.push(Point::new(x, y, z));
        }
        Polyline::new(new_pts)
    }

    /// Translate every point of this polyline by `v` (in place).
    /// Mirrors C++ `Polyline::translate`.
    pub fn translate(&mut self, v: &Vector) {
        for i in 0..self.point_count() {
            let idx = i * 3;
            self.coords[idx]     += v[0];
            self.coords[idx + 1] += v[1];
            self.coords[idx + 2] += v[2];
        }
    }

    /// Slide both endpoints of edge `edge_idx` outward by `distance`.
    /// Negative `distance` slides them inward. For closed polylines the
    /// closing-duplicate vertex is kept in sync.
    pub fn extend_edge_equally(&mut self, edge_idx: usize, distance: f64) {
        let n = self.point_count();
        if n < 2 || edge_idx + 1 >= n {
            return;
        }
        let i = edge_idx;
        let j = edge_idx + 1;
        let pi = self.get_point(i).unwrap();
        let pj = self.get_point(j).unwrap();
        let dx = pj[0] - pi[0];
        let dy = pj[1] - pi[1];
        let dz = pj[2] - pi[2];
        let len = (dx*dx + dy*dy + dz*dz).sqrt();
        if len < 1e-12 {
            return;
        }
        let inv = 1.0 / len;
        let ux = dx * inv * distance;
        let uy = dy * inv * distance;
        let uz = dz * inv * distance;
        let new_pi = Point::new(pi[0]-ux, pi[1]-uy, pi[2]-uz);
        let new_pj = Point::new(pj[0]+ux, pj[1]+uy, pj[2]+uz);
        self.set_point(i, &new_pi);
        self.set_point(j, &new_pj);
        if i == 0 {
            self.set_point(n - 1, &new_pi);
        }
        if j == n - 1 {
            self.set_point(0, &new_pj);
        }
    }

     /// Recompute plane if we have at least 3 points
     fn recompute_plane_if_needed(&mut self) {
         self.plane_dirty = true;
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
         map.serialize_entry("guid", self.guid())?;
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

         let guid_str = value
             .get("guid")
             .and_then(|v| v.as_str())
             .map(|s| s.to_string())
             .unwrap_or_else(|| uuid::Uuid::new_v4().to_string());

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

         let polyline = Polyline {
             guid: { let c = std::sync::OnceLock::new(); let _ = c.set(guid_str); c },
             name,
             coords,
             plane: Plane::default(),
             plane_dirty: true,
             width,
             linecolor,
             xform,
         };
         Ok(polyline)
     }
 }

 impl Polyline {

     /// Serializes the Polyline to a JSON string.
     pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
         crate::encoders::sorted_json_string(self)
     }

    /// Deserializes a Polyline from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    pub fn json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_else(|_| Self::default())
    }

    /// Serializes the Polyline to a JSON file.
    pub fn json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    /// Deserializes a Polyline from a JSON file.
    pub fn json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
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

        let proto = crate::proto::Polyline {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            coords: self.coords.clone(),
            width: self.width,
            linecolor: Some(crate::proto::Color {
                guid: self.linecolor.guid().to_string(),
                name: self.linecolor.name.clone(),
                r: self.linecolor.r as i32,
                g: self.linecolor.g as i32,
                b: self.linecolor.b as i32,
                a: self.linecolor.a as i32,
            }),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid().to_string(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
        };
        proto.encode_to_vec()
    }

    /// Create Polyline from protobuf binary data.
    ///
    /// # Arguments
    ///
    /// * `data` - Byte slice containing protobuf-encoded polyline data.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized Polyline or an error.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        let proto = crate::proto::Polyline::decode(data)?;

        let mut pl = Self::from_coords(proto.coords);
        pl.set_guid(proto.guid);
        pl.name = proto.name;
        pl.width = proto.width;

        if let Some(color) = proto.linecolor {
            pl.linecolor.set_guid(color.guid);
            pl.linecolor.name = color.name;
            pl.linecolor.r = color.r as u8;
            pl.linecolor.g = color.g as u8;
            pl.linecolor.b = color.b as u8;
            pl.linecolor.a = color.a as u8;
        }

        if let Some(xform) = proto.xform {
            pl.xform.set_guid(xform.guid);
            pl.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 {
                    pl.xform.m[i] = *val;
                }
            }
        }

        Ok(pl)
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
    /// The deserialized Polyline.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
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
        // Direction vector (no clone needed - use direct coordinate access)
        let dx = line_end[0] - line_start[0];
        let dy = line_end[1] - line_start[1];
        let dz = line_end[2] - line_start[2];
        let dod = dx * dx + dy * dy + dz * dz;

        if dod > 0.0 {
            // Vector from line_start to point
            let px = point[0] - line_start[0];
            let py = point[1] - line_start[1];
            let pz = point[2] - line_start[2];
            
            // Vector from line_end to point
            let qx = point[0] - line_end[0];
            let qy = point[1] - line_end[1];
            let qz = point[2] - line_end[2];
            
            let dist_start_sq = px * px + py * py + pz * pz;
            let dist_end_sq = qx * qx + qy * qy + qz * qz;
            
            if dist_start_sq <= dist_end_sq {
                (px * dx + py * dy + pz * dz) / dod
            } else {
                1.0 + (qx * dx + qy * dy + qz * dz) / dod
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

            // Compute magnitude_squared directly without cloning
            let mid0_dx = mid_line0_end[0] - mid_line0_start[0];
            let mid0_dy = mid_line0_end[1] - mid_line0_start[1];
            let mid0_dz = mid_line0_end[2] - mid_line0_start[2];
            let mid0_mag_sq = mid0_dx * mid0_dx + mid0_dy * mid0_dy + mid0_dz * mid0_dz;
            
            let mid1_dx = mid_line1_end[0] - mid_line1_start[0];
            let mid1_dy = mid_line1_end[1] - mid_line1_start[1];
            let mid1_dz = mid_line1_end[2] - mid_line1_start[2];
            let mid1_mag_sq = mid1_dx * mid1_dx + mid1_dy * mid1_dy + mid1_dz * mid1_dz;

            if mid0_mag_sq > mid1_mag_sq {
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

    pub fn closed(&self) -> Self {
        if self.is_closed() {
            return Self::from_coords(self.coords.clone());
        }
        let mut new_coords = self.coords.clone();
        new_coords.push(self.coords[0]);
        new_coords.push(self.coords[1]);
        new_coords.push(self.coords[2]);
        Self::from_coords(new_coords)
    }

    /// Merge consecutive collinear segments in-place; closed polyline wraps around
    pub fn merge_collinear(&mut self, tol: f64) {
        let closed = self.is_closed();
        let mut pts = self.get_points();
        if closed && pts.len() > 1 {
            pts.pop();
        }
        let zt2 = Tolerance::ZERO_TOLERANCE * Tolerance::ZERO_TOLERANCE;
        let mut changed = true;
        while changed {
            changed = false;
            let m = pts.len();
            if m < 3 { break; }
            let mut out = Vec::new();
            for i in 0..m {
                let p = (i + m - 1) % m;
                let nx = (i + 1) % m;
                if !closed && (i == 0 || i == m - 1) { out.push(pts[i].clone()); continue; }
                let (ax, ay, az) = (pts[i][0]-pts[p][0], pts[i][1]-pts[p][1], pts[i][2]-pts[p][2]);
                let (bx, by, bz) = (pts[nx][0]-pts[i][0], pts[nx][1]-pts[i][1], pts[nx][2]-pts[i][2]);
                let (cx, cy, cz) = (ay*bz-az*by, az*bx-ax*bz, ax*by-ay*bx);
                let (a2, b2) = (ax*ax+ay*ay+az*az, bx*bx+by*by+bz*bz);
                if a2 < zt2 || b2 < zt2 || cx*cx+cy*cy+cz*cz < tol*tol*a2*b2 {
                    changed = true;
                } else {
                    out.push(pts[i].clone());
                }
            }
            pts = out;
        }
        self.coords.clear();
        for p in &pts {
            self.coords.push(p[0]);
            self.coords.push(p[1]);
            self.coords.push(p[2]);
        }
        if closed && !pts.is_empty() {
            self.coords.push(pts[0][0]);
            self.coords.push(pts[0][1]);
            self.coords.push(pts[0][2]);
        }
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

    /// Winding-number point-in-polygon test. p.x/y tested; polygon vertex z ignored.
    pub fn point_in_polygon_2d(&self, p: &crate::point::Point) -> bool {
        let (px, py) = (p[0], p[1]);
        let coords = self.get_points();
        let mut winding: i32 = 0;
        let n = coords.len();
        for i in 0..n {
            let j = (i + 1) % n;
            let y0 = coords[i][1];
            let y1 = coords[j][1];
            if y0 <= py {
                if y1 > py {
                    let x0 = coords[i][0]; let x1 = coords[j][0];
                    if (x1 - x0) * (py - y0) - (px - x0) * (y1 - y0) > 0.0 { winding += 1; }
                }
            } else if y1 <= py {
                let x0 = coords[i][0]; let x1 = coords[j][0];
                if (x1 - x0) * (py - y0) - (px - x0) * (y1 - y0) < 0.0 { winding -= 1; }
            }
        }
        winding != 0
    }

    /// Get average plane from polyline points
    pub fn get_average_plane(&self) -> (Point, Vector, Vector, Vector) {
        let origin = self.center();
        let points = self.get_points();

        let x_axis = if points.len() >= 2 {
            let mut x = points[1].clone() - points[0].clone();
            x.normalize_self();
            x
        } else {
            Vector::new(1.0, 0.0, 0.0)
        };

        let z_axis = self.average_normal();
        let mut y_axis = z_axis.cross(&x_axis);
        y_axis.normalize_self();

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
    pub fn extend_line_segment(
        line_start: &mut Point,
        line_end: &mut Point,
        distance0: f64,
        distance1: f64,
    ) {
        let mut v = line_end.clone() - line_start.clone();
        v.normalize_self();

        *line_start = line_start.clone() - (v.clone() * distance0);
        *line_end = line_end.clone() + (v * distance1);
    }

    /// Shrink line segment inward by specified distance
    pub fn shrink_line_segment(line_start: &mut Point, line_end: &mut Point, distance: f64) {
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
            v_norm.normalize_self();
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
            dir0.normalize_self();

            let mut dir1 = points[next].clone() - points[current].clone();
            dir1.normalize_self();

            let mut cross = dir0.cross(&dir1);
            cross.normalize_self();

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

    // -----------------------------------------------------------------------
    // Group B: Geometry utilities
    // -----------------------------------------------------------------------

    /// Linear interpolation: type 0=no endpoints, 1=both, 2=start only.
    pub fn interpolate_points(from: &Point, to: &Point, steps: usize, kind: u8) -> Vec<Point> {
        let lerp = |t: f64| {
            Point::new(
                from[0] + t * (to[0] - from[0]),
                from[1] + t * (to[1] - from[1]),
                from[2] + t * (to[2] - from[2]),
            )
        };
        let mut pts = Vec::new();
        match kind {
            1 => {
                pts.push(from.clone());
                for i in 1..=steps {
                    pts.push(lerp(i as f64 / (steps + 1) as f64));
                }
                pts.push(to.clone());
            }
            2 => {
                pts.push(from.clone());
                for i in 1..=steps {
                    pts.push(lerp(i as f64 / (steps + 1) as f64));
                }
            }
            _ => {
                for i in 1..=steps {
                    pts.push(lerp(i as f64 / (steps + 1) as f64));
                }
            }
        }
        pts
    }

    /// 2D convex hull (quickhull) in the polygon's local plane.
    pub fn quick_hull(polygon: &Polyline) -> Polyline {
        let (orig, xa, ya, _za) = polygon.get_average_plane();
        let pts = polygon.get_points();

        // Project to 2D
        let pts2d: Vec<[f64; 2]> = pts.iter().map(|p| {
            let dx = p[0] - orig[0];
            let dy = p[1] - orig[1];
            let dz = p[2] - orig[2];
            [dx * xa[0] + dy * xa[1] + dz * xa[2],
             dx * ya[0] + dy * ya[1] + dz * ya[2]]
        }).collect();

        fn ccw_2d(ax: f64, ay: f64, bx: f64, by: f64, px: f64, py: f64) -> f64 {
            (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        }
        fn qh_recurse(v: &[[f64; 2]], ax: f64, ay: f64, bx: f64, by: f64, hull: &mut Vec<[f64; 2]>) {
            if v.is_empty() { return; }
            let (fi, _) = v.iter().enumerate()
                .max_by(|(_, a), (_, b)| {
                    ccw_2d(ax, ay, bx, by, a[0], a[1])
                        .partial_cmp(&ccw_2d(ax, ay, bx, by, b[0], b[1]))
                        .unwrap()
                }).unwrap();
            let fx = v[fi][0]; let fy = v[fi][1];
            let left: Vec<_> = v.iter().filter(|p| ccw_2d(ax, ay, fx, fy, p[0], p[1]) > 0.0).cloned().collect();
            qh_recurse(&left, ax, ay, fx, fy, hull);
            hull.push([fx, fy]);
            let right: Vec<_> = v.iter().filter(|p| ccw_2d(fx, fy, bx, by, p[0], p[1]) > 0.0).cloned().collect();
            qh_recurse(&right, fx, fy, bx, by, hull);
        }

        let ai = pts2d.iter().enumerate().min_by(|(_, a), (_, b)| a[0].partial_cmp(&b[0]).unwrap()).map(|(i, _)| i).unwrap_or(0);
        let bi = pts2d.iter().enumerate().max_by(|(_, a), (_, b)| a[0].partial_cmp(&b[0]).unwrap()).map(|(i, _)| i).unwrap_or(0);
        let (ax, ay) = (pts2d[ai][0], pts2d[ai][1]);
        let (bx, by) = (pts2d[bi][0], pts2d[bi][1]);

        let left: Vec<_>  = pts2d.iter().filter(|p| ccw_2d(ax, ay, bx, by, p[0], p[1]) > 0.0).cloned().collect();
        let right: Vec<_> = pts2d.iter().filter(|p| ccw_2d(ax, ay, bx, by, p[0], p[1]) <= 0.0).cloned().collect();
        let mut hull = vec![[ax, ay]];
        qh_recurse(&left, ax, ay, bx, by, &mut hull);
        hull.push([bx, by]);
        qh_recurse(&right, bx, by, ax, ay, &mut hull);

        let pts3d: Vec<Point> = hull.iter().map(|h| {
            Point::new(orig[0] + h[0] * xa[0] + h[1] * ya[0],
                       orig[1] + h[0] * xa[1] + h[1] * ya[1],
                       orig[2] + h[0] * xa[2] + h[1] * ya[2])
        }).collect();
        Polyline::new(pts3d)
    }

    /// Minimum-area bounding rectangle via rotating calipers; returns closed 5-pt Polyline.
    pub fn bounding_rectangle(polygon: &Polyline) -> Option<Polyline> {
        let hull = Self::quick_hull(polygon);
        if hull.point_count() <= 2 { return None; }
        let (orig, xa, ya, _za) = polygon.get_average_plane();

        // Project hull to 2D
        let hull_pts = hull.get_points();
        let hull2d: Vec<[f64; 2]> = hull_pts.iter().map(|p| {
            let dx = p[0] - orig[0]; let dy = p[1] - orig[1]; let dz = p[2] - orig[2];
            [dx * xa[0] + dy * xa[1] + dz * xa[2],
             dx * ya[0] + dy * ya[1] + dz * ya[2]]
        }).collect();

        let mut best_area = f64::MAX;
        let mut best = (0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64);
        let hn = hull2d.len();
        for i in 0..hn {
            let j = (i + 1) % hn;
            let ex = hull2d[j][0] - hull2d[i][0];
            let ey = hull2d[j][1] - hull2d[i][1];
            let len = (ex * ex + ey * ey).sqrt();
            if len < 1e-12 { continue; }
            let (ca, sa) = (ex / len, ey / len);
            let (mut min_u, mut max_u) = (f64::MAX, f64::MIN);
            let (mut min_v, mut max_v) = (f64::MAX, f64::MIN);
            for h in &hull2d {
                let u =  h[0] * ca + h[1] * sa;
                let v = -h[0] * sa + h[1] * ca;
                min_u = min_u.min(u); max_u = max_u.max(u);
                min_v = min_v.min(v); max_v = max_v.max(v);
            }
            let area = (max_u - min_u) * (max_v - min_v);
            if area < best_area {
                best_area = area;
                best = (min_u, max_u, min_v, max_v, ey.atan2(ex));
            }
        }
        let (min_u, max_u, min_v, max_v, angle) = best;
        let (ca, sa) = (angle.cos(), angle.sin());
        let rot_back = |u: f64, v: f64| -> [f64; 2] { [u * ca - v * sa, u * sa + v * ca] };
        let to3d = |u2: f64, v2: f64| -> Point {
            Point::new(orig[0] + u2 * xa[0] + v2 * ya[0],
                       orig[1] + u2 * xa[1] + v2 * ya[1],
                       orig[2] + u2 * xa[2] + v2 * ya[2])
        };
        let c = [rot_back(min_u, min_v), rot_back(min_u, max_v),
                 rot_back(max_u, max_v), rot_back(max_u, min_v)];
        let mut pts3d: Vec<Point> = c.iter().map(|h| to3d(h[0], h[1])).collect();
        pts3d.push(pts3d[0].clone());
        Some(Polyline::new(pts3d))
    }

    /// Grid of interior points; offset_dist ignored (no clipper); div_dist = grid spacing.
    pub fn grid_of_points_in_polygon(polygon: &Polyline, offset_dist: f64, div_dist: f64, max_pts: usize) -> Vec<Point> {
        if div_dist < 1e-12 { return Vec::new(); }
        let (orig, xa, ya, _za) = polygon.get_average_plane();
        let pts = polygon.get_points();

        // Build 2D polygon, skip duplicate last point if closed
        let last = if pts.len() > 1 {
            let a = &pts[0]; let b = &pts[pts.len()-1];
            if (a[0]-b[0]).abs()<1e-10 && (a[1]-b[1]).abs()<1e-10 && (a[2]-b[2]).abs()<1e-10
                { pts.len() - 1 } else { pts.len() }
        } else { pts.len() };

        let mut poly2d: Vec<[f64; 2]> = pts[..last].iter().map(|p| {
            let dx = p[0]-orig[0]; let dy = p[1]-orig[1]; let dz = p[2]-orig[2];
            [dx*xa[0]+dy*xa[1]+dz*xa[2], dx*ya[0]+dy*ya[1]+dz*ya[2]]
        }).collect();

        if poly2d.is_empty() { return Vec::new(); }

        // Miter offset in 2D (negative = inward, positive = outward). Reuses
        // the same algorithm as Intersection::offset_in_3d. Falls back to the
        // un-offset polygon if the result degenerates.
        if offset_dist != 0.0 && poly2d.len() >= 3 {
            let n = poly2d.len();
            let mut signed_area = 0.0;
            for i in 0..n {
                let a = poly2d[i];
                let b = poly2d[(i+1) % n];
                signed_area += a[0]*b[1] - b[0]*a[1];
            }
            let delta = if signed_area < 0.0 { -offset_dist } else { offset_dist };

            let mut normals: Vec<[f64; 2]> = Vec::with_capacity(n);
            for i in 0..n {
                let a = poly2d[i];
                let b = poly2d[(i+1) % n];
                let ex = b[0]-a[0];
                let ey = b[1]-a[1];
                let len = (ex*ex + ey*ey).sqrt();
                if len < 1e-12 { normals.push([0.0, 0.0]); }
                else { normals.push([ey/len, -ex/len]); }
            }

            let mut out: Vec<[f64; 2]> = Vec::with_capacity(n * 3);
            for i in 0..n {
                let np = normals[(i + n - 1) % n];
                let nn = normals[i];
                let cos_a = np[0]*nn[0] + np[1]*nn[1];
                let sin_a = np[0]*nn[1] - np[1]*nn[0];
                let denom = 1.0 + cos_a;
                let concave = (cos_a > -0.999) && (sin_a * delta < 0.0) && (offset_dist > 0.0);
                if concave {
                    out.push([poly2d[i][0] + np[0]*delta, poly2d[i][1] + np[1]*delta]);
                    out.push([poly2d[i][0], poly2d[i][1]]);
                    out.push([poly2d[i][0] + nn[0]*delta, poly2d[i][1] + nn[1]*delta]);
                } else if denom.abs() < 1e-9 {
                    let mx = (np[0]+nn[0])*0.5;
                    let my = (np[1]+nn[1])*0.5;
                    out.push([poly2d[i][0] + mx*delta, poly2d[i][1] + my*delta]);
                } else {
                    let bx = (np[0]+nn[0])/denom;
                    let by = (np[1]+nn[1])/denom;
                    out.push([poly2d[i][0] + bx*delta, poly2d[i][1] + by*delta]);
                }
            }

            let mut out_area = 0.0;
            for i in 0..out.len() {
                let a = out[i];
                let b = out[(i+1) % out.len()];
                out_area += a[0]*b[1] - b[0]*a[1];
            }
            if out.len() >= 3 && out_area.abs() > 1e-4 { poly2d = out; }
        }

        let (mut x_min, mut x_max) = (f64::MAX, f64::MIN);
        let (mut y_min, mut y_max) = (f64::MAX, f64::MIN);
        for p in &poly2d {
            x_min = x_min.min(p[0]); x_max = x_max.max(p[0]);
            y_min = y_min.min(p[1]); y_max = y_max.max(p[1]);
        }

        fn pt_in_poly(px: f64, py: f64, poly: &[[f64; 2]]) -> bool {
            let n = poly.len();
            let mut inside = false;
            let mut j = n - 1;
            for i in 0..n {
                let xi = poly[i][0]; let yi = poly[i][1];
                let xj = poly[j][0]; let yj = poly[j][1];
                if ((yi > py) != (yj > py)) && (px < (xj-xi)*(py-yi)/(yj-yi)+xi) {
                    inside = !inside;
                }
                j = i;
            }
            inside
        }

        let mut result = Vec::new();
        let mut u = x_min;
        while u <= x_max + 1e-10 && result.len() < max_pts {
            let mut v = y_min;
            while v <= y_max + 1e-10 && result.len() < max_pts {
                if pt_in_poly(u, v, &poly2d) {
                    result.push(Point::new(
                        orig[0] + u*xa[0] + v*ya[0],
                        orig[1] + u*xa[1] + v*ya[1],
                        orig[2] + u*xa[2] + v*ya[2],
                    ));
                }
                v += div_dist;
            }
            u += div_dist;
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

        average_normal.normalize_self();
        average_normal
    }

    fn simplify_perp_dist(pt: &Point, line_start: &Point, line_end: &Point) -> f64 {
        let dx = line_end[0] - line_start[0];
        let dy = line_end[1] - line_start[1];
        let dz = line_end[2] - line_start[2];
        let len_sq = dx * dx + dy * dy + dz * dz;
        if len_sq == 0.0 {
            let ex = pt[0] - line_start[0];
            let ey = pt[1] - line_start[1];
            let ez = pt[2] - line_start[2];
            return (ex * ex + ey * ey + ez * ez).sqrt();
        }
        let t = ((pt[0] - line_start[0]) * dx + (pt[1] - line_start[1]) * dy + (pt[2] - line_start[2]) * dz) / len_sq;
        let t = t.max(0.0).min(1.0);
        let cx = line_start[0] + t * dx;
        let cy = line_start[1] + t * dy;
        let cz = line_start[2] + t * dz;
        let ex = pt[0] - cx;
        let ey = pt[1] - cy;
        let ez = pt[2] - cz;
        (ex * ex + ey * ey + ez * ez).sqrt()
    }

    fn simplify_rdp(points: &[Point], start: usize, end: usize, tolerance: f64, keep: &mut Vec<bool>) {
        if end <= start + 1 { return; }
        let mut max_dist = 0.0_f64;
        let mut max_idx = start;
        for i in (start + 1)..end {
            let d = Self::simplify_perp_dist(&points[i], &points[start], &points[end]);
            if d > max_dist {
                max_dist = d;
                max_idx = i;
            }
        }
        if max_dist > tolerance {
            keep[max_idx] = true;
            Self::simplify_rdp(points, start, max_idx, tolerance, keep);
            Self::simplify_rdp(points, max_idx, end, tolerance, keep);
        }
    }

    pub fn simplify_points(points: &[Point], tolerance: f64) -> Vec<Point> {
        let n = points.len();
        if n < 3 { return points.to_vec(); }
        let mut keep = vec![false; n];
        keep[0] = true;
        keep[n - 1] = true;
        Self::simplify_rdp(points, 0, n - 1, tolerance, &mut keep);
        points.iter().enumerate().filter(|(i, _)| keep[*i]).map(|(_, p)| p.clone()).collect()
    }

    pub fn simplify(&self, tolerance: f64) -> Polyline {
        let pts = self.get_points();
        let simplified = Self::simplify_points(&pts, tolerance);
        Polyline::new(simplified)
    }

    /// Largest inscribed circle via mapbox polylabel.
    /// polylines[0] is the outer boundary; polylines[1..] are holes.
    pub fn polylabel(polylines: &[Polyline], precision: f64) -> (Point, crate::plane::Plane, f64) {
        if polylines.is_empty() {
            return (Point::new(0.0, 0.0, 0.0), crate::plane::Plane::default(), 0.0);
        }
        let (orig, xa, ya, za) = polylines[0].get_average_plane();
        let to2d = |p: &Point| -> [f64; 2] {
            let dx = p[0]-orig[0]; let dy = p[1]-orig[1]; let dz = p[2]-orig[2];
            [dx*xa[0]+dy*xa[1]+dz*xa[2], dx*ya[0]+dy*ya[1]+dz*ya[2]]
        };

        let mut rings2d: Vec<Vec<[f64; 2]>> = Vec::with_capacity(polylines.len());
        let mut sizes: Vec<f64> = Vec::with_capacity(polylines.len());
        for pl in polylines {
            let pts = pl.get_points();
            let last = if pts.len() > 1 {
                let a = &pts[0]; let b = &pts[pts.len()-1];
                if (a[0]-b[0]).abs()<1e-10 && (a[1]-b[1]).abs()<1e-10 && (a[2]-b[2]).abs()<1e-10
                    { pts.len() - 1 } else { pts.len() }
            } else { pts.len() };
            let mut ring: Vec<[f64; 2]> = Vec::with_capacity(last);
            let (mut mnx, mut mxx) = (f64::INFINITY, f64::NEG_INFINITY);
            let (mut mny, mut mxy) = (f64::INFINITY, f64::NEG_INFINITY);
            for p in &pts[..last] {
                let uv = to2d(p);
                mnx = mnx.min(uv[0]); mxx = mxx.max(uv[0]);
                mny = mny.min(uv[1]); mxy = mxy.max(uv[1]);
                ring.push(uv);
            }
            let dx = mxx - mnx; let dy = mxy - mny;
            sizes.push(dx*dx + dy*dy);
            rings2d.push(ring);
        }
        let mut ids: Vec<usize> = (0..rings2d.len()).collect();
        ids.sort_by(|&a, &b| sizes[b].partial_cmp(&sizes[a]).unwrap_or(std::cmp::Ordering::Equal));
        let polygon: Vec<Vec<[f64; 2]>> = ids.iter().map(|&i| rings2d[i].clone()).collect();

        let cr = mapbox_polylabel(&polygon, precision);
        let center = Point::new(orig[0] + cr[0]*xa[0] + cr[1]*ya[0],
                                orig[1] + cr[0]*xa[1] + cr[1]*ya[1],
                                orig[2] + cr[0]*xa[2] + cr[1]*ya[2]);
        let plane = crate::plane::Plane::new(orig.clone(), xa.clone(), ya.clone());
        let _ = za;
        (center, plane, cr[2])
    }

    /// Points on the polylabel inscribed circle, scaled by `scale`.
    pub fn polylabel_circle_division_points(
        division_direction_in_3d: &Vector,
        polylines: &[Polyline],
        division: usize,
        scale: f64,
        precision: f64,
        orient_to_closest_edge: bool,
    ) -> Vec<Point> {
        let (center, plane, r) = Self::polylabel(polylines, precision);
        let radius = r * scale;

        let is_direction_valid =
            division_direction_in_3d[0] != 0.0 ||
            division_direction_in_3d[1] != 0.0 ||
            division_direction_in_3d[2] != 0.0;

        let mut edge_i: usize = 0;
        let mut edge_j: usize = 0;
        let mut best_sq = f64::INFINITY;
        if orient_to_closest_edge {
            for (i, pl) in polylines.iter().enumerate() {
                let pts = pl.get_points();
                if pts.len() < 2 { continue; }
                for j in 0..pts.len()-1 {
                    let a = &pts[j]; let b = &pts[j+1];
                    let ex = b[0]-a[0]; let ey = b[1]-a[1]; let ez = b[2]-a[2];
                    let len2 = ex*ex + ey*ey + ez*ez;
                    if len2 <= 0.0 { continue; }
                    let px = center[0]-a[0]; let py = center[1]-a[1]; let pz = center[2]-a[2];
                    let t = (px*ex + py*ey + pz*ez) / len2;
                    if t < 0.0 || t > 1.0 { continue; }
                    let cx = a[0]+t*ex; let cy = a[1]+t*ey; let cz = a[2]+t*ez;
                    let dx = center[0]-cx; let dy = center[1]-cy; let dz = center[2]-cz;
                    let d2 = dx*dx + dy*dy + dz*dz;
                    if d2 < best_sq { best_sq = d2; edge_i = i; edge_j = j; }
                }
            }
        }

        let z_axis_ref = plane.z_axis().clone();
        let (mut x_axis, mut y_axis);
        if is_direction_valid || orient_to_closest_edge {
            let dir = if orient_to_closest_edge && best_sq.is_finite() {
                let pts = polylines[edge_i].get_points();
                Vector::new(pts[edge_j+1][0]-pts[edge_j][0],
                            pts[edge_j+1][1]-pts[edge_j][1],
                            pts[edge_j+1][2]-pts[edge_j][2])
            } else {
                division_direction_in_3d.clone()
            };
            x_axis = dir.clone();
            y_axis = Vector::new(dir[1]*z_axis_ref[2] - dir[2]*z_axis_ref[1],
                                 dir[2]*z_axis_ref[0] - dir[0]*z_axis_ref[2],
                                 dir[0]*z_axis_ref[1] - dir[1]*z_axis_ref[0]);
        } else {
            x_axis = plane.x_axis().clone();
            y_axis = plane.y_axis().clone();
        }
        let mut z_axis = z_axis_ref;
        let unit = |v: &mut Vector| {
            let l = (v[0]*v[0]+v[1]*v[1]+v[2]*v[2]).sqrt();
            if l > 0.0 { *v = Vector::new(v[0]/l, v[1]/l, v[2]/l); }
        };
        unit(&mut x_axis); unit(&mut y_axis); unit(&mut z_axis);

        let mut points: Vec<Point> = Vec::with_capacity(division);
        let pi_rad = std::f64::consts::PI / 180.0;
        let chunk = 360.0 / (division as f64);
        for i in 0..division {
            let deg = (i as f64) * chunk;
            let r = (45.0 + deg) * pi_rad;
            let u = radius * r.cos();
            let v = radius * r.sin();
            points.push(Point::new(center[0] + u*x_axis[0] + v*y_axis[0],
                                   center[1] + u*x_axis[1] + v*y_axis[1],
                                   center[2] + u*x_axis[2] + v*y_axis[2]));
        }
        points
    }
}

// ========== mapbox polylabel helpers (native) ==========
fn pl_seg_dist_sq(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let mut x = ax; let mut y = ay;
    let mut dx = bx - x; let mut dy = by - y;
    if dx != 0.0 || dy != 0.0 {
        let t = ((px - x) * dx + (py - y) * dy) / (dx*dx + dy*dy);
        if t > 1.0 { x = bx; y = by; }
        else if t > 0.0 { x += dx * t; y += dy * t; }
    }
    dx = px - x; dy = py - y;
    dx*dx + dy*dy
}

fn pl_point_to_poly_dist(px: f64, py: f64, polygon: &[Vec<[f64; 2]>]) -> f64 {
    let mut inside = false;
    let mut min_sq = f64::INFINITY;
    for ring in polygon {
        let len = ring.len();
        if len == 0 { continue; }
        let mut j = len - 1;
        for i in 0..len {
            let ax = ring[i][0]; let ay = ring[i][1];
            let bx = ring[j][0]; let by = ring[j][1];
            if (ay > py) != (by > py) && (px < (bx-ax)*(py-ay)/(by-ay) + ax) {
                inside = !inside;
            }
            min_sq = min_sq.min(pl_seg_dist_sq(px, py, ax, ay, bx, by));
            j = i;
        }
    }
    (if inside { 1.0 } else { -1.0 }) * min_sq.sqrt()
}

#[derive(Clone, Copy)]
struct PCell { cx: f64, cy: f64, h: f64, d: f64, mx: f64 }
impl PCell {
    fn new(cx: f64, cy: f64, h: f64, polygon: &[Vec<[f64; 2]>]) -> Self {
        let d = pl_point_to_poly_dist(cx, cy, polygon);
        PCell { cx, cy, h, d, mx: d + h * std::f64::consts::SQRT_2 }
    }
}
impl PartialEq for PCell { fn eq(&self, o: &Self) -> bool { self.mx == o.mx } }
impl Eq for PCell {}
impl PartialOrd for PCell { fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> { self.mx.partial_cmp(&o.mx) } }
impl Ord for PCell { fn cmp(&self, o: &Self) -> std::cmp::Ordering { self.partial_cmp(o).unwrap_or(std::cmp::Ordering::Equal) } }

fn pl_centroid_cell(polygon: &[Vec<[f64; 2]>]) -> PCell {
    let mut area = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    let ring = &polygon[0];
    let len = ring.len();
    let mut j = len - 1;
    for i in 0..len {
        let ax = ring[i][0]; let ay = ring[i][1];
        let bx = ring[j][0]; let by = ring[j][1];
        let f = ax*by - bx*ay;
        cx += (ax+bx)*f;
        cy += (ay+by)*f;
        area += f * 3.0;
        j = i;
    }
    if area == 0.0 { return PCell::new(ring[0][0], ring[0][1], 0.0, polygon); }
    PCell::new(cx/area, cy/area, 0.0, polygon)
}

fn mapbox_polylabel(polygon: &[Vec<[f64; 2]>], precision: f64) -> [f64; 3] {
    let outer = &polygon[0];
    let (mut mnx, mut mxx) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut mny, mut mxy) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in outer {
        mnx = mnx.min(p[0]); mxx = mxx.max(p[0]);
        mny = mny.min(p[1]); mxy = mxy.max(p[1]);
    }
    let sx = mxx - mnx; let sy = mxy - mny;
    let cell_size = sx.min(sy);
    let mut h = cell_size / 2.0;
    if cell_size == 0.0 { return [mnx, mny, 0.0]; }

    let mut queue: std::collections::BinaryHeap<PCell> = std::collections::BinaryHeap::new();
    let mut x = mnx;
    while x < mxx {
        let mut y = mny;
        while y < mxy {
            queue.push(PCell::new(x + h, y + h, h, polygon));
            y += cell_size;
        }
        x += cell_size;
    }

    let mut best = pl_centroid_cell(polygon);
    let bbox_c = PCell::new(mnx + sx/2.0, mny + sy/2.0, 0.0, polygon);
    if bbox_c.d > best.d { best = bbox_c; }

    while let Some(cell) = queue.pop() {
        if cell.d > best.d { best = cell; }
        if cell.mx - best.d <= precision { continue; }
        h = cell.h / 2.0;
        queue.push(PCell::new(cell.cx - h, cell.cy - h, h, polygon));
        queue.push(PCell::new(cell.cx + h, cell.cy - h, h, polygon));
        queue.push(PCell::new(cell.cx - h, cell.cy + h, h, polygon));
        queue.push(PCell::new(cell.cx + h, cell.cy + h, h, polygon));
    }
    [best.cx, best.cy, best.d]
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

impl Index<usize> for Polyline {
    type Output = [f64];

    fn index(&self, index: usize) -> &Self::Output {
        if index >= self.point_count() {
            panic!("Index out of range");
        }
        let idx = index * 3;
        &self.coords[idx..idx + 3]
    }
}

impl IndexMut<usize> for Polyline {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        if index >= self.point_count() {
            panic!("Index out of range");
        }
        let idx = index * 3;
        &mut self.coords[idx..idx + 3]
    }
}

impl Polyline {
    pub fn boolean_op(a: &Polyline, b: &Polyline, clip_type: i32) -> Vec<Polyline> {
        crate::boolean_polyline::boolean_op(a, b, clip_type)
    }

    pub fn boolean_op_plane(a: &Polyline, b: &Polyline, plane: &crate::plane::Plane, clip_type: i32) -> Vec<Polyline> {
        let ox = plane.origin()[0]; let oy = plane.origin()[1]; let oz = plane.origin()[2];
        let xx = plane.x_axis()[0]; let xy = plane.x_axis()[1]; let xz = plane.x_axis()[2];
        let yx = plane.y_axis()[0]; let yy = plane.y_axis()[1]; let yz = plane.y_axis()[2];

        // Reuse thread-local Polyline wrappers for the projected 2D inputs.
        // This avoids 2 fresh Polyline allocations per call (each with a
        // "my_polyline" String, default Plane/Xform/Color, and a new coords
        // Vec) — saves ~5-10 ms across the 3515 boolean_op_plane calls in
        // compute_face_to_face.
        PROJECTED_BUFFERS.with(|cell| {
            let mut borrow = cell.borrow_mut();
            let (pa2d, pb2d) = &mut *borrow;

            // Project `a` into pa2d (reusing pa2d.coords capacity).
            let na = a.point_count();
            pa2d.coords.clear();
            pa2d.coords.reserve(na * 3);
            for i in 0..na {
                let dx = a.coords[i*3] - ox;
                let dy = a.coords[i*3+1] - oy;
                let dz = a.coords[i*3+2] - oz;
                pa2d.coords.push(dx*xx + dy*xy + dz*xz);
                pa2d.coords.push(dx*yx + dy*yy + dz*yz);
                pa2d.coords.push(0.0);
            }
            // Snap closing point if nearly closed (within 1mm)
            if na >= 4 {
                let dx = pa2d.coords[(na-1)*3] - pa2d.coords[0];
                let dy = pa2d.coords[(na-1)*3+1] - pa2d.coords[1];
                if dx*dx + dy*dy < 1.0 {
                    pa2d.coords[(na-1)*3] = pa2d.coords[0];
                    pa2d.coords[(na-1)*3+1] = pa2d.coords[1];
                }
            }

            // Project `b` into pb2d (reusing pb2d.coords capacity).
            let nb = b.point_count();
            pb2d.coords.clear();
            pb2d.coords.reserve(nb * 3);
            for i in 0..nb {
                let dx = b.coords[i*3] - ox;
                let dy = b.coords[i*3+1] - oy;
                let dz = b.coords[i*3+2] - oz;
                pb2d.coords.push(dx*xx + dy*xy + dz*xz);
                pb2d.coords.push(dx*yx + dy*yy + dz*yz);
                pb2d.coords.push(0.0);
            }
            if nb >= 4 {
                let dx = pb2d.coords[(nb-1)*3] - pb2d.coords[0];
                let dy = pb2d.coords[(nb-1)*3+1] - pb2d.coords[1];
                if dx*dx + dy*dy < 1.0 {
                    pb2d.coords[(nb-1)*3] = pb2d.coords[0];
                    pb2d.coords[(nb-1)*3+1] = pb2d.coords[1];
                }
            }

            // Ensure CCW winding (Vatti containment requires CCW).
            ensure_ccw_inplace(&mut pa2d.coords);
            ensure_ccw_inplace(&mut pb2d.coords);

            let mut results = crate::boolean_polyline::boolean_op(pa2d, pb2d, clip_type);

            // Inverse-transform results back to 3D.
            for r in results.iter_mut() {
                let n = r.coords.len() / 3;
                for i in 0..n {
                    let u = r.coords[i*3];
                    let v = r.coords[i*3+1];
                    r.coords[i*3]   = ox + u*xx + v*yx;
                    r.coords[i*3+1] = oy + u*xy + v*yy;
                    r.coords[i*3+2] = oz + u*xz + v*yz;
                }
            }
            results
        })
    }
}

/// CCW-orient a stride-3 2D polygon in place (z coord is 0 throughout).
/// Factored out of `boolean_op_plane` so it can be called on a raw `&mut Vec<f64>`
/// rather than needing a full `&mut Polyline` wrapper.
fn ensure_ccw_inplace(coords: &mut Vec<f64>) {
    let n = coords.len() / 3;
    let mut m = n;
    if m >= 4 {
        let dx = coords[(m-1)*3] - coords[0];
        let dy = coords[(m-1)*3+1] - coords[1];
        if dx*dx + dy*dy < 1e-10 { m -= 1; }
    }
    if m < 3 { return; }
    let mut area = 0.0;
    for i in 0..m {
        let j = (i + 1) % m;
        area += coords[i*3] * coords[j*3+1] - coords[j*3] * coords[i*3+1];
    }
    if area < 0.0 {
        for i in 0..n/2 {
            let j = n - 1 - i;
            coords.swap(i*3, j*3);
            coords.swap(i*3+1, j*3+1);
            coords.swap(i*3+2, j*3+2);
        }
    }
}

thread_local! {
    /// Reused projected-polygon buffers for `Polyline::boolean_op_plane`. Holds
    /// two full `Polyline` structs so their coord Vec capacity, name String,
    /// and default Plane/Xform/Color live across all calls — see the boolean
    /// workload in `Session::compute_face_to_face` which fires ~3500 times per
    /// `main_1` run.
    static PROJECTED_BUFFERS: std::cell::RefCell<(Polyline, Polyline)>
        = std::cell::RefCell::new((
            Polyline::from_coords(Vec::new()),
            Polyline::from_coords(Vec::new()),
        ));
}

impl fmt::Display for Polyline {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Polyline(guid={}, name={}, points={})",
            self.guid(),
            self.name,
            self.point_count()
        )
    }
}

#[cfg(test)]
#[path = "polyline_test.rs"]
mod polyline_test;
