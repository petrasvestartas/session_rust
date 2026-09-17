use crate::{Color, Point, SpatialOctree, Vector, Xform};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Add, AddAssign, Sub, SubAssign};
use std::sync::OnceLock;

/// A point cloud as flat coordinate, color and normal arrays with an optional LOD octree.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "PointCloud")]
pub struct PointCloud {
    #[serde(
        default,
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: OnceLock<String>, // Lazy guid.
    pub name: String, // Cloud name.
    #[serde(default = "PointCloud::default_point_size")]
    pub point_size: f64, // Display point size.
    #[serde(rename = "coords", default)]
    _coords: Vec<f64>, // Flat [x, y, z, ...].
    #[serde(rename = "colors", default)]
    _colors: Vec<i32>, // Flat [r, g, b, a, ...] as 0-255.
    #[serde(rename = "normals", default)]
    _normals: Vec<f64>, // Flat [nx, ny, nz, ...].
    #[serde(rename = "lod_min", default)]
    _lod_min: Vec<f64>, // Node cube min corner, 3 per node.
    #[serde(rename = "lod_size", default)]
    _lod_size: Vec<f64>, // Node cube edge length.
    #[serde(rename = "lod_spacing", default)]
    _lod_spacing: Vec<f64>, // Node grid-accept spacing.
    #[serde(rename = "lod_level", default)]
    _lod_level: Vec<i32>, // Node depth from the root.
    #[serde(rename = "lod_first", default)]
    _lod_first: Vec<i32>, // Node first point index.
    #[serde(rename = "lod_count", default)]
    _lod_count: Vec<i32>, // Node point count.
    #[serde(rename = "lod_children", default)]
    _lod_children: Vec<i32>, // Node child indices, 8 per node, -1 unused.
    #[serde(rename = "point_ids", default)]
    _point_ids: Vec<u32>, // Stable point ids parallel to the points.
}

impl Default for PointCloud {
    /// Constructs an empty cloud.
    fn default() -> Self {
        Self::from_coords(Vec::new(), Vec::new(), Vec::new())
    }
}

impl PointCloud {
    /// Constructs from points, normals and colors.
    pub fn new(points: Vec<Point>, normals: Vec<Vector>, colors: Vec<Color>) -> Self {
        let mut cloud = Self::default();
        cloud._coords.reserve(points.len() * 3);

        for p in &points {
            cloud.add_point(p);
        }

        cloud._normals.reserve(normals.len() * 3);

        for n in &normals {
            cloud.add_normal(n);
        }

        cloud._colors.reserve(colors.len() * 4);

        for c in &colors {
            cloud.add_color(c);
        }

        cloud
    }

    fn default_point_size() -> f64 {
        1.0
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

    /// Constructs from flat arrays: coords [x, y, z, ...], colors [r, g, b, a, ...] as 0-255, normals [nx, ny, nz, ...].
    pub fn from_coords(coords: Vec<f64>, colors: Vec<i32>, normals: Vec<f64>) -> Self {
        Self {
            guid: OnceLock::new(),
            name: "my_pointcloud".to_string(),
            point_size: 1.0,
            _coords: coords,
            _colors: colors,
            _normals: normals,
            _lod_min: Vec::new(),
            _lod_size: Vec::new(),
            _lod_spacing: Vec::new(),
            _lod_level: Vec::new(),
            _lod_first: Vec::new(),
            _lod_count: Vec::new(),
            _lod_children: Vec::new(),
            _point_ids: Vec::new(),
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Transforms points and normals in place.
    pub fn transform(&mut self, xform: &Xform) {
        for i in 0..self.point_count() {
            let point = self.get_point(i).transformed(xform);
            self.set_point(i, &point);
        }

        for i in 0..self.normal_count() {
            let normal = self.get_normal(i).transformed(xform);
            self.set_normal(i, &normal);
        }
    }

    /// Returns a transformed copy.
    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut result = self.clone();
        result.transform(xform);

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Points
    // ═══════════════════════════════════════════════════════════════════════════

    /// Returns the number of points.
    pub fn point_count(&self) -> usize {
        self._coords.len() / 3
    }

    /// Returns the number of points.
    pub fn len(&self) -> usize {
        self.point_count()
    }

    /// Returns whether the cloud has no points.
    pub fn is_empty(&self) -> bool {
        self._coords.is_empty()
    }

    /// Returns the point at index.
    pub fn get_point(&self, index: usize) -> Point {
        let idx = index * 3;
        Point::new(
            self._coords[idx],
            self._coords[idx + 1],
            self._coords[idx + 2],
        )
    }

    /// Sets the point at index.
    pub fn set_point(&mut self, index: usize, point: &Point) {
        let idx = index * 3;
        self._coords[idx] = point[0];
        self._coords[idx + 1] = point[1];
        self._coords[idx + 2] = point[2];
    }

    /// Appends a point.
    pub fn add_point(&mut self, point: &Point) {
        self._coords.push(point[0]);
        self._coords.push(point[1]);
        self._coords.push(point[2]);
    }

    /// Returns all points.
    pub fn get_points(&self) -> Vec<Point> {
        let mut points = Vec::with_capacity(self.point_count());

        for i in 0..self.point_count() {
            points.push(self.get_point(i));
        }

        points
    }

    /// Returns the flat coordinate array itself; get_point builds a Point per call.
    pub fn coords(&self) -> &[f64] {
        &self._coords
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Colors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Returns the number of colors.
    pub fn color_count(&self) -> usize {
        self._colors.len() / 4
    }

    /// Returns the color at index.
    pub fn get_color(&self, index: usize) -> Color {
        let idx = index * 4;
        Color::new(
            self._colors[idx] as f32 / 255.0,
            self._colors[idx + 1] as f32 / 255.0,
            self._colors[idx + 2] as f32 / 255.0,
            self._colors[idx + 3] as f32 / 255.0,
        )
    }

    /// Sets the color at index.
    pub fn set_color(&mut self, index: usize, color: &Color) {
        let idx = index * 4;
        self._colors[idx] = (color.r * 255.0).round() as i32;
        self._colors[idx + 1] = (color.g * 255.0).round() as i32;
        self._colors[idx + 2] = (color.b * 255.0).round() as i32;
        self._colors[idx + 3] = (color.a * 255.0).round() as i32;
    }

    /// Appends a color.
    pub fn add_color(&mut self, color: &Color) {
        self._colors.push((color.r * 255.0).round() as i32);
        self._colors.push((color.g * 255.0).round() as i32);
        self._colors.push((color.b * 255.0).round() as i32);
        self._colors.push((color.a * 255.0).round() as i32);
    }

    /// Returns all colors.
    pub fn get_colors(&self) -> Vec<Color> {
        let mut colors = Vec::with_capacity(self.color_count());

        for i in 0..self.color_count() {
            colors.push(self.get_color(i));
        }

        colors
    }

    /// Returns the flat 0-255 color array itself, the encoding the proto carries.
    pub fn colors(&self) -> &[i32] {
        &self._colors
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Normals
    // ═══════════════════════════════════════════════════════════════════════════

    /// Returns the number of normals.
    pub fn normal_count(&self) -> usize {
        self._normals.len() / 3
    }

    /// Returns the normal at index.
    pub fn get_normal(&self, index: usize) -> Vector {
        let idx = index * 3;
        Vector::new(
            self._normals[idx],
            self._normals[idx + 1],
            self._normals[idx + 2],
        )
    }

    /// Sets the normal at index.
    pub fn set_normal(&mut self, index: usize, normal: &Vector) {
        let idx = index * 3;
        self._normals[idx] = normal[0];
        self._normals[idx + 1] = normal[1];
        self._normals[idx + 2] = normal[2];
    }

    /// Appends a normal.
    pub fn add_normal(&mut self, normal: &Vector) {
        self._normals.push(normal[0]);
        self._normals.push(normal[1]);
        self._normals.push(normal[2]);
    }

    /// Returns all normals.
    pub fn get_normals(&self) -> Vec<Vector> {
        let mut normals = Vec::with_capacity(self.normal_count());

        for i in 0..self.normal_count() {
            normals.push(self.get_normal(i));
        }

        normals
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SESSION_VIEWER
    // ═══════════════════════════════════════════════════════════════════════════

    /// The flat normal array itself, read by the viewer's cloud walk
    pub fn normals(&self) -> &[f64] {
        &self._normals
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LOD octree
    // ═══════════════════════════════════════════════════════════════════════════

    /// Builds the octree and permute the arrays into octree order, so a node is one contiguous range.
    pub fn build_lod(&mut self, root_spacing: f64, leaf_capacity: usize) {
        let tree = SpatialOctree::from_coords(&self._coords, root_spacing, leaf_capacity);
        let order = tree.order();

        if self._point_ids.is_empty() {
            for i in 0..self.point_count() {
                self._point_ids.push(i as u32);
            }
        }

        let has_colors = self._colors.len() == order.len() * 4;
        let has_normals = self._normals.len() == order.len() * 3;
        let mut coords = Vec::with_capacity(self._coords.len());
        let mut colors = Vec::with_capacity(self._colors.len());
        let mut normals = Vec::with_capacity(self._normals.len());
        let mut ids = Vec::with_capacity(self._point_ids.len());

        for &idx in order {
            ids.push(self._point_ids[idx]);

            for k in 0..3 {
                coords.push(self._coords[idx * 3 + k]);
            }

            if has_colors {
                for k in 0..4 {
                    colors.push(self._colors[idx * 4 + k]);
                }
            }

            if has_normals {
                for k in 0..3 {
                    normals.push(self._normals[idx * 3 + k]);
                }
            }
        }

        self._coords = coords;
        self._point_ids = ids;

        if has_colors {
            self._colors = colors;
        }

        if has_normals {
            self._normals = normals;
        }

        self._lod_min.clear();
        self._lod_size.clear();
        self._lod_spacing.clear();
        self._lod_level.clear();
        self._lod_first.clear();
        self._lod_count.clear();
        self._lod_children.clear();

        for i in 0..tree.node_count() {
            let cube = tree.node_cube(i);
            let range = tree.node_range(i);
            let kids = tree.children(i);

            for k in 0..3 {
                self._lod_min.push(cube.0[k] - cube.1 * 0.5);
            }

            self._lod_size.push(cube.1);
            self._lod_spacing.push(tree.node_spacing(i));
            self._lod_level.push(tree.node_level(i) as i32);
            self._lod_first.push(range.0 as i32);
            self._lod_count.push(range.1 as i32);

            for k in 0..8 {
                self._lod_children
                    .push(if k < kids.len() { kids[k] as i32 } else { -1 });
            }
        }
    }

    /// Returns whether an octree has been built.
    pub fn has_lod(&self) -> bool {
        !self._lod_size.is_empty()
    }

    /// Returns the number of octree nodes.
    pub fn lod_node_count(&self) -> usize {
        self._lod_size.len()
    }

    /// Returns the node cube center and edge length.
    pub fn lod_cube(&self, i: usize) -> (Point, f64) {
        let half = self._lod_size[i] * 0.5;
        (
            Point::new(
                self._lod_min[i * 3] + half,
                self._lod_min[i * 3 + 1] + half,
                self._lod_min[i * 3 + 2] + half,
            ),
            self._lod_size[i],
        )
    }

    /// Returns the grid-accept spacing of a node.
    pub fn lod_spacing(&self, i: usize) -> f64 {
        self._lod_spacing[i]
    }

    /// Returns the node depth from the root.
    pub fn lod_level(&self, i: usize) -> i32 {
        self._lod_level[i]
    }

    /// Returns the node point range as (first, count) into the reordered arrays.
    pub fn lod_range(&self, i: usize) -> (i32, i32) {
        (self._lod_first[i], self._lod_count[i])
    }

    /// Returns the present child node indices compacted into 8 slots, -1 unused.
    pub fn lod_children(&self, i: usize) -> Vec<i32> {
        self._lod_children[i * 8..i * 8 + 8].to_vec()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Point ids
    // ═══════════════════════════════════════════════════════════════════════════

    /// Returns the stable ids parallel to the points, minted by the first build_lod; empty before that.
    pub fn point_ids(&self) -> &[u32] {
        &self._point_ids
    }

    /// Returns the stable id of the point at index; the index itself before a tree is built.
    pub fn point_id(&self, index: usize) -> u32 {
        if self._point_ids.is_empty() {
            index as u32
        } else {
            self._point_ids[index]
        }
    }

    /// Returns the current index of a stable id, None when the cloud has no such point.
    #[allow(clippy::manual_find)]
    pub fn index_of_id(&self, id: u32) -> Option<usize> {
        if self._point_ids.is_empty() {
            return if (id as usize) < self.point_count() {
                Some(id as usize)
            } else {
                None
            };
        }

        for i in 0..self._point_ids.len() {
            if self._point_ids[i] == id {
                return Some(i);
            }
        }

        None
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
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Ok(Self::from_proto(crate::proto::PointCloud::decode(data)?))
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

    /// The proto message; pb_dumps encodes it and Session embeds it
    pub fn to_proto(&self) -> crate::proto::PointCloud {
        let mut colors = Vec::with_capacity(self._colors.len());

        for &c in &self._colors {
            colors.push(c as u32);
        }

        crate::proto::PointCloud {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            point_size: self.point_size,
            coords: self._coords.clone(),
            colors,
            normals: self._normals.clone(),
            lod_min: self._lod_min.clone(),
            lod_size: self._lod_size.clone(),
            lod_spacing: self._lod_spacing.clone(),
            lod_level: self._lod_level.clone(),
            lod_first: self._lod_first.clone(),
            lod_count: self._lod_count.clone(),
            lod_children: self._lod_children.clone(),
            point_ids: self._point_ids.clone(),
        }
    }

    /// Cloud from a decoded proto message
    pub fn from_proto(proto: crate::proto::PointCloud) -> Self {
        let mut colors = Vec::with_capacity(proto.colors.len());

        for &c in &proto.colors {
            colors.push(c as i32);
        }

        let mut cloud = Self::from_coords(proto.coords, colors, proto.normals);

        if !proto.guid.is_empty() {
            cloud.set_guid(proto.guid);
        }

        cloud.name = proto.name;

        if proto.point_size > 0.0 {
            cloud.point_size = proto.point_size;
        }

        cloud._lod_min = proto.lod_min;
        cloud._lod_size = proto.lod_size;
        cloud._lod_spacing = proto.lod_spacing;
        cloud._lod_level = proto.lod_level;
        cloud._lod_first = proto.lod_first;
        cloud._lod_count = proto.lod_count;
        cloud._lod_children = proto.lod_children;
        cloud._point_ids = proto.point_ids;

        cloud
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════

    /// "N points"
    pub fn str(&self) -> String {
        format!("{} points", self.point_count())
    }

    /// Returns "PointCloud(name, N points, N colors, N normals)".
    pub fn repr(&self) -> String {
        format!(
            "PointCloud({}, {} points, {} colors, {} normals)",
            self.name,
            self.point_count(),
            self.color_count(),
            self.normal_count()
        )
    }
}

impl fmt::Display for PointCloud {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

impl PartialEq for PointCloud {
    /// Compares name, arrays, LOD ranges and point ids; guid ignored.
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && self._coords == other._coords
            && self._colors == other._colors
            && self._normals == other._normals
            && self._lod_first == other._lod_first
            && self._lod_count == other._lod_count
            && self._point_ids == other._point_ids
    }
}

impl AddAssign<&Vector> for PointCloud {
    /// Translates in place.
    fn add_assign(&mut self, other: &Vector) {
        for i in (0..self._coords.len()).step_by(3) {
            self._coords[i] += other[0];
            self._coords[i + 1] += other[1];
            self._coords[i + 2] += other[2];
        }
    }
}

impl SubAssign<&Vector> for PointCloud {
    /// Translates back in place.
    fn sub_assign(&mut self, other: &Vector) {
        for i in (0..self._coords.len()).step_by(3) {
            self._coords[i] -= other[0];
            self._coords[i + 1] -= other[1];
            self._coords[i + 2] -= other[2];
        }
    }
}

impl Add<&Vector> for PointCloud {
    type Output = PointCloud;

    /// Returns a translated copy.
    fn add(self, other: &Vector) -> PointCloud {
        let mut result = self;
        result += other;

        result
    }
}

impl Sub<&Vector> for PointCloud {
    type Output = PointCloud;

    /// Returns a copy translated back.
    fn sub(self, other: &Vector) -> PointCloud {
        let mut result = self;
        result -= other;

        result
    }
}

impl Add<&Vector> for &PointCloud {
    type Output = PointCloud;

    /// Returns a translated copy.
    fn add(self, other: &Vector) -> PointCloud {
        self.duplicate() + other
    }
}

impl Sub<&Vector> for &PointCloud {
    type Output = PointCloud;

    /// Returns a copy translated back.
    fn sub(self, other: &Vector) -> PointCloud {
        self.duplicate() - other
    }
}
