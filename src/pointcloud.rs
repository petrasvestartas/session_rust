use crate::{Color, Point, Vector, Xform};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::fmt;
use std::ops::{Add, AddAssign, Sub, SubAssign};

/// A point cloud with coordinates, normals, and colors stored as flat arrays.
#[derive(Debug, Clone)]
pub struct PointCloud {
    /// Lazily generated unique identifier
    guid: std::sync::OnceLock<String>,
    /// Human-readable name
    pub name: String,
    /// Point size for rendering
    pub point_size: f64,
    /// Flat coords [x0, y0, z0, x1, y1, z1, ...]
    _coords: Vec<f64>,
    /// Flat colors [r0, g0, b0, a0, ...]
    _colors: Vec<i32>,
    /// Flat normals [nx0, ny0, nz0, ...]
    _normals: Vec<f64>,
    /// LOD octree over the points, one flat array per SpatialOctree node field. Built by
    /// build_lod(), which PERMUTES the three arrays above into octree order - so a node is one
    /// contiguous (_lod_first, _lod_count) range and the order permutation never has to be
    /// stored. Empty means no octree.
    _lod_min: Vec<f64>,
    _lod_size: Vec<f64>,
    _lod_spacing: Vec<f64>,
    _lod_level: Vec<i32>,
    _lod_first: Vec<i32>,
    _lod_count: Vec<i32>,
    _lod_children: Vec<i32>,
    /// STABLE per-point ids, one per point, parallel to _coords. Assigned once by the first
    /// build_lod and permuted with the points ever after, so an index that moves does not take
    /// the point's identity with it. Empty = no tree yet, so the index IS the id.
    _point_ids: Vec<u32>,
}

impl Default for PointCloud {
    fn default() -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: "my_pointcloud".to_string(),
            point_size: 1.0,
            _coords: Vec::new(),
            _colors: Vec::new(),
            _normals: Vec::new(),
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
}

impl PointCloud {
    /// Constructor with points, normals, and colors
    pub fn new(points: Vec<Point>, normals: Vec<Vector>, colors: Vec<Color>) -> Self {
        let mut pc = Self::default();

        pc._coords.reserve(points.len() * 3);
        for p in &points {
            pc._coords.push(p[0]);
            pc._coords.push(p[1]);
            pc._coords.push(p[2]);
        }

        pc._colors.reserve(colors.len() * 4);
        for c in &colors {
            pc._colors.push((c.r * 255.0).round() as i32);
            pc._colors.push((c.g * 255.0).round() as i32);
            pc._colors.push((c.b * 255.0).round() as i32);
            pc._colors.push((c.a * 255.0).round() as i32);
        }

        pc._normals.reserve(normals.len() * 3);
        for n in &normals {
            pc._normals.push(n[0]);
            pc._normals.push(n[1]);
            pc._normals.push(n[2]);
        }

        pc
    }

    /// Create from flat arrays of coords, colors, and normals
    pub fn from_coords(coords: Vec<f64>, colors: Vec<i32>, normals: Vec<f64>) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
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

    /// Lazy GUID accessor
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the GUID explicitly
    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Point Access
    // ═══════════════════════════════════════════════════════════════════════════

    /// Number of points
    pub fn point_count(&self) -> usize {
        self._coords.len() / 3
    }

    /// Alias for point_count()
    pub fn len(&self) -> usize {
        self.point_count()
    }

    /// True when the cloud has no points
    pub fn is_empty(&self) -> bool {
        self._coords.is_empty()
    }

    /// Get point at index
    pub fn get_point(&self, index: usize) -> Point {
        let idx = index * 3;
        Point::new(
            self._coords[idx],
            self._coords[idx + 1],
            self._coords[idx + 2],
        )
    }

    /// Set point at index
    pub fn set_point(&mut self, index: usize, point: &Point) {
        let idx = index * 3;
        self._coords[idx] = point[0];
        self._coords[idx + 1] = point[1];
        self._coords[idx + 2] = point[2];
    }

    /// Append a point to the cloud
    pub fn add_point(&mut self, point: &Point) {
        self._coords.push(point[0]);
        self._coords.push(point[1]);
        self._coords.push(point[2]);
    }

    /// The flat coordinate array itself, [x0, y0, z0, x1, ...]. A renderer walking millions of
    /// points cannot afford `get_point` per point: that builds a `Point`, and a `Point` owns a
    /// name and a colour, so a 13.8M-point scan spends most of its walk in the allocator.
    pub fn coords(&self) -> &[f64] {
        &self._coords
    }

    /// Get all points as a vector
    pub fn get_points(&self) -> Vec<Point> {
        let mut points = Vec::with_capacity(self.point_count());
        for i in 0..self.point_count() {
            let idx = i * 3;
            points.push(Point::new(
                self._coords[idx],
                self._coords[idx + 1],
                self._coords[idx + 2],
            ));
        }
        points
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Color Access
    // ═══════════════════════════════════════════════════════════════════════════

    /// Number of colors
    pub fn color_count(&self) -> usize {
        self._colors.len() / 4
    }

    /// Get color at index
    pub fn get_color(&self, index: usize) -> Color {
        let idx = index * 4;
        Color::new(
            self._colors[idx] as f32 / 255.0,
            self._colors[idx + 1] as f32 / 255.0,
            self._colors[idx + 2] as f32 / 255.0,
            self._colors[idx + 3] as f32 / 255.0,
        )
    }

    /// Set color at index
    pub fn set_color(&mut self, index: usize, color: &Color) {
        let idx = index * 4;
        self._colors[idx] = (color.r * 255.0).round() as i32;
        self._colors[idx + 1] = (color.g * 255.0).round() as i32;
        self._colors[idx + 2] = (color.b * 255.0).round() as i32;
        self._colors[idx + 3] = (color.a * 255.0).round() as i32;
    }

    /// Append a color to the cloud
    pub fn add_color(&mut self, color: &Color) {
        self._colors.push((color.r * 255.0).round() as i32);
        self._colors.push((color.g * 255.0).round() as i32);
        self._colors.push((color.b * 255.0).round() as i32);
        self._colors.push((color.a * 255.0).round() as i32);
    }

    /// The flat colour array itself, [r0, g0, b0, a0, r1, ...] as 0-255 - the same encoding the
    /// proto carries. Same reason as `coords`: `get_color` builds a `Color`, which owns a name.
    pub fn colors(&self) -> &[i32] {
        &self._colors
    }

    /// The flat normal array, [nx0, ny0, nz0, ...]; empty when the cloud has none
    pub fn normals(&self) -> &[f64] {
        &self._normals
    }

    /// Get all colors as a vector
    pub fn get_colors(&self) -> Vec<Color> {
        let mut colors = Vec::with_capacity(self.color_count());
        for i in 0..self.color_count() {
            let idx = i * 4;
            colors.push(Color::new(
                self._colors[idx] as f32 / 255.0,
                self._colors[idx + 1] as f32 / 255.0,
                self._colors[idx + 2] as f32 / 255.0,
                self._colors[idx + 3] as f32 / 255.0,
            ));
        }
        colors
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Normal Access
    // ═══════════════════════════════════════════════════════════════════════════

    /// Number of normals
    pub fn normal_count(&self) -> usize {
        self._normals.len() / 3
    }

    /// Get normal at index
    pub fn get_normal(&self, index: usize) -> Vector {
        let idx = index * 3;
        Vector::new(
            self._normals[idx],
            self._normals[idx + 1],
            self._normals[idx + 2],
        )
    }

    /// Set normal at index
    pub fn set_normal(&mut self, index: usize, normal: &Vector) {
        let idx = index * 3;
        self._normals[idx] = normal[0];
        self._normals[idx + 1] = normal[1];
        self._normals[idx + 2] = normal[2];
    }

    /// Append a normal to the cloud
    pub fn add_normal(&mut self, normal: &Vector) {
        self._normals.push(normal[0]);
        self._normals.push(normal[1]);
        self._normals.push(normal[2]);
    }

    /// Get all normals as a vector
    pub fn get_normals(&self) -> Vec<Vector> {
        let mut normals = Vec::with_capacity(self.normal_count());
        for i in 0..self.normal_count() {
            let idx = i * 3;
            normals.push(Vector::new(
                self._normals[idx],
                self._normals[idx + 1],
                self._normals[idx + 2],
            ));
        }
        normals
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String Representations
    // ═══════════════════════════════════════════════════════════════════════════

    /// Simple string form (like Python __str__)
    pub fn str(&self) -> String {
        format!("{} points", self.point_count())
    }

    /// Detailed representation (like Python __repr__)
    pub fn repr(&self) -> String {
        format!(
            "PointCloud({}, {} points, {} colors, {} normals)",
            self.name,
            self.point_count(),
            self.color_count(),
            self.normal_count()
        )
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Duplicate and Equality
    // ═══════════════════════════════════════════════════════════════════════════

    /// Deep copy this cloud with a new guid
    pub fn duplicate(&self) -> Self {
        let mut result = self.clone();
        result.guid = std::sync::OnceLock::new();
        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Apply a transformation to this cloud in place
    pub fn transform(&mut self, xform: &Xform) {
        for i in 0..self.point_count() {
            let idx = i * 3;
            let mut pt = Point::new(
                self._coords[idx],
                self._coords[idx + 1],
                self._coords[idx + 2],
            );
            pt.transform(xform);
            self._coords[idx] = pt[0];
            self._coords[idx + 1] = pt[1];
            self._coords[idx + 2] = pt[2];
        }

        for i in 0..self.normal_count() {
            let idx = i * 3;
            let mut n = Vector::new(
                self._normals[idx],
                self._normals[idx + 1],
                self._normals[idx + 2],
            );
            n.transform(xform);
            self._normals[idx] = n[0];
            self._normals[idx + 1] = n[1];
            self._normals[idx + 2] = n[2];
        }
    }

    /// Return a copy of this cloud with its xform applied
    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut result = self.clone();
        result.transform(xform);
        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LOD Octree
    // ═══════════════════════════════════════════════════════════════════════════

    /// Build the LOD octree and REORDER the points into octree order. Every point's index
    /// changes; a node becomes one contiguous range. Expensive - about 10 s on 14 M points -
    /// so it is called once by whoever writes the cloud, never per construction.
    pub fn build_lod(&mut self, root_spacing: f64, leaf_capacity: usize) {
        let tree = crate::SpatialOctree::from_coords(&self._coords, root_spacing, leaf_capacity);
        let order = tree.order();

        // Identity is minted HERE, before the first permutation, so an id records where a point
        // began. After this the ids travel with the points and the index is free to move.
        if self._point_ids.is_empty() {
            self._point_ids = (0..self._coords.len() as u32 / 3).collect();
        }

        // Permute the three parallel arrays into octree order. This is what lets a node be one
        // (first, count) range, so `order` itself never has to be stored - 4 bytes a point.
        let mut coords = Vec::with_capacity(self._coords.len());
        let mut colors = Vec::with_capacity(self._colors.len());
        let mut normals = Vec::with_capacity(self._normals.len());
        let mut ids = Vec::with_capacity(self._point_ids.len());
        let has_colors = self._colors.len() == order.len() * 4;
        let has_normals = self._normals.len() == order.len() * 3;
        for &idx in order {
            ids.push(self._point_ids[idx]);
            coords.extend_from_slice(&self._coords[idx * 3..idx * 3 + 3]);
            if has_colors {
                colors.extend_from_slice(&self._colors[idx * 4..idx * 4 + 4]);
            }
            if has_normals {
                normals.extend_from_slice(&self._normals[idx * 3..idx * 3 + 3]);
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

        let n = tree.node_count();
        self._lod_min.clear();
        self._lod_size.clear();
        self._lod_spacing.clear();
        self._lod_level.clear();
        self._lod_first.clear();
        self._lod_count.clear();
        self._lod_children.clear();
        for i in 0..n {
            let (center, size) = tree.node_cube(i);
            self._lod_min.push(center[0] - size * 0.5);
            self._lod_min.push(center[1] - size * 0.5);
            self._lod_min.push(center[2] - size * 0.5);
            self._lod_size.push(size);
            self._lod_spacing.push(tree.node_spacing(i));
            self._lod_level.push(tree.node_level(i) as i32);
            let (first, count) = tree.node_range(i);
            self._lod_first.push(first as i32);
            self._lod_count.push(count as i32);
            let kids = tree.children(i);
            for k in 0..8 {
                self._lod_children
                    .push(kids.get(k).map_or(-1, |&c| c as i32));
            }
        }
    }

    /// True when an octree has been built
    pub fn has_lod(&self) -> bool {
        !self._lod_size.is_empty()
    }

    /// Number of octree nodes
    pub fn lod_node_count(&self) -> usize {
        self._lod_size.len()
    }

    /// Node cube: center and edge length
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

    /// Grid-accept spacing of a node
    pub fn lod_spacing(&self, i: usize) -> f64 {
        self._lod_spacing[i]
    }

    /// Node depth from the root
    pub fn lod_level(&self, i: usize) -> i32 {
        self._lod_level[i]
    }

    /// Node point range as (first, count) into the reordered arrays
    pub fn lod_range(&self, i: usize) -> (i32, i32) {
        (self._lod_first[i], self._lod_count[i])
    }

    /// Present child node indices, compacted, -1 padding
    pub fn lod_children(&self, i: usize) -> Vec<i32> {
        self._lod_children[i * 8..i * 8 + 8].to_vec()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Stable Point Ids
    // ═══════════════════════════════════════════════════════════════════════════

    /// The stable ids, parallel to the points. Empty until a tree is built.
    pub fn point_ids(&self) -> &[u32] {
        &self._point_ids
    }

    /// The stable id of a point, by its CURRENT index. Falls back to the index itself while no
    /// tree has been built, which is exactly what the id would have been.
    pub fn point_id(&self, index: usize) -> u32 {
        if self._point_ids.is_empty() {
            index as u32
        } else {
            self._point_ids[index]
        }
    }

    /// Where a stable id lives NOW, or None if this cloud has no such point. Linear: a caller
    /// resolving many ids should build its own map.
    pub fn index_of_id(&self, id: u32) -> Option<usize> {
        if self._point_ids.is_empty() {
            return ((id as usize) < self._coords.len() / 3).then_some(id as usize);
        }
        self._point_ids.iter().position(|&v| v == id)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON Serialization
    // ═══════════════════════════════════════════════════════════════════════════

    /// Serialize to JSON string
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from JSON string
    pub fn jsonload(json_str: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_str)?)
    }

    /// Convert to JSON string (infallible fallback)
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    /// Load from JSON string (infallible fallback)
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_else(|_| Self::default())
    }

    /// Write JSON to file
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json_str = self.jsondump()?;
        std::fs::write(filepath, json_str)?;
        Ok(())
    }

    /// Read JSON from file
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json_str = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json_str)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf Serialization
    // ═══════════════════════════════════════════════════════════════════════════

    /// Convert to protobuf binary bytes
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        self.to_proto().encode_to_vec()
    }

    /// The proto struct itself — pb_dumps encodes it; Session embeds it directly.
    pub fn to_proto(&self) -> crate::proto::PointCloud {
        use crate::proto;

        proto::PointCloud {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            coords: self._coords.iter().map(|&v| v as f64).collect(),
            colors: self._colors.iter().map(|&c| c as u32).collect(),
            normals: self._normals.iter().map(|&v| v as f64).collect(),
            point_size: self.point_size as f64,
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

    /// Load from protobuf binary bytes
    pub fn pb_loads(data: &[u8]) -> Self {
        use crate::proto;
        use prost::Message;
        Self::from_proto(proto::PointCloud::decode(data).expect("Failed to decode protobuf"))
    }

    /// Build from an already-decoded proto — pb_loads decodes then calls this.
    pub fn from_proto(proto: crate::proto::PointCloud) -> Self {
        let mut pc = Self::from_coords(
            proto.coords.into_iter().map(|v| v as f64).collect(),
            proto.colors.into_iter().map(|c| c as i32).collect(),
            proto.normals.into_iter().map(|v| v as f64).collect(),
        );
        pc.set_guid(proto.guid);
        pc.name = proto.name;
        pc.point_size = if proto.point_size > 0.0 {
            proto.point_size as f64
        } else {
            1.0
        };
        pc._lod_min = proto.lod_min;
        pc._lod_size = proto.lod_size;
        pc._lod_spacing = proto.lod_spacing;
        pc._lod_level = proto.lod_level;
        pc._lod_first = proto.lod_first;
        pc._lod_count = proto.lod_count;
        pc._lod_children = proto.lod_children;
        pc._point_ids = proto.point_ids;

        pc
    }

    /// Write protobuf to file
    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    /// Read protobuf from file
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// PartialEq
// ═══════════════════════════════════════════════════════════════════════════

impl PartialEq for PointCloud {
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

// ═══════════════════════════════════════════════════════════════════════════
// No-copy Operators
// ═══════════════════════════════════════════════════════════════════════════

impl AddAssign<Vector> for PointCloud {
    fn add_assign(&mut self, other: Vector) {
        for i in 0..self.point_count() {
            let idx = i * 3;
            self._coords[idx] += other[0];
            self._coords[idx + 1] += other[1];
            self._coords[idx + 2] += other[2];
        }
    }
}

impl SubAssign<Vector> for PointCloud {
    fn sub_assign(&mut self, other: Vector) {
        for i in 0..self.point_count() {
            let idx = i * 3;
            self._coords[idx] -= other[0];
            self._coords[idx + 1] -= other[1];
            self._coords[idx + 2] -= other[2];
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Copy Operators
// ═══════════════════════════════════════════════════════════════════════════

impl Add<Vector> for PointCloud {
    type Output = PointCloud;

    fn add(self, other: Vector) -> PointCloud {
        let mut result = self.clone();
        result += other;
        result
    }
}

impl Add<Vector> for &PointCloud {
    type Output = PointCloud;

    fn add(self, other: Vector) -> PointCloud {
        let mut result = self.clone();
        result += other;
        result
    }
}

impl Sub<Vector> for PointCloud {
    type Output = PointCloud;

    fn sub(self, other: Vector) -> PointCloud {
        let mut result = self.clone();
        result -= other;
        result
    }
}

impl Sub<Vector> for &PointCloud {
    type Output = PointCloud;

    fn sub(self, other: Vector) -> PointCloud {
        let mut result = self.clone();
        result -= other;
        result
    }
}

impl fmt::Display for PointCloud {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.repr())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Custom Serialization - Flat arrays for efficiency
// ═══════════════════════════════════════════════════════════════════════════

impl Serialize for PointCloud {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("PointCloud", 16)?;

        state.serialize_field("type", "PointCloud")?;
        state.serialize_field("guid", self.guid())?;
        state.serialize_field("name", &self.name)?;
        state.serialize_field("coords", &self._coords)?;
        state.serialize_field("colors", &self._colors)?;
        state.serialize_field("normals", &self._normals)?;
        state.serialize_field("point_size", &self.point_size)?;
        state.serialize_field("lod_children", &self._lod_children)?;
        state.serialize_field("lod_count", &self._lod_count)?;
        state.serialize_field("lod_first", &self._lod_first)?;
        state.serialize_field("lod_level", &self._lod_level)?;
        state.serialize_field("lod_min", &self._lod_min)?;
        state.serialize_field("lod_size", &self._lod_size)?;
        state.serialize_field("lod_spacing", &self._lod_spacing)?;
        state.serialize_field("point_ids", &self._point_ids)?;

        state.end()
    }
}

impl<'de> Deserialize<'de> for PointCloud {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        use serde::de::{self, MapAccess, Visitor};

        #[derive(Deserialize)]
        #[serde(field_identifier, rename_all = "lowercase")]
        enum Field {
            Type,
            Guid,
            Name,
            Coords,
            Colors,
            Normals,
            #[serde(rename = "point_size")]
            PointSize,
            #[serde(rename = "lod_children")]
            LodChildren,
            #[serde(rename = "lod_count")]
            LodCount,
            #[serde(rename = "lod_first")]
            LodFirst,
            #[serde(rename = "lod_level")]
            LodLevel,
            #[serde(rename = "lod_min")]
            LodMin,
            #[serde(rename = "lod_size")]
            LodSize,
            #[serde(rename = "lod_spacing")]
            LodSpacing,
            #[serde(rename = "point_ids")]
            PointIds,
        }

        struct PointCloudVisitor;

        impl<'de> Visitor<'de> for PointCloudVisitor {
            type Value = PointCloud;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct PointCloud")
            }

            fn visit_map<V>(self, mut map: V) -> Result<PointCloud, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut guid = None;
                let mut name = None;
                let mut coords: Option<Vec<f64>> = None;
                let mut colors: Option<Vec<i32>> = None;
                let mut normals: Option<Vec<f64>> = None;
                let mut point_size = None;
                let mut lod_children: Option<Vec<i32>> = None;
                let mut lod_count: Option<Vec<i32>> = None;
                let mut lod_first: Option<Vec<i32>> = None;
                let mut lod_level: Option<Vec<i32>> = None;
                let mut lod_min: Option<Vec<f64>> = None;
                let mut lod_size: Option<Vec<f64>> = None;
                let mut lod_spacing: Option<Vec<f64>> = None;
                let mut point_ids: Option<Vec<u32>> = None;

                while let Some(key) = map.next_key()? {
                    match key {
                        Field::Type => {
                            let _: String = map.next_value()?;
                        }
                        Field::Guid => {
                            guid = Some(map.next_value()?);
                        }
                        Field::Name => {
                            name = Some(map.next_value()?);
                        }
                        Field::Coords => {
                            coords = Some(map.next_value()?);
                        }
                        Field::Colors => {
                            colors = Some(map.next_value()?);
                        }
                        Field::Normals => {
                            normals = Some(map.next_value()?);
                        }
                        Field::PointSize => {
                            point_size = Some(map.next_value()?);
                        }
                        Field::LodChildren => {
                            lod_children = Some(map.next_value()?);
                        }
                        Field::LodCount => {
                            lod_count = Some(map.next_value()?);
                        }
                        Field::LodFirst => {
                            lod_first = Some(map.next_value()?);
                        }
                        Field::LodLevel => {
                            lod_level = Some(map.next_value()?);
                        }
                        Field::LodMin => {
                            lod_min = Some(map.next_value()?);
                        }
                        Field::LodSize => {
                            lod_size = Some(map.next_value()?);
                        }
                        Field::LodSpacing => {
                            lod_spacing = Some(map.next_value()?);
                        }
                        Field::PointIds => {
                            point_ids = Some(map.next_value()?);
                        }
                    }
                }

                let guid_str = guid.ok_or_else(|| de::Error::missing_field("guid"))?;
                let name = name.ok_or_else(|| de::Error::missing_field("name"))?;
                let coords = coords.ok_or_else(|| de::Error::missing_field("coords"))?;
                let colors = colors.ok_or_else(|| de::Error::missing_field("colors"))?;
                let normals = normals.ok_or_else(|| de::Error::missing_field("normals"))?;
                let point_size = point_size.unwrap_or(1.0);

                Ok(PointCloud {
                    guid: {
                        let c = std::sync::OnceLock::new();
                        let _ = c.set(guid_str);
                        c
                    },
                    name,
                    point_size,
                    _coords: coords,
                    _colors: colors,
                    _normals: normals,
                    _lod_min: lod_min.unwrap_or_default(),
                    _lod_size: lod_size.unwrap_or_default(),
                    _lod_spacing: lod_spacing.unwrap_or_default(),
                    _lod_level: lod_level.unwrap_or_default(),
                    _lod_first: lod_first.unwrap_or_default(),
                    _lod_count: lod_count.unwrap_or_default(),
                    _lod_children: lod_children.unwrap_or_default(),
                    _point_ids: point_ids.unwrap_or_default(),
                })
            }
        }

        const FIELDS: &[&str] = &[
            "type",
            "guid",
            "name",
            "coords",
            "colors",
            "normals",
            "point_size",
        ];
        deserializer.deserialize_struct("PointCloud", FIELDS, PointCloudVisitor)
    }
}
