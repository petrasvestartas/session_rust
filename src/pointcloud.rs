use crate::{Color, Point, Vector, Xform};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::fmt;
use std::ops::{Add, AddAssign, Sub, SubAssign};
use uuid::Uuid;

/// A point cloud with coordinates, normals, and colors stored as flat arrays.
///
/// Internally stores data as flat arrays for efficient serialization:
/// - _coords: [x0, y0, z0, x1, y1, z1, ...]
/// - _colors: [r0, g0, b0, a0, r1, g1, b1, a1, ...]
/// - _normals: [nx0, ny0, nz0, nx1, ny1, nz1, ...]
#[derive(Debug, Clone)]
pub struct PointCloud {
    pub guid: String,
    pub name: String,
    pub point_size: f64,
    pub xform: Xform,
    _coords: Vec<f64>,
    _colors: Vec<i32>,
    _normals: Vec<f64>,
}

impl Default for PointCloud {
    fn default() -> Self {
        Self {
            guid: Uuid::new_v4().to_string(),
            name: "my_pointcloud".to_string(),
            point_size: 1.0,
            xform: Xform::identity(),
            _coords: Vec::new(),
            _colors: Vec::new(),
            _normals: Vec::new(),
        }
    }
}

impl PointCloud {
    /// Create a new PointCloud from vectors of Point, Vector, and Color.
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
            pc._colors.push(c.r as i32);
            pc._colors.push(c.g as i32);
            pc._colors.push(c.b as i32);
            pc._colors.push(c.a as i32);
        }

        pc._normals.reserve(normals.len() * 3);
        for n in &normals {
            pc._normals.push(n[0]);
            pc._normals.push(n[1]);
            pc._normals.push(n[2]);
        }

        pc
    }

    /// Create from flat arrays.
    pub fn from_coords(coords: Vec<f64>, colors: Vec<i32>, normals: Vec<f64>) -> Self {
        Self {
            guid: Uuid::new_v4().to_string(),
            name: "my_pointcloud".to_string(),
            point_size: 1.0,
            xform: Xform::identity(),
            _coords: coords,
            _colors: colors,
            _normals: normals,
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Point Access
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn point_count(&self) -> usize {
        self._coords.len() / 3
    }

    pub fn len(&self) -> usize {
        self.point_count()
    }

    pub fn is_empty(&self) -> bool {
        self._coords.is_empty()
    }

    pub fn get_point(&self, index: usize) -> Point {
        let idx = index * 3;
        Point::new(self._coords[idx], self._coords[idx + 1], self._coords[idx + 2])
    }

    pub fn set_point(&mut self, index: usize, point: &Point) {
        let idx = index * 3;
        self._coords[idx] = point[0];
        self._coords[idx + 1] = point[1];
        self._coords[idx + 2] = point[2];
    }

    pub fn add_point(&mut self, point: &Point) {
        self._coords.push(point[0]);
        self._coords.push(point[1]);
        self._coords.push(point[2]);
    }

    pub fn get_points(&self) -> Vec<Point> {
        let mut points = Vec::with_capacity(self.point_count());
        for i in 0..self.point_count() {
            let idx = i * 3;
            points.push(Point::new(self._coords[idx], self._coords[idx + 1], self._coords[idx + 2]));
        }
        points
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Color Access
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn color_count(&self) -> usize {
        self._colors.len() / 4
    }

    pub fn get_color(&self, index: usize) -> Color {
        let idx = index * 4;
        Color::new(
            self._colors[idx] as u8,
            self._colors[idx + 1] as u8,
            self._colors[idx + 2] as u8,
            self._colors[idx + 3] as u8,
        )
    }

    pub fn set_color(&mut self, index: usize, color: &Color) {
        let idx = index * 4;
        self._colors[idx] = color.r as i32;
        self._colors[idx + 1] = color.g as i32;
        self._colors[idx + 2] = color.b as i32;
        self._colors[idx + 3] = color.a as i32;
    }

    pub fn add_color(&mut self, color: &Color) {
        self._colors.push(color.r as i32);
        self._colors.push(color.g as i32);
        self._colors.push(color.b as i32);
        self._colors.push(color.a as i32);
    }

    pub fn get_colors(&self) -> Vec<Color> {
        let mut colors = Vec::with_capacity(self.color_count());
        for i in 0..self.color_count() {
            let idx = i * 4;
            colors.push(Color::new(
                self._colors[idx] as u8,
                self._colors[idx + 1] as u8,
                self._colors[idx + 2] as u8,
                self._colors[idx + 3] as u8,
            ));
        }
        colors
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Normal Access
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn normal_count(&self) -> usize {
        self._normals.len() / 3
    }

    pub fn get_normal(&self, index: usize) -> Vector {
        let idx = index * 3;
        Vector::new(self._normals[idx], self._normals[idx + 1], self._normals[idx + 2])
    }

    pub fn set_normal(&mut self, index: usize, normal: &Vector) {
        let idx = index * 3;
        self._normals[idx] = normal[0];
        self._normals[idx + 1] = normal[1];
        self._normals[idx + 2] = normal[2];
    }

    pub fn add_normal(&mut self, normal: &Vector) {
        self._normals.push(normal[0]);
        self._normals.push(normal[1]);
        self._normals.push(normal[2]);
    }

    pub fn get_normals(&self) -> Vec<Vector> {
        let mut normals = Vec::with_capacity(self.normal_count());
        for i in 0..self.normal_count() {
            let idx = i * 3;
            normals.push(Vector::new(self._normals[idx], self._normals[idx + 1], self._normals[idx + 2]));
        }
        normals
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representations
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn str(&self) -> String {
        format!("{} points", self.point_count())
    }

    pub fn repr(&self) -> String {
        format!(
            "PointCloud({}, {} points, {} colors, {} normals)",
            self.name,
            self.point_count(),
            self.color_count(),
            self.normal_count()
        )
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Duplicate and Equality
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn duplicate(&self) -> Self {
        let mut result = self.clone();
        result.guid = Uuid::new_v4().to_string();
        result
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn transform(&mut self) {
        for i in 0..self.point_count() {
            let idx = i * 3;
            let mut pt = Point::new(self._coords[idx], self._coords[idx + 1], self._coords[idx + 2]);
            self.xform.transform_point(&mut pt);
            self._coords[idx] = pt[0];
            self._coords[idx + 1] = pt[1];
            self._coords[idx + 2] = pt[2];
        }

        for i in 0..self.normal_count() {
            let idx = i * 3;
            let mut n = Vector::new(self._normals[idx], self._normals[idx + 1], self._normals[idx + 2]);
            self.xform.transform_vector(&mut n);
            self._normals[idx] = n[0];
            self._normals[idx + 1] = n[1];
            self._normals[idx + 2] = n[2];
        }

        self.xform = Xform::identity();
    }

    pub fn transformed(&self) -> Self {
        let mut result = self.clone();
        result.transform();
        result
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let mut buf = Vec::new();
        let formatter = serde_json::ser::PrettyFormatter::with_indent(b"  ");
        let mut ser = serde_json::Serializer::with_formatter(&mut buf, formatter);
        serde::Serialize::serialize(self, &mut ser)?;
        Ok(String::from_utf8(buf)?)
    }

    pub fn jsonload(json_str: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_str)?)
    }

    pub fn json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json_str = self.jsondump()?;
        std::fs::write(filepath, json_str)?;
        Ok(())
    }

    pub fn json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json_str = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json_str)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn to_protobuf(&self) -> Vec<u8> {
        use crate::proto;
        use prost::Message;

        let mut proto_xform = proto::Xform::default();
        proto_xform.guid = self.xform.guid.clone();
        proto_xform.name = self.xform.name.clone();
        proto_xform.matrix = self.xform.m.to_vec();

        let proto = proto::PointCloud {
            guid: self.guid.clone(),
            name: self.name.clone(),
            coords: self._coords.clone(),
            colors: self._colors.iter().map(|&c| c as u32).collect(),
            normals: self._normals.clone(),
            point_size: self.point_size,
            xform: Some(proto_xform),
        };

        proto.encode_to_vec()
    }

    pub fn from_protobuf(data: &[u8]) -> Self {
        use crate::proto;
        use prost::Message;

        let proto = proto::PointCloud::decode(data).expect("Failed to decode protobuf");

        let mut pc = Self::from_coords(
            proto.coords,
            proto.colors.iter().map(|&c| c as i32).collect(),
            proto.normals,
        );
        pc.guid = proto.guid;
        pc.name = proto.name;
        pc.point_size = if proto.point_size > 0.0 { proto.point_size } else { 1.0 };

        if let Some(xform_proto) = proto.xform {
            pc.xform.guid = xform_proto.guid;
            pc.xform.name = xform_proto.name;
            for (i, &val) in xform_proto.matrix.iter().enumerate() {
                if i < 16 {
                    pc.xform.m[i] = val;
                }
            }
        }

        pc
    }

    pub fn protobuf_dump(&self, filepath: &str) {
        let data = self.to_protobuf();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    pub fn protobuf_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::from_protobuf(&data)
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// PartialEq
///////////////////////////////////////////////////////////////////////////////////////////

impl PartialEq for PointCloud {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && self._coords == other._coords
            && self._colors == other._colors
            && self._normals == other._normals
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// No-copy Operators
///////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////
// Copy Operators
///////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////
// Custom Serialization - Flat arrays for efficiency
///////////////////////////////////////////////////////////////////////////////////////////

impl Serialize for PointCloud {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("PointCloud", 8)?;

        state.serialize_field("type", "PointCloud")?;
        state.serialize_field("guid", &self.guid)?;
        state.serialize_field("name", &self.name)?;
        state.serialize_field("coords", &self._coords)?;
        state.serialize_field("colors", &self._colors)?;
        state.serialize_field("normals", &self._normals)?;
        state.serialize_field("point_size", &self.point_size)?;
        state.serialize_field("xform", &self.xform)?;

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
            Xform,
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
                let mut xform = None;

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
                        Field::Xform => {
                            xform = Some(map.next_value()?);
                        }
                    }
                }

                let guid = guid.ok_or_else(|| de::Error::missing_field("guid"))?;
                let name = name.ok_or_else(|| de::Error::missing_field("name"))?;
                let coords = coords.ok_or_else(|| de::Error::missing_field("coords"))?;
                let colors = colors.ok_or_else(|| de::Error::missing_field("colors"))?;
                let normals = normals.ok_or_else(|| de::Error::missing_field("normals"))?;
                let point_size = point_size.unwrap_or(1.0);
                let xform = xform.ok_or_else(|| de::Error::missing_field("xform"))?;

                Ok(PointCloud {
                    guid,
                    name,
                    point_size,
                    xform,
                    _coords: coords,
                    _colors: colors,
                    _normals: normals,
                })
            }
        }

        const FIELDS: &[&str] = &[
            "type", "guid", "name", "coords", "colors", "normals", "point_size", "xform",
        ];
        deserializer.deserialize_struct("PointCloud", FIELDS, PointCloudVisitor)
    }
}
