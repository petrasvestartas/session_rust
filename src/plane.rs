use crate::{Point, Vector, Xform};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde::ser::SerializeMap;
use uuid::Uuid;

#[derive(Debug, Clone)]
pub struct Plane {
    pub guid: String,
    pub name: String,
    pub width: f64,
    _origin: Point,
    _x_axis: Vector,
    _y_axis: Vector,
    _z_axis: Vector,
    _a: f64,
    _b: f64,
    _c: f64,
    _d: f64,
    pub xform: Xform,
}

// Custom serialization to use single flat frame array of 12 numbers
// [ox, oy, oz, xx, xy, xz, yx, yy, yz, zx, zy, zz]
// Plane equation coefficients (a, b, c, d) are computed on load, not stored
impl Serialize for Plane {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut map = serializer.serialize_map(Some(6))?;
        map.serialize_entry("type", "Plane")?;
        map.serialize_entry("guid", &self.guid)?;
        map.serialize_entry("name", &self.name)?;
        // Single flat frame array of 12 numbers: origin + x_axis + y_axis + z_axis
        map.serialize_entry("frame", &[
            self._origin[0], self._origin[1], self._origin[2],
            self._x_axis[0], self._x_axis[1], self._x_axis[2],
            self._y_axis[0], self._y_axis[1], self._y_axis[2],
            self._z_axis[0], self._z_axis[1], self._z_axis[2],
        ])?;
        map.serialize_entry("width", &self.width)?;
        map.serialize_entry("xform", &self.xform)?;
        map.end()
    }
}

// Custom deserialization to parse flat frame array of 12 numbers
// Plane equation coefficients (a, b, c, d) are computed from z_axis and origin
impl<'de> Deserialize<'de> for Plane {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct PlaneData {
            guid: String,
            name: String,
            frame: [f64; 12],  // [ox, oy, oz, xx, xy, xz, yx, yy, yz, zx, zy, zz]
            #[serde(default = "default_width")]
            width: f64,
            #[serde(default)]
            xform: Option<Xform>,
        }

        fn default_width() -> f64 {
            1.0
        }

        let data = PlaneData::deserialize(deserializer)?;

        // Parse frame array
        let origin = Point::new(data.frame[0], data.frame[1], data.frame[2]);
        let x_axis = Vector::new(data.frame[3], data.frame[4], data.frame[5]);
        let y_axis = Vector::new(data.frame[6], data.frame[7], data.frame[8]);
        let z_axis = Vector::new(data.frame[9], data.frame[10], data.frame[11]);

        // Compute plane equation coefficients from z_axis (normal) and origin
        let a = z_axis[0];
        let b = z_axis[1];
        let c = z_axis[2];
        let d = -(a * origin[0] + b * origin[1] + c * origin[2]);

        Ok(Plane {
            guid: data.guid,
            name: data.name,
            width: data.width,
            _origin: origin,
            _x_axis: x_axis,
            _y_axis: y_axis,
            _z_axis: z_axis,
            _a: a,
            _b: b,
            _c: c,
            _d: d,
            xform: data.xform.unwrap_or_else(Xform::identity),
        })
    }
}

impl Default for Plane {
    fn default() -> Self {
        Self {
            guid: Uuid::new_v4().to_string(),
            name: "my_plane".to_string(),
            width: 1.0,
            _origin: Point::default(),
            _x_axis: Vector::x_axis(),
            _y_axis: Vector::y_axis(),
            _z_axis: Vector::z_axis(),
            _a: 0.0,
            _b: 0.0,
            _c: 1.0,
            _d: 0.0,
            xform: Xform::identity(),
        }
    }
}

impl Plane {
    pub fn new(point: Point, mut x_axis: Vector, mut y_axis: Vector) -> Self {
        x_axis.normalize();
        let dot_product = y_axis.dot(&x_axis);
        y_axis -= x_axis.clone() * dot_product;
        y_axis.normalize();
        let mut z_axis = x_axis.cross(&y_axis);
        z_axis.normalize();

        let a = z_axis[0];
        let b = z_axis[1];
        let c = z_axis[2];
        let d = -(a * point[0] + b * point[1] + c * point[2]);

        Self {
            guid: Uuid::new_v4().to_string(),
            name: "my_plane".to_string(),
            width: 1.0,
            _origin: point,
            _x_axis: x_axis,
            _y_axis: y_axis,
            _z_axis: z_axis,
            _a: a,
            _b: b,
            _c: c,
            _d: d,
            xform: Xform::identity(),
        }
    }

    pub fn with_name(point: Point, mut x_axis: Vector, mut y_axis: Vector, name: String) -> Self {
        x_axis.normalize();
        let dot_product = y_axis.dot(&x_axis);
        y_axis -= x_axis.clone() * dot_product;
        y_axis.normalize();
        let mut z_axis = x_axis.cross(&y_axis);
        z_axis.normalize();

        let a = z_axis[0];
        let b = z_axis[1];
        let c = z_axis[2];
        let d = -(a * point[0] + b * point[1] + c * point[2]);

        Self {
            guid: Uuid::new_v4().to_string(),
            name,
            width: 1.0,
            _origin: point,
            _x_axis: x_axis,
            _y_axis: y_axis,
            _z_axis: z_axis,
            _a: a,
            _b: b,
            _c: c,
            _d: d,
            xform: Xform::identity(),
        }
    }

    pub fn from_point_normal(point: Point, normal: Vector) -> Self {
        let origin = point.clone();
        let mut z_axis = normal;
        z_axis.normalize();
        let mut x_axis = Vector::default();
        x_axis.perpendicular_to(&z_axis);
        x_axis.normalize();
        let mut y_axis = z_axis.cross(&x_axis);
        y_axis.normalize();

        let a = z_axis[0];
        let b = z_axis[1];
        let c = z_axis[2];
        let d = -(a * origin[0] + b * origin[1] + c * origin[2]);

        Self {
            guid: Uuid::new_v4().to_string(),
            name: "my_plane".to_string(),
            width: 1.0,
            _origin: origin,
            _x_axis: x_axis,
            _y_axis: y_axis,
            _z_axis: z_axis,
            _a: a,
            _b: b,
            _c: c,
            _d: d,
            xform: Xform::identity(),
        }
    }

    pub fn from_points(points: Vec<Point>) -> Self {
        if points.len() < 3 {
            return Self::default();
        }

        let point1 = &points[0];
        let point2 = &points[1];
        let point3 = &points[2];
        let v1 = point2.clone() - point1.clone();
        let v2 = point3.clone() - point1.clone();
        let mut z_axis = v1.cross(&v2);
        z_axis.normalize();
        let mut x_axis = Vector::default();
        x_axis.perpendicular_to(&z_axis);
        x_axis.normalize();
        let mut y_axis = z_axis.cross(&x_axis);
        y_axis.normalize();
        let origin = point1.clone();

        let a = z_axis[0];
        let b = z_axis[1];
        let c = z_axis[2];
        let d = -(a * origin[0] + b * origin[1] + c * origin[2]);

        Self {
            guid: Uuid::new_v4().to_string(),
            name: "my_plane".to_string(),
            width: 1.0,
            _origin: origin,
            _x_axis: x_axis,
            _y_axis: y_axis,
            _z_axis: z_axis,
            _a: a,
            _b: b,
            _c: c,
            _d: d,
            xform: Xform::identity(),
        }
    }

    pub fn from_two_points(point1: Point, point2: Point) -> Self {
        let origin = point1.clone();

        let mut direction = point2.clone() - point1.clone();
        direction.normalize();
        let mut z_axis = Vector::default();
        z_axis.perpendicular_to(&direction);
        z_axis.normalize();

        let x_axis = direction;
        let mut y_axis = z_axis.cross(&x_axis);
        y_axis.normalize();

        let a = z_axis[0];
        let b = z_axis[1];
        let c = z_axis[2];
        let d = -(a * origin[0] + b * origin[1] + c * origin[2]);

        Self {
            guid: Uuid::new_v4().to_string(),
            name: "my_plane".to_string(),
            width: 1.0,
            _origin: origin,
            _x_axis: x_axis,
            _y_axis: y_axis,
            _z_axis: z_axis,
            _a: a,
            _b: b,
            _c: c,
            _d: d,
            xform: Xform::identity(),
        }
    }

    pub fn xy_plane() -> Self {
        Self {
            guid: Uuid::new_v4().to_string(),
            name: "xy_plane".to_string(),
            width: 1.0,
            _origin: Point::new(0.0, 0.0, 0.0),
            _x_axis: Vector::x_axis(),
            _y_axis: Vector::y_axis(),
            _z_axis: Vector::z_axis(),
            _a: 0.0,
            _b: 0.0,
            _c: 1.0,
            _d: 0.0,
            xform: Xform::identity(),
        }
    }

    pub fn yz_plane() -> Self {
        Self {
            guid: Uuid::new_v4().to_string(),
            name: "yz_plane".to_string(),
            width: 1.0,
            _origin: Point::new(0.0, 0.0, 0.0),
            _x_axis: Vector::y_axis(),
            _y_axis: Vector::z_axis(),
            _z_axis: Vector::x_axis(),
            _a: 1.0,
            _b: 0.0,
            _c: 0.0,
            _d: 0.0,
            xform: Xform::identity(),
        }
    }

    pub fn xz_plane() -> Self {
        Self {
            guid: Uuid::new_v4().to_string(),
            name: "xz_plane".to_string(),
            width: 1.0,
            _origin: Point::new(0.0, 0.0, 0.0),
            _x_axis: Vector::x_axis(),
            _y_axis: Vector::new(0.0, 0.0, -1.0),
            _z_axis: Vector::new(0.0, 1.0, 0.0),
            _a: 0.0,
            _b: 1.0,
            _c: 0.0,
            _d: 0.0,
            xform: Xform::identity(),
        }
    }

    pub fn origin(&self) -> Point {
        self._origin.clone()
    }

    /// Returns a reference to the origin point (avoids clone).
    pub fn origin_ref(&self) -> &Point {
        &self._origin
    }

    pub fn x_axis(&self) -> Vector {
        self._x_axis.clone()
    }

    /// Returns a reference to the x-axis (avoids clone).
    pub fn x_axis_ref(&self) -> &Vector {
        &self._x_axis
    }

    pub fn y_axis(&self) -> Vector {
        self._y_axis.clone()
    }

    /// Returns a reference to the y-axis (avoids clone).
    pub fn y_axis_ref(&self) -> &Vector {
        &self._y_axis
    }

    pub fn z_axis(&self) -> Vector {
        self._z_axis.clone()
    }

    /// Returns a reference to the z-axis (avoids clone).
    pub fn z_axis_ref(&self) -> &Vector {
        &self._z_axis
    }

    /// Check if the plane coordinate system is right-handed.
    ///
    /// A coordinate system is right-handed if z_axis = x_axis × y_axis.
    ///
    /// # Returns
    ///
    /// `true` if the plane is right-handed, `false` otherwise.
    pub fn is_right_hand(&self) -> bool {
        let cross = self._x_axis.cross(&self._y_axis);
        cross.dot(&self._z_axis) > 0.0
    }

    pub fn a(&self) -> f64 {
        self._a
    }

    pub fn b(&self) -> f64 {
        self._b
    }

    pub fn c(&self) -> f64 {
        self._c
    }

    pub fn d(&self) -> f64 {
        self._d
    }

    pub fn reverse(&mut self) {
        std::mem::swap(&mut self._x_axis, &mut self._y_axis);
        self._z_axis.reverse();

        self._a = self._z_axis[0];
        self._b = self._z_axis[1];
        self._c = self._z_axis[2];
        self._d =
            -(self._a * self._origin[0] + self._b * self._origin[1] + self._c * self._origin[2]);
    }

    pub fn rotate(&mut self, angles_in_radians: f64) {
        let cos_angle = angles_in_radians.cos();
        let sin_angle = angles_in_radians.sin();

        let new_x = self._x_axis.clone() * cos_angle + self._y_axis.clone() * sin_angle;
        let new_y = self._y_axis.clone() * cos_angle - self._x_axis.clone() * sin_angle;

        self._x_axis = new_x;
        self._y_axis = new_y;

    }

    pub fn is_same_direction(plane0: &Plane, plane1: &Plane, can_be_flipped: bool) -> bool {
        let n0 = plane0._z_axis.clone();
        let n1 = plane1._z_axis.clone();

        let parallel = n0.is_parallel_to(&n1);

        if can_be_flipped {
            parallel != 0
        } else {
            parallel == 1
        }
    }

    pub fn is_same_position(plane0: &Plane, plane1: &Plane) -> bool {
        let dist0 = (plane0._a * plane1._origin[0]
            + plane0._b * plane1._origin[1]
            + plane0._c * plane1._origin[2]
            + plane0._d)
            .abs();

        let dist1 = (plane1._a * plane0._origin[0]
            + plane1._b * plane0._origin[1]
            + plane1._c * plane0._origin[2]
            + plane1._d)
            .abs();

        let tolerance = crate::tolerance::Tolerance::ZERO_TOLERANCE;
        dist0 < tolerance && dist1 < tolerance
    }

    pub fn is_coplanar(plane0: &Plane, plane1: &Plane, can_be_flipped: bool) -> bool {
        Self::is_same_direction(plane0, plane1, can_be_flipped)
            && Self::is_same_position(plane0, plane1)
    }
}

impl std::ops::Index<usize> for Plane {
    type Output = Vector;

    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self._x_axis,
            1 => &self._y_axis,
            _ => &self._z_axis,
        }
    }
}

impl std::ops::IndexMut<usize> for Plane {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self._x_axis,
            1 => &mut self._y_axis,
            _ => &mut self._z_axis,
        }
    }
}

impl std::ops::AddAssign<Vector> for Plane {
    fn add_assign(&mut self, other: Vector) {
        self._origin += other;
        self._d =
            -(self._a * self._origin[0] + self._b * self._origin[1] + self._c * self._origin[2]);
    }
}

impl std::ops::SubAssign<Vector> for Plane {
    fn sub_assign(&mut self, other: Vector) {
        self._origin -= other;
        self._d =
            -(self._a * self._origin[0] + self._b * self._origin[1] + self._c * self._origin[2]);
    }
}

impl std::ops::Add<Vector> for Plane {
    type Output = Plane;

    fn add(self, other: Vector) -> Plane {
        let mut result = self.clone();
        result += other;
        result
    }
}

impl std::ops::Sub<Vector> for Plane {
    type Output = Plane;

    fn sub(self, other: Vector) -> Plane {
        let mut result = self.clone();
        result -= other;
        result
    }
}

impl PartialEq for Plane {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name &&
        self._origin == other._origin &&
        self._x_axis == other._x_axis &&
        self._y_axis == other._y_axis &&
        self._z_axis == other._z_axis
    }
}

impl Plane {
    /// Translate (move) a plane along its normal direction by a specified distance
    pub fn translate_by_normal(&self, distance: f64) -> Plane {
        let mut normal = self._z_axis.clone();
        normal.normalize();

        let new_origin = self._origin.clone() + (normal * distance);

        Plane::new(new_origin, self._x_axis.clone(), self._y_axis.clone())
    }
}

impl std::fmt::Display for Plane {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Plane(origin={}, x_axis={}, y_axis={}, z_axis={}, guid={}, name={})",
            self._origin, self._x_axis, self._y_axis, self._z_axis, self.guid, self.name
        )
    }
}

impl Plane {
    pub fn transform(&mut self) {
        // No clone needed - transform methods take &self
        self.xform.transform_point(&mut self._origin);
        self.xform.transform_vector(&mut self._x_axis);
        self.xform.transform_vector(&mut self._y_axis);
        self.xform.transform_vector(&mut self._z_axis);
        self.xform = Xform::identity();
    }

    pub fn transformed(&self) -> Self {
        let mut result = self.clone();
        result.transform();
        result
    }

    /// Create a deep copy with a new GUID.
    pub fn duplicate(&self) -> Self {
        let mut result = self.clone();
        result.guid = Uuid::new_v4().to_string();
        result
    }

    /// Minimal string representation.
    pub fn str(&self) -> String {
        use crate::tolerance::TOLERANCE;
        let prec = crate::tolerance::Tolerance::ROUNDING;
        format!(
            "{}, {}, {}",
            TOLERANCE.format_number(self._origin[0], prec),
            TOLERANCE.format_number(self._origin[1], prec),
            TOLERANCE.format_number(self._origin[2], prec),
        )
    }

    /// Full string representation.
    pub fn repr(&self) -> String {
        use crate::tolerance::TOLERANCE;
        let prec = crate::tolerance::Tolerance::ROUNDING;
        format!(
            "Plane({}, {}, {}, {}, {}, {}, {})",
            self.name,
            TOLERANCE.format_number(self._origin[0], prec),
            TOLERANCE.format_number(self._origin[1], prec),
            TOLERANCE.format_number(self._origin[2], prec),
            TOLERANCE.format_number(self._z_axis[0], prec),
            TOLERANCE.format_number(self._z_axis[1], prec),
            TOLERANCE.format_number(self._z_axis[2], prec),
        )
    }

    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        Ok(serde_json::to_string_pretty(self)?)
    }

    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    pub fn json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_else(|_| Self::default())
    }

    /// Write JSON to file.
    pub fn json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    /// Read JSON from file.
    pub fn json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json)
    }
}

// Protobuf serialization (requires "protobuf" feature)
impl Plane {
    /// Convert to protobuf binary format.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        // Use single flat frame array of 12 numbers
        let proto = crate::proto::Plane {
            guid: self.guid.clone(),
            name: self.name.clone(),
            frame: vec![
                self._origin[0], self._origin[1], self._origin[2],
                self._x_axis[0], self._x_axis[1], self._x_axis[2],
                self._y_axis[0], self._y_axis[1], self._y_axis[2],
                self._z_axis[0], self._z_axis[1], self._z_axis[2],
            ],
            width: self.width,
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid.clone(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
        };
        proto.encode_to_vec()
    }

    /// Create Plane from protobuf binary data.
    pub fn pb_loads(data: &[u8]) -> Result<Self, prost::DecodeError> {
        use prost::Message;
        let proto = crate::proto::Plane::decode(data)?;

        // Parse frame array
        let origin = Point::new(proto.frame[0], proto.frame[1], proto.frame[2]);
        let x_axis = Vector::new(proto.frame[3], proto.frame[4], proto.frame[5]);
        let y_axis = Vector::new(proto.frame[6], proto.frame[7], proto.frame[8]);
        let z_axis = Vector::new(proto.frame[9], proto.frame[10], proto.frame[11]);

        // Compute plane equation coefficients
        let a = z_axis[0];
        let b = z_axis[1];
        let c = z_axis[2];
        let d = -(a * origin[0] + b * origin[1] + c * origin[2]);

        // Load xform if present
        let xform = if let Some(proto_xform) = proto.xform {
            let mut x = Xform::identity();
            x.guid = proto_xform.guid;
            x.name = proto_xform.name;
            if proto_xform.matrix.len() == 16 {
                x.m.copy_from_slice(&proto_xform.matrix);
            }
            x
        } else {
            Xform::identity()
        };

        Ok(Plane {
            guid: proto.guid,
            name: proto.name,
            width: if proto.width > 0.0 { proto.width } else { 1.0 },
            _origin: origin,
            _x_axis: x_axis,
            _y_axis: y_axis,
            _z_axis: z_axis,
            _a: a,
            _b: b,
            _c: c,
            _d: d,
            xform,
        })
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
}

#[cfg(test)]
#[path = "plane_test.rs"]
mod plane_test;
