use crate::point::Point;
use crate::polyline::Polyline;
use crate::tolerance::Tolerance;
use crate::tolerance::TOLERANCE;
use crate::tolerance::{SCALE, TO_DEGREES, TO_RADIANS};
use crate::xform::Xform;
use serde::{Deserialize, Serialize};
use std::cell::Cell;
use std::fmt;
use std::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};
use std::sync::OnceLock;

/// A 3D vector with a cached magnitude
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Vector")]
pub struct Vector {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: OnceLock<String>,
    pub name: String,
    #[serde(rename = "x")]
    _x: f64,
    #[serde(rename = "y")]
    _y: f64,
    #[serde(rename = "z")]
    _z: f64,
    #[serde(skip)]
    _magnitude: Cell<f64>,
    #[serde(skip)]
    _has_magnitude: Cell<bool>,
}

impl Default for Vector {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }
}

impl Vector {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self::with_name(x, y, z, "my_vector")
    }

    /// Construct from coordinates and a name
    pub fn with_name(x: f64, y: f64, z: f64, name: &str) -> Self {
        Self {
            guid: OnceLock::new(),
            name: name.to_string(),
            _x: x,
            _y: y,
            _z: z,
            _magnitude: Cell::new(0.0),
            _has_magnitude: Cell::new(false),
        }
    }

    /// Copy (new guid, same data)
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = OnceLock::new();
        copy
    }

    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Zero vector
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// Unit vector along x
    pub fn x_axis() -> Self {
        Self::new(1.0, 0.0, 0.0)
    }

    /// Unit vector along y
    pub fn y_axis() -> Self {
        Self::new(0.0, 1.0, 0.0)
    }

    /// Unit vector along z
    pub fn z_axis() -> Self {
        Self::new(0.0, 0.0, 1.0)
    }

    /// Vector from p0 to p1
    pub fn from_points(p0: &Point, p1: &Point) -> Self {
        Self::new(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Transform in place; only rotation and scale apply, a vector has no position
    pub fn transform(&mut self, xform: &Xform) {
        let x = self._x;
        let y = self._y;
        let z = self._z;
        let m = &xform.m;
        self._x = m[0] * x + m[4] * y + m[8] * z;
        self._y = m[1] * x + m[5] * y + m[9] * z;
        self._z = m[2] * x + m[6] * y + m[10] * z;
        self._has_magnitude.set(false);
    }

    /// Transformed copy
    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut result = self.duplicate();
        result.transform(xform);
        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════

    /// Negate every component in place
    pub fn reverse(&mut self) {
        self._x = -self._x;
        self._y = -self._y;
        self._z = -self._z;
    }

    /// Magnitude scaled to stay finite for large components
    fn compute_magnitude(&self) -> f64 {
        let mut ax = self._x.abs();
        let mut ay = self._y.abs();
        let mut az = self._z.abs();
        let x_zero = ax < Tolerance::ZERO_TOLERANCE;
        let y_zero = ay < Tolerance::ZERO_TOLERANCE;
        let z_zero = az < Tolerance::ZERO_TOLERANCE;
        if x_zero && y_zero && z_zero {
            return 0.0;
        }
        if x_zero && y_zero {
            return az;
        }
        if x_zero && z_zero {
            return ay;
        }
        if y_zero && z_zero {
            return ax;
        }
        if ay >= ax && ay >= az {
            std::mem::swap(&mut ax, &mut ay);
        } else if az >= ax && az >= ay {
            std::mem::swap(&mut ax, &mut az);
        }
        if ax > f64::MIN_POSITIVE {
            ay /= ax;
            az /= ax;
            return ax * (1.0 + ay * ay + az * az).sqrt();
        }
        if ax > 0.0 && ax.is_finite() {
            return ax;
        }
        0.0
    }

    /// Cached magnitude
    pub fn magnitude(&self) -> f64 {
        if !self._has_magnitude.get() {
            self._magnitude.set(self.compute_magnitude());
            self._has_magnitude.set(true);
        }
        self._magnitude.get()
    }

    /// Squared magnitude without the square root
    pub fn magnitude_squared(&self) -> f64 {
        self._x * self._x + self._y * self._y + self._z * self._z
    }

    /// Unit length in place; false when the magnitude is zero
    pub fn normalize_self(&mut self) -> bool {
        let d = self.compute_magnitude();
        if d <= 0.0 {
            return false;
        }
        self._x /= d;
        self._y /= d;
        self._z /= d;
        self._magnitude.set(1.0);
        self._has_magnitude.set(true);
        true
    }

    /// Unit length copy
    pub fn normalized(&self) -> Self {
        let mut result = Vector::new(self._x, self._y, self._z);
        result.normalize_self();
        result
    }

    /// Dot product
    pub fn dot(&self, other: &Vector) -> f64 {
        self._x * other[0] + self._y * other[1] + self._z * other[2]
    }

    /// Cross product
    pub fn cross(&self, other: &Vector) -> Vector {
        Vector::new(
            self._y * other[2] - self._z * other[1],
            self._z * other[0] - self._x * other[2],
            self._x * other[1] - self._y * other[0],
        )
    }

    /// Angle to other, negated when the cross product points down z; zero below tolerance
    pub fn angle(
        &self,
        other: &Vector,
        sign_by_cross_product: bool,
        degrees: bool,
        tolerance: Option<f64>,
    ) -> f64 {
        let tolerance = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
        let denominator = self.magnitude() * other.magnitude();
        if denominator < tolerance {
            return 0.0;
        }
        let cos_angle = (self.dot(other) / denominator).clamp(-1.0, 1.0);
        let mut angle = cos_angle.acos();
        if sign_by_cross_product && self.cross(other)[2] < 0.0 {
            angle = -angle;
        }
        if degrees {
            angle * TO_DEGREES
        } else {
            angle
        }
    }

    /// Projection onto projection_vector: (projection, projected length, perpendicular, perpendicular length)
    pub fn projection(
        &self,
        projection_vector: &Vector,
        tolerance: Option<f64>,
    ) -> (Vector, f64, Vector, f64) {
        let tolerance = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
        let projection_vector_length = projection_vector.magnitude();
        if projection_vector_length < tolerance {
            return (
                Vector::new(0.0, 0.0, 0.0),
                0.0,
                Vector::new(0.0, 0.0, 0.0),
                0.0,
            );
        }
        let projection_vector_unit = projection_vector / projection_vector_length;
        let projected_length = self.dot(&projection_vector_unit);
        let projected = &projection_vector_unit * projected_length;
        let perpendicular = self - &projected;
        let perpendicular_length = perpendicular.magnitude();
        (
            projected,
            projected_length,
            perpendicular,
            perpendicular_length,
        )
    }

    /// 1 parallel, -1 antiparallel, 0 neither
    pub fn is_parallel_to(&self, other: &Vector) -> i32 {
        let cos_tolerance = (Tolerance::ANGLE_TOLERANCE_DEGREES * TO_RADIANS).cos();
        let denominator = self.magnitude() * other.magnitude();
        if denominator <= 0.0 {
            return 0;
        }
        let cos_angle = self.dot(other) / denominator;
        if cos_angle >= cos_tolerance {
            return 1;
        }
        if cos_angle <= -cos_tolerance {
            return -1;
        }
        0
    }

    /// True when the dot product is within tolerance of zero
    pub fn is_perpendicular_to(&self, other: &Vector) -> bool {
        self.dot(other).abs() < Tolerance::ZERO_TOLERANCE
    }

    /// Set this vector perpendicular to v; false when v is zero
    pub fn perpendicular_to(&mut self, v: &Vector) -> bool {
        let mut i = 0;
        let mut j = 1;
        let mut k = 2;
        let mut a = v[0];
        let mut b = -v[1];
        if v[1].abs() > v[0].abs() {
            if v[2].abs() > v[1].abs() {
                i = 2;
                j = 1;
                k = 0;
                a = v[2];
                b = -v[1];
            } else if v[2].abs() >= v[0].abs() {
                i = 1;
                j = 2;
                k = 0;
                a = v[1];
                b = -v[2];
            } else {
                i = 1;
                j = 0;
                k = 2;
                a = v[1];
                b = -v[0];
            }
        } else if v[2].abs() > v[0].abs() {
            i = 2;
            j = 0;
            k = 1;
            a = v[2];
            b = -v[0];
        } else if v[2].abs() > v[1].abs() {
            i = 0;
            j = 2;
            k = 1;
            a = v[0];
            b = -v[2];
        }
        let mut coords = [0.0, 0.0, 0.0];
        coords[i] = b;
        coords[j] = a;
        coords[k] = 0.0;
        self._x = coords[0];
        self._y = coords[1];
        self._z = coords[2];
        self._has_magnitude.set(false);
        a != 0.0
    }

    /// True when the magnitude is within tolerance of zero
    pub fn is_zero(&self) -> bool {
        self.compute_magnitude() < Tolerance::ZERO_TOLERANCE
    }

    /// Copy scaled along its direction so its rise along z equals vertical_height
    pub fn get_leveled_vector(&self, vertical_height: f64) -> Vector {
        let mut result = Vector::new(self._x, self._y, self._z);
        if result.normalize_self() {
            let angle_rad = result.angle(&Vector::z_axis(), false, true, None) * TO_RADIANS;
            result *= vertical_height / angle_rad.cos();
        }
        result
    }

    /// Angles to the x, y and z axes
    pub fn coordinate_direction_3angles(&self, degrees: bool) -> [f64; 3] {
        let r = (self._x * self._x + self._y * self._y + self._z * self._z).sqrt();
        if r == 0.0 {
            return [0.0, 0.0, 0.0];
        }
        let alpha = (self._x / r).acos();
        let beta = (self._y / r).acos();
        let gamma = (self._z / r).acos();
        if degrees {
            return [alpha * TO_DEGREES, beta * TO_DEGREES, gamma * TO_DEGREES];
        }
        [alpha, beta, gamma]
    }

    /// Polar angle from z and azimuth from x
    pub fn coordinate_direction_2angles(&self, degrees: bool) -> [f64; 2] {
        let r = (self._x * self._x + self._y * self._y + self._z * self._z).sqrt();
        if r == 0.0 {
            return [0.0, 0.0];
        }
        let phi = (self._z / r).acos();
        let theta = self._y.atan2(self._x);
        if degrees {
            return [phi * TO_DEGREES, theta * TO_DEGREES];
        }
        [phi, theta]
    }

    /// Angle in degrees of the xy projection from the x-axis
    pub fn angle_between_vector_xy_components(vector: &Vector) -> f64 {
        vector[1].atan2(vector[0]) * TO_DEGREES
    }

    /// Component-wise sum
    pub fn sum_of_vectors(vectors: &[Vector]) -> Vector {
        let mut sum = Vector::new(0.0, 0.0, 0.0);
        for vector in vectors {
            sum += vector;
        }
        sum
    }

    /// Component-wise average; empty input returns zero
    pub fn average(vectors: &[Vector]) -> Vector {
        if vectors.is_empty() {
            return Vector::zero();
        }
        Self::sum_of_vectors(vectors) / vectors.len() as f64
    }

    pub fn scale(&mut self, factor: f64) {
        self._x *= factor;
        self._y *= factor;
        self._z *= factor;
        self._has_magnitude.set(false);
    }

    /// Scale by SCALE
    pub fn scale_up(&mut self) {
        self.scale(SCALE);
    }

    /// Scale by 1 / SCALE
    pub fn scale_down(&mut self) {
        self.scale(1.0 / SCALE);
    }

    /// Reflection through the plane with the given unit normal
    pub fn reflect(&self, plane_normal: &Vector) -> Vector {
        let d = self.dot(plane_normal);
        Vector::new(
            self._x - 2.0 * d * plane_normal[0],
            self._y - 2.0 * d * plane_normal[1],
            self._z - 2.0 * d * plane_normal[2],
        )
    }

    /// Unit area-weighted normal of a polygon by Newell's method
    pub fn average_normal(points: &[Point]) -> Vector {
        if points.is_empty() {
            return Vector::zero();
        }
        let last = points.len() - 1;
        let dx = points[last][0] - points[0][0];
        let dy = points[last][1] - points[0][1];
        let dz = points[last][2] - points[0][2];
        let n = if dx * dx + dy * dy + dz * dz < 1e-10 {
            points.len() - 1
        } else {
            points.len()
        };
        let mut normal = Vector::new(0.0, 0.0, 0.0);
        for i in 0..n {
            let prev = (i + n - 1) % n;
            let next = (i + 1) % n;
            let ax = points[i][0] - points[prev][0];
            let ay = points[i][1] - points[prev][1];
            let az = points[i][2] - points[prev][2];
            let bx = points[next][0] - points[i][0];
            let by = points[next][1] - points[i][1];
            let bz = points[next][2] - points[i][2];
            normal[0] += ay * bz - az * by;
            normal[1] += az * bx - ax * bz;
            normal[2] += ax * by - ay * bx;
        }
        normal.normalize_self();
        normal
    }

    /// Unit area-weighted normal of a polygon by Newell's method
    pub fn average_normal_polyline(polyline: &Polyline) -> Vector {
        Self::average_normal(&polyline.get_points())
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Triangle laws
    // ═══════════════════════════════════════════════════════════════════════════

    /// Third side from two sides and the angle between them
    pub fn cosine_law(
        triangle_edge_length_a: f64,
        triangle_edge_length_b: f64,
        angle_in_between_edges: f64,
        degrees: bool,
    ) -> f64 {
        let to_radians = if degrees { TO_RADIANS } else { 1.0 };
        (triangle_edge_length_a * triangle_edge_length_a
            + triangle_edge_length_b * triangle_edge_length_b
            - 2.0
                * triangle_edge_length_a
                * triangle_edge_length_b
                * (angle_in_between_edges * to_radians).cos())
        .sqrt()
    }

    /// Angle opposite side b from side a, the angle opposite a and side b
    pub fn sine_law_angle(
        triangle_edge_length_a: f64,
        angle_in_front_of_a: f64,
        triangle_edge_length_b: f64,
        degrees: bool,
    ) -> f64 {
        let to_radians = if degrees { TO_RADIANS } else { 1.0 };
        let to_degrees = if degrees { TO_DEGREES } else { 1.0 };
        (triangle_edge_length_b * (angle_in_front_of_a * to_radians).sin() / triangle_edge_length_a)
            .asin()
            * to_degrees
    }

    /// Side b from side a and the angles opposite a and b
    pub fn sine_law_length(
        triangle_edge_length_a: f64,
        angle_in_front_of_a: f64,
        angle_in_front_of_b: f64,
        degrees: bool,
    ) -> f64 {
        let to_radians = if degrees { TO_RADIANS } else { 1.0 };
        triangle_edge_length_a * (angle_in_front_of_b * to_radians).sin()
            / (angle_in_front_of_a * to_radians).sin()
    }

    /// Angle opposite side c from the three sides
    pub fn angle_from_cosine_law(
        triangle_edge_length_a: f64,
        triangle_edge_length_b: f64,
        triangle_edge_length_c: f64,
        degrees: bool,
    ) -> f64 {
        let cos_c = (triangle_edge_length_a * triangle_edge_length_a
            + triangle_edge_length_b * triangle_edge_length_b
            - triangle_edge_length_c * triangle_edge_length_c)
            / (2.0 * triangle_edge_length_a * triangle_edge_length_b);
        let angle_rad = cos_c.acos();
        if degrees {
            return angle_rad * TO_DEGREES;
        }
        angle_rad
    }

    /// Side opposite the first angle from two angles and the side opposite the second
    pub fn side_from_sine_law(
        angle_in_front_of_result_side: f64,
        angle_in_front_of_known_side: f64,
        known_side_length: f64,
        degrees: bool,
    ) -> f64 {
        let to_radians = if degrees { TO_RADIANS } else { 1.0 };
        known_side_length * (angle_in_front_of_result_side * to_radians).sin()
            / (angle_in_front_of_known_side * to_radians).sin()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════

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
        Self::jsonload(json_string).unwrap_or_default()
    }

    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.jsondump()?)?;
        Ok(())
    }

    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        let proto = crate::proto::Vector {
            name: self.name.clone(),
            x: self._x,
            y: self._y,
            z: self._z,
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Vector::decode(data)?;
        let mut vector = Self::new(proto.x, proto.y, proto.z);
        vector.name = proto.name;
        Ok(vector)
    }

    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════

    /// "x, y, z"
    pub fn str(&self) -> String {
        let prec = Tolerance::ROUNDING;
        format!(
            "{}, {}, {}",
            TOLERANCE.format_number(self._x, prec),
            TOLERANCE.format_number(self._y, prec),
            TOLERANCE.format_number(self._z, prec)
        )
    }

    /// "Vector(name, x, y, z, magnitude)"
    pub fn repr(&self) -> String {
        let prec = Tolerance::ROUNDING;
        format!(
            "Vector({}, {}, {}, {}, {})",
            self.name,
            TOLERANCE.format_number(self._x, prec),
            TOLERANCE.format_number(self._y, prec),
            TOLERANCE.format_number(self._z, prec),
            TOLERANCE.format_number(self.magnitude(), prec)
        )
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SESSION_VIEWER
    // ═══════════════════════════════════════════════════════════════════════════

    /// GPU-ready [x, y, z] as f32, the f64 to f32 boundary for wgpu upload
    pub fn to_f32(&self) -> [f32; 3] {
        [self._x as f32, self._y as f32, self._z as f32]
    }
}

impl fmt::Display for Vector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

impl Index<usize> for Vector {
    type Output = f64;

    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self._x,
            1 => &self._y,
            2 => &self._z,
            _ => panic!("Index out of range"),
        }
    }
}

impl IndexMut<usize> for Vector {
    /// Mutable coordinate; drops the cached magnitude
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        self._has_magnitude.set(false);
        match index {
            0 => &mut self._x,
            1 => &mut self._y,
            2 => &mut self._z,
            _ => panic!("Index out of range"),
        }
    }
}

impl PartialEq for Vector {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && (self._x * 1000000.0).round() == (other._x * 1000000.0).round()
            && (self._y * 1000000.0).round() == (other._y * 1000000.0).round()
            && (self._z * 1000000.0).round() == (other._z * 1000000.0).round()
    }
}

impl MulAssign<f64> for Vector {
    fn mul_assign(&mut self, factor: f64) {
        self._x *= factor;
        self._y *= factor;
        self._z *= factor;
        self._has_magnitude.set(false);
    }
}

impl DivAssign<f64> for Vector {
    fn div_assign(&mut self, factor: f64) {
        self._x /= factor;
        self._y /= factor;
        self._z /= factor;
        self._has_magnitude.set(false);
    }
}

impl AddAssign<Vector> for Vector {
    fn add_assign(&mut self, other: Vector) {
        self._x += other[0];
        self._y += other[1];
        self._z += other[2];
        self._has_magnitude.set(false);
    }
}

impl AddAssign<&Vector> for Vector {
    fn add_assign(&mut self, other: &Vector) {
        self._x += other[0];
        self._y += other[1];
        self._z += other[2];
        self._has_magnitude.set(false);
    }
}

impl SubAssign<Vector> for Vector {
    fn sub_assign(&mut self, other: Vector) {
        self._x -= other[0];
        self._y -= other[1];
        self._z -= other[2];
        self._has_magnitude.set(false);
    }
}

impl SubAssign<&Vector> for Vector {
    fn sub_assign(&mut self, other: &Vector) {
        self._x -= other[0];
        self._y -= other[1];
        self._z -= other[2];
        self._has_magnitude.set(false);
    }
}

impl Mul<f64> for Vector {
    type Output = Vector;

    fn mul(self, factor: f64) -> Vector {
        Vector::new(self._x * factor, self._y * factor, self._z * factor)
    }
}

impl Div<f64> for Vector {
    type Output = Vector;

    fn div(self, factor: f64) -> Vector {
        Vector::new(self._x / factor, self._y / factor, self._z / factor)
    }
}

impl Add<Vector> for Vector {
    type Output = Vector;

    fn add(self, other: Vector) -> Vector {
        Vector::new(self._x + other[0], self._y + other[1], self._z + other[2])
    }
}

impl Sub<Vector> for Vector {
    type Output = Vector;

    fn sub(self, other: Vector) -> Vector {
        Vector::new(self._x - other[0], self._y - other[1], self._z - other[2])
    }
}

impl Neg for Vector {
    type Output = Vector;

    fn neg(self) -> Vector {
        Vector::new(-self._x, -self._y, -self._z)
    }
}

impl Mul<f64> for &Vector {
    type Output = Vector;

    fn mul(self, factor: f64) -> Vector {
        Vector::new(self._x * factor, self._y * factor, self._z * factor)
    }
}

impl Div<f64> for &Vector {
    type Output = Vector;

    fn div(self, factor: f64) -> Vector {
        Vector::new(self._x / factor, self._y / factor, self._z / factor)
    }
}

impl Add<Vector> for &Vector {
    type Output = Vector;

    fn add(self, other: Vector) -> Vector {
        Vector::new(self._x + other[0], self._y + other[1], self._z + other[2])
    }
}

impl Add<&Vector> for &Vector {
    type Output = Vector;

    fn add(self, other: &Vector) -> Vector {
        Vector::new(self._x + other[0], self._y + other[1], self._z + other[2])
    }
}

impl Sub<Vector> for &Vector {
    type Output = Vector;

    fn sub(self, other: Vector) -> Vector {
        Vector::new(self._x - other[0], self._y - other[1], self._z - other[2])
    }
}

impl Sub<&Vector> for &Vector {
    type Output = Vector;

    fn sub(self, other: &Vector) -> Vector {
        Vector::new(self._x - other[0], self._y - other[1], self._z - other[2])
    }
}

impl Neg for &Vector {
    type Output = Vector;

    fn neg(self) -> Vector {
        Vector::new(-self._x, -self._y, -self._z)
    }
}
