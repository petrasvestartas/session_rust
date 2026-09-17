use crate::tolerance::Tolerance;
use crate::tolerance::PI;
use crate::tolerance::TOLERANCE;
use crate::{Plane, Point, Vector};
use serde::ser::SerializeStruct;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::fmt;
use std::ops::{Add, Index, IndexMut, Mul, Neg, Sub};
use std::sync::OnceLock;

/// A rotation as scalar plus vector part: q = s + xi + yj + zk.
#[derive(Debug, Clone)]
pub struct Quaternion {
    guid: OnceLock<String>, // Lazy guid.
    pub name: String,       // Quaternion name.
    pub scalar: f64,        // Scalar part s.
    pub vector: Vector,     // Vector part (x, y, z).
}

impl Default for Quaternion {
    /// Constructs the identity rotation.
    fn default() -> Self {
        Self::identity()
    }
}

impl Quaternion {
    /// Constructs from raw components; vector is (i, j, k), not a rotation axis.
    pub fn new(scalar: f64, vector: Vector) -> Self {
        Self {
            guid: OnceLock::new(),
            name: "my_quaternion".to_string(),
            scalar,
            vector,
        }
    }

    /// Copies with a new guid and the same data.
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

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Constructs the rotation that does nothing: scalar 1, vector 0.
    pub fn identity() -> Self {
        Self::new(1.0, Vector::new(0.0, 0.0, 0.0))
    }

    /// Constructs from raw components; vector is (i, j, k), not a rotation axis.
    pub fn from_components(scalar: f64, vector: Vector) -> Self {
        Self::new(scalar, vector)
    }

    /// Constructs the unit quaternion rotating by angle radians around axis.
    pub fn from_axis_angle(axis: Vector, angle: f64) -> Self {
        if axis.magnitude() < 1e-10 {
            return Self::identity();
        }

        let ax = axis.normalized();
        let half = angle * 0.5;

        Self::new(half.cos(), ax * half.sin())
    }

    /// Constructs the shortest rotation taking direction src to direction dst.
    pub fn from_arc(src: Vector, dst: Vector) -> Self {
        let s = src.normalized();
        let d = dst.normalized();
        let cross = s.cross(&d);
        let dot_val = s.dot(&d);

        if cross.magnitude() < 1e-10 {
            if dot_val < 0.0 {
                let mut perp = s.cross(&Vector::new(0.0, 0.0, 1.0));

                if perp.magnitude() < 1e-10 {
                    perp = s.cross(&Vector::new(0.0, 1.0, 0.0));
                }

                return Self::from_axis_angle(perp.normalized(), PI);
            }

            return Self::identity();
        }

        Self::new(1.0 + dot_val, cross).normalized()
    }

    /// Constructs the rotation from Euler angles in XYZ convention.
    pub fn from_euler(x: f64, y: f64, z: f64) -> Self {
        let s1 = (x * 0.5).sin();
        let c1 = (x * 0.5).cos();
        let s2 = (y * 0.5).sin();
        let c2 = (y * 0.5).cos();
        let s3 = (z * 0.5).sin();
        let c3 = (z * 0.5).cos();
        Self::new(
            -s1 * s2 * s3 + c1 * c2 * c3,
            Vector::new(
                s1 * c2 * c3 + s2 * s3 * c1,
                -s1 * s3 * c2 + s2 * c1 * c3,
                s1 * s2 * c3 + s3 * c1 * c2,
            ),
        )
    }

    /// Constructs the rotation mapping the frame of plane_a onto the frame of plane_b.
    pub fn from_rotation(plane_a: &Plane, plane_b: &Plane) -> Self {
        let xa = plane_a.x_axis();
        let ya = plane_a.y_axis();
        let za = plane_a.z_axis();
        let xb = plane_b.x_axis();
        let yb = plane_b.y_axis();
        let zb = plane_b.z_axis();
        let mut m = [[0.0_f64; 3]; 3];

        for (i, row) in m.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = xb[i] * xa[j] + yb[i] * ya[j] + zb[i] * za[j];
            }
        }

        let eps = 1.490116119385e-8;
        let mut is_identity = true;

        for (i, row) in m.iter().enumerate() {
            for (j, &value) in row.iter().enumerate() {
                if (value - if i == j { 1.0 } else { 0.0 }).abs() > eps {
                    is_identity = false;
                }
            }
        }

        if is_identity {
            return Self::identity();
        }

        let mut i = 2;

        if m[0][0] >= m[1][1] && m[0][0] >= m[2][2] {
            i = 0;
        } else if m[1][1] >= m[0][0] && m[1][1] >= m[2][2] {
            i = 1;
        }

        let j = (i + 1) % 3;
        let k = (i + 2) % 3;
        let mut s = 1.0 + m[i][i] - m[j][j] - m[k][k];

        if s <= 0.0 {
            return Self::identity();
        }

        let r = s.sqrt();
        s = 0.5 / r;
        let mut q = [0.0_f64; 3];
        q[i] = 0.5 * r;
        q[j] = s * (m[i][j] + m[j][i]);
        q[k] = s * (m[k][i] + m[i][k]);

        Self::new(s * (m[k][j] - m[j][k]), Vector::new(q[0], q[1], q[2]))
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════

    /// Returns the unit axis and angle in radians; (0, 0, 1) and 0 near identity.
    pub fn to_axis_angle(&self) -> (Vector, f64) {
        let qn = self.normalized();
        let s = qn.scalar.clamp(-1.0, 1.0);
        let angle = 2.0 * s.acos();
        let sin_half = (1.0 - s * s).sqrt();

        if sin_half < 1e-12 {
            return (Vector::new(0.0, 0.0, 1.0), 0.0);
        }

        (&qn.vector / sin_half, angle)
    }

    /// Returns a rotated copy of vec: q * v * q^-1.
    pub fn rotate_vector(&self, vec: Vector) -> Vector {
        let uv = self.vector.cross(&vec);
        let uuv = self.vector.cross(&uv);

        vec + (uv * self.scalar + uuv) * 2.0
    }

    /// Returns the world XY plane rotated by this quaternion.
    pub fn get_rotation(&self) -> Plane {
        let a = self.scalar;
        let b = self.vector[0];
        let c = self.vector[1];
        let d = self.vector[2];
        let xaxis = Vector::new(
            a * a + b * b - c * c - d * d,
            2.0 * (a * d + b * c),
            2.0 * (b * d - a * c),
        );
        let yaxis = Vector::new(
            2.0 * (b * c - a * d),
            a * a - b * b + c * c - d * d,
            2.0 * (a * b + c * d),
        );
        let zaxis = Vector::new(
            2.0 * (a * c + b * d),
            2.0 * (c * d - a * b),
            a * a - b * b - c * c + d * d,
        );

        Plane::from_frame(Point::new(0.0, 0.0, 0.0), xaxis, yaxis, zaxis)
    }

    /// Returns the 4D length.
    pub fn magnitude(&self) -> f64 {
        self.magnitude_squared().sqrt()
    }

    /// Returns the squared magnitude without the square root.
    pub fn magnitude_squared(&self) -> f64 {
        self.scalar * self.scalar + self.vector.dot(&self.vector)
    }

    /// Returns a unit length copy; identity when the magnitude is zero.
    pub fn normalized(&self) -> Self {
        let mag = self.magnitude();

        if mag < 1e-10 {
            return Self::identity();
        }

        let mut q = Self::new(self.scalar / mag, &self.vector / mag);
        q.name = self.name.clone();

        q
    }

    /// Returns (s, -v); the inverse of a unit quaternion.
    pub fn conjugate(&self) -> Self {
        let mut q = Self::new(self.scalar, -&self.vector);
        q.name = self.name.clone();

        q
    }

    /// Returns the multiplicative inverse: conjugate over squared magnitude.
    pub fn invert(&self) -> Self {
        let mag2 = self.magnitude_squared();

        if mag2 < 1e-20 {
            return Self::identity();
        }

        let mut q = Self::new(self.scalar / mag2, &self.vector * (-1.0 / mag2));
        q.name = self.name.clone();

        q
    }

    /// Returns the 4D dot product.
    pub fn dot(&self, other: &Self) -> f64 {
        self.scalar * other.scalar + self.vector.dot(&other.vector)
    }

    /// Returns the spherical interpolation at constant angular velocity.
    pub fn slerp(&self, other: &Self, amount: f64) -> Self {
        let mut target = other.clone();
        let mut dot_val = self.dot(&target);

        if dot_val < 0.0 {
            target = -target;
            dot_val = -dot_val;
        }

        if dot_val > 0.9995 {
            return (self.clone() + (target - self.clone()) * amount).normalized();
        }

        let theta = dot_val.clamp(-1.0, 1.0).acos();
        let scale1 = (theta * (1.0 - amount)).sin();
        let scale2 = (theta * amount).sin();

        (self.clone() * scale1 + target * scale2) * (1.0 / theta.sin())
    }

    /// Returns the normalized linear interpolation, cheaper than slerp.
    pub fn nlerp(&self, other: &Self, amount: f64) -> Self {
        (self.clone() * (1.0 - amount) + other.clone() * amount).normalized()
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
        let proto = crate::proto::Quaternion {
            a: self.scalar,
            b: self.vector[0],
            c: self.vector[1],
            d: self.vector[2],
            name: self.name.clone(),
        };

        proto.encode_to_vec()
    }

    /// Deserializes from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Quaternion::decode(data)?;
        let mut q = Self::new(proto.a, Vector::new(proto.b, proto.c, proto.d));
        q.name = proto.name;

        Ok(q)
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

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════

    /// Returns "s, x, y, z".
    pub fn str(&self) -> String {
        let prec = Tolerance::ROUNDING;
        format!(
            "{}, {}, {}, {}",
            TOLERANCE.format_number(self.scalar, prec),
            TOLERANCE.format_number(self.vector[0], prec),
            TOLERANCE.format_number(self.vector[1], prec),
            TOLERANCE.format_number(self.vector[2], prec)
        )
    }

    /// Returns "Quaternion(name, s, x, y, z)".
    pub fn repr(&self) -> String {
        let prec = Tolerance::ROUNDING;
        format!(
            "Quaternion({}, {}, {}, {}, {})",
            self.name,
            TOLERANCE.format_number(self.scalar, prec),
            TOLERANCE.format_number(self.vector[0], prec),
            TOLERANCE.format_number(self.vector[1], prec),
            TOLERANCE.format_number(self.vector[2], prec)
        )
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SESSION_VIEWER
    // ═══════════════════════════════════════════════════════════════════════════

    /// Returns GPU-ready [x, y, z, w] as f32, the f64 to f32 boundary for wgpu upload.
    pub fn to_f32(&self) -> [f32; 4] {
        [
            self.vector[0] as f32,
            self.vector[1] as f32,
            self.vector[2] as f32,
            self.scalar as f32,
        ]
    }
}

impl fmt::Display for Quaternion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

impl Index<usize> for Quaternion {
    type Output = f64;

    /// Returns the component by index (0=scalar, 1=x, 2=y, 3=z).
    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.scalar,
            1..=3 => &self.vector[index - 1],
            _ => panic!("Index out of range"),
        }
    }
}

impl IndexMut<usize> for Quaternion {
    /// Returns the mutable component by index (0=scalar, 1=x, 2=y, 3=z).
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.scalar,
            1..=3 => &mut self.vector[index - 1],
            _ => panic!("Index out of range"),
        }
    }
}

impl PartialEq for Quaternion {
    /// Compares name and components to six decimals; guid ignored.
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && (self.scalar * 1000000.0).round() == (other.scalar * 1000000.0).round()
            && (self.vector[0] * 1000000.0).round() == (other.vector[0] * 1000000.0).round()
            && (self.vector[1] * 1000000.0).round() == (other.vector[1] * 1000000.0).round()
            && (self.vector[2] * 1000000.0).round() == (other.vector[2] * 1000000.0).round()
    }
}

impl Mul<Quaternion> for Quaternion {
    type Output = Quaternion;

    /// Returns the composition: (a * b) applies b first, then a.
    fn mul(self, other: Quaternion) -> Quaternion {
        Quaternion::new(
            self.scalar * other.scalar - self.vector.dot(&other.vector),
            &other.vector * self.scalar
                + &self.vector * other.scalar
                + self.vector.cross(&other.vector),
        )
    }
}

impl Mul<f64> for Quaternion {
    type Output = Quaternion;

    /// Returns a copy scaled by amount.
    fn mul(self, amount: f64) -> Quaternion {
        Quaternion::new(self.scalar * amount, self.vector * amount)
    }
}

impl Add<Quaternion> for Quaternion {
    type Output = Quaternion;

    /// Returns the component-wise sum.
    fn add(self, other: Quaternion) -> Quaternion {
        Quaternion::new(self.scalar + other.scalar, self.vector + other.vector)
    }
}

impl Sub<Quaternion> for Quaternion {
    type Output = Quaternion;

    /// Returns the component-wise difference.
    fn sub(self, other: Quaternion) -> Quaternion {
        Quaternion::new(self.scalar - other.scalar, self.vector - other.vector)
    }
}

impl Neg for Quaternion {
    type Output = Quaternion;

    /// Returns the negated copy.
    fn neg(self) -> Quaternion {
        Quaternion::new(-self.scalar, -self.vector)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Serde
// ═══════════════════════════════════════════════════════════════════════════

impl Serialize for Quaternion {
    /// Serializes flat fields s, x, y, z beside guid, name and type.
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut state = serializer.serialize_struct("Quaternion", 7)?;
        state.serialize_field("guid", self.guid())?;
        state.serialize_field("name", &self.name)?;
        state.serialize_field("s", &self.scalar)?;
        state.serialize_field("type", "Quaternion")?;
        state.serialize_field("x", &self.vector[0])?;
        state.serialize_field("y", &self.vector[1])?;
        state.serialize_field("z", &self.vector[2])?;

        state.end()
    }
}

/// Flat JSON fields for deserialization.
#[derive(Deserialize)]
struct QuaternionFields {
    guid: String,
    name: String,
    s: f64,
    x: f64,
    y: f64,
    z: f64,
}

impl<'de> Deserialize<'de> for Quaternion {
    /// Deserializes from flat fields s, x, y, z, guid and name.
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let fields = QuaternionFields::deserialize(deserializer)?;
        let mut q = Quaternion::new(fields.s, Vector::new(fields.x, fields.y, fields.z));
        q.set_guid(fields.guid);
        q.name = fields.name;

        Ok(q)
    }
}
