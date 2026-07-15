use crate::Plane;
use crate::Point;
use crate::Vector;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::ops::{Add, Index, IndexMut, Mul, Neg, Sub};

/// A quaternion for 3D rotations (scalar + vector).
#[derive(Debug, Clone)]
pub struct Quaternion {
    /// Type identifier
    pub typ: String,
    /// Lazily generated unique identifier
    guid: std::sync::OnceLock<String>,
    /// Human-readable name
    pub name: String,
    /// Scalar part
    pub scalar: f64,
    /// Vector part
    pub vector: Vector,
}

impl Serialize for Quaternion {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("Quaternion", 6)?;
        state.serialize_field("type", &self.typ)?;
        state.serialize_field("guid", self.guid())?;
        state.serialize_field("name", &self.name)?;
        state.serialize_field("s", &self.scalar)?;
        state.serialize_field("x", &self.vector[0])?;
        state.serialize_field("y", &self.vector[1])?;
        state.serialize_field("z", &self.vector[2])?;
        state.end()
    }
}

impl<'de> Deserialize<'de> for Quaternion {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct QuaternionHelper {
            #[serde(rename = "type")]
            typ: String,
            guid: String,
            name: String,
            s: f64,
            x: f64,
            y: f64,
            z: f64,
        }

        let helper = QuaternionHelper::deserialize(deserializer)?;
        let guid = std::sync::OnceLock::new();
        let _ = guid.set(helper.guid);
        Ok(Quaternion {
            typ: helper.typ,
            guid,
            name: helper.name,
            scalar: helper.s,
            vector: Vector::new(helper.x, helper.y, helper.z),
        })
    }
}

impl Quaternion {
    /// Internal helper: create quaternion preserving typ/name but new guid
    fn apply(&self, scalar: f64, vector: Vector) -> Self {
        Quaternion { typ: self.typ.clone(), guid: std::sync::OnceLock::new(), name: self.name.clone(), scalar, vector }
    }

    /// Construct from scalar w and vector components (xi, yj, zk)
    pub fn new(w: f64, xi: f64, yj: f64, zk: f64) -> Self {
        Quaternion {
            typ: "Quaternion".to_string(),
            guid: std::sync::OnceLock::new(),
            name: "my_quaternion".to_string(),
            scalar: w,
            vector: Vector::new(xi, yj, zk),
        }
    }

    /// Create a quaternion from raw scalar (real) and vector (imaginary) components.
    ///
    /// **WARNING:** The `vector` argument is NOT a rotation axis. It is the
    /// `(i, j, k)` coefficients of the quaternion. Most users want
    /// [`Quaternion::from_axis_angle`] instead.
    ///
    /// A quaternion is canonically written as `q = s + xi + yj + zk` where
    /// `s` is the scalar (real) part and `(x, y, z)` is the vector (imaginary)
    /// part. Use this constructor only when you have raw quaternion components.
    ///
    /// # Visually constructing a plane from `(s, v)` values
    ///
    /// 1. If `v` should be the plane's **normal** (the geometric meaning users
    ///    usually expect), bypass the quaternion entirely:
    ///    ```ignore
    ///    let p = Plane::from_point_normal(Point::new(0.0,0.0,0.0), v);
    ///    ```
    ///
    /// 2. If you want the plane produced by the quaternion's rotation
    ///    (i.e. the world XY plane rotated by `q`), normalize first:
    ///    ```ignore
    ///    let p = Quaternion::from_components(s, v).normalized().get_rotation();
    ///    ```
    ///    The result's normal is the rotation of `(0,0,1)` by `q`, which
    ///    equals `v` only in the trivial case where the rotation axis is Z.
    ///
    /// 3. If you want a quaternion whose rotation produces a plane with
    ///    normal `v`, use [`Quaternion::from_arc`]:
    ///    ```ignore
    ///    let q = Quaternion::from_arc(Vector::new(0.0,0.0,1.0), v.normalized());
    ///    let p = q.get_rotation();   // p.z_axis() == v.normalized()
    ///    ```
    pub fn from_components(scalar: f64, vector: Vector) -> Self {
        Quaternion {
            typ: "Quaternion".to_string(),
            guid: std::sync::OnceLock::new(),
            name: "my_quaternion".to_string(),
            scalar,
            vector,
        }
    }

    /// Lazy GUID accessor
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set GUID
    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Identity quaternion (scalar=1, vector=0)
    pub fn identity() -> Self {
        Self::from_components(1.0, Vector::new(0.0, 0.0, 0.0))
    }

    /// Build a unit quaternion that rotates by `angle` radians around `axis`.
    /// THE everyday rotation builder. Use this whenever you can describe the
    /// rotation as "spin by N radians around this direction" - turning a wheel,
    /// opening a door, orbiting a camera. The result is always unit-length.
    pub fn from_axis_angle(axis: Vector, angle: f64) -> Self {
        let ax = axis.normalized();
        let half = angle * 0.5;
        Self::from_components(half.cos(), ax * half.sin())
    }

    /// Extract `(axis, angle in radians)` from this quaternion — the inverse of
    /// [`Quaternion::from_axis_angle`].
    ///
    /// Geometric meaning of a quaternion `(s, v)`:
    ///
    /// - `axis  = v / |v|`
    /// - `angle = 2 * acos(s / |q|)`
    ///
    /// Normalizes internally, so non-unit quaternions are handled correctly.
    ///
    /// Edge case: for the identity quaternion (or any near-identity) the
    /// axis is undefined; this function returns `(Vector(0, 0, 1), 0.0)`.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let q = Quaternion::from_components(2.0, Vector::new(1.0, 2.0, 3.0));
    /// let (axis, angle) = q.to_axis_angle();
    /// // axis = (1,2,3)/sqrt(14), angle ≈ 2.1617 rad ≈ 123.85°
    ///
    /// // Reconstruct via geometric form:
    /// let q2 = Quaternion::from_axis_angle(axis, angle);
    /// // q2 == q.normalized()
    /// ```
    pub fn to_axis_angle(&self) -> (Vector, f64) {
        let qn = self.normalized();
        let s = qn.scalar.clamp(-1.0, 1.0);
        let angle = 2.0 * s.acos();
        let sin_half = (1.0 - s * s).sqrt();
        if sin_half < 1e-12 {
            return (Vector::new(0.0, 0.0, 1.0), 0.0);
        }
        let axis = Vector::new(
            qn.vector[0] / sin_half,
            qn.vector[1] / sin_half,
            qn.vector[2] / sin_half,
        );
        (axis, angle)
    }

    /// Build the shortest rotation that maps direction `src` to direction `dst`.
    /// Use this for "look at" logic (point a camera at a target), aligning a
    /// model's forward axis with a desired direction, or snapping one face
    /// normal to another. Both arguments are normalized internally.
    pub fn from_arc(src: Vector, dst: Vector) -> Self {
        let s = src.normalized();
        let d = dst.normalized();
        let cross = s.cross(&d);
        let dot_val = s.dot(&d);
        if cross.magnitude() < 1e-10 {
            if dot_val < 0.0 {
                let perp = s.cross(&Vector::new(0.0, 0.0, 1.0));
                let perp = if perp.magnitude() < 1e-10 {
                    s.cross(&Vector::new(0.0, 1.0, 0.0))
                } else {
                    perp
                };
                return Self::from_axis_angle(perp.normalized(), crate::tolerance::PI);
            }
            return Self::identity();
        }
        Self::from_components(1.0 + dot_val, cross).normalized()
    }

    /// Build a quaternion from three Euler angles (XYZ convention).
    /// Use only at I/O boundaries: importing rotations stored as pitch/yaw/roll
    /// or accepting user input. AVOID for composition - Euler angles suffer
    /// from gimbal lock. Store/compose as quaternions, convert to Euler only
    /// to display or save.
    pub fn from_euler(x: f64, y: f64, z: f64) -> Self {
        let (s1, c1) = ((x * 0.5).sin(), (x * 0.5).cos());
        let (s2, c2) = ((y * 0.5).sin(), (y * 0.5).cos());
        let (s3, c3) = ((z * 0.5).sin(), (z * 0.5).cos());
        Self::from_components(
            -s1 * s2 * s3 + c1 * c2 * c3,
            Vector::new(
                s1 * c2 * c3 + s2 * s3 * c1,
                -s1 * s3 * c2 + s2 * c1 * c3,
                s1 * s2 * c3 + s3 * c1 * c2,
            ),
        )
    }

    /// Build the quaternion that maps the basis of `plane_a` onto the basis of
    /// `plane_b`. Use this to snap one local frame to another - aligning two
    /// CAD parts by their reference planes, transferring a frame between
    /// objects, or computing the relative rotation between two coordinate
    /// systems. (Rhino: Quaternion.Rotation(plane, plane))
    pub fn from_rotation(plane_a: &Plane, plane_b: &Plane) -> Self {
        let xa = plane_a.x_axis_ref(); let ya = plane_a.y_axis_ref(); let za = plane_a.z_axis_ref();
        let xb = plane_b.x_axis_ref(); let yb = plane_b.y_axis_ref(); let zb = plane_b.z_axis_ref();
        let mut m = [[0.0_f64; 3]; 3];
        m[0][0] = xb[0]*xa[0] + yb[0]*ya[0] + zb[0]*za[0];
        m[0][1] = xb[0]*xa[1] + yb[0]*ya[1] + zb[0]*za[1];
        m[0][2] = xb[0]*xa[2] + yb[0]*ya[2] + zb[0]*za[2];
        m[1][0] = xb[1]*xa[0] + yb[1]*ya[0] + zb[1]*za[0];
        m[1][1] = xb[1]*xa[1] + yb[1]*ya[1] + zb[1]*za[1];
        m[1][2] = xb[1]*xa[2] + yb[1]*ya[2] + zb[1]*za[2];
        m[2][0] = xb[2]*xa[0] + yb[2]*ya[0] + zb[2]*za[0];
        m[2][1] = xb[2]*xa[1] + yb[2]*ya[1] + zb[2]*za[1];
        m[2][2] = xb[2]*xa[2] + yb[2]*ya[2] + zb[2]*za[2];
        let mut is_identity = true;
        let eps = 1.490116119385e-8_f64;
        'outer: for i in 0..3 {
            for j in 0..3 {
                let d = if i == j { (m[i][i] - 1.0).abs() } else { m[i][j].abs() };
                if d > eps { is_identity = false; break 'outer; }
            }
        }
        if is_identity { return Self::from_components(1.0, Vector::new(0.0, 0.0, 0.0)); }
        let i = if m[0][0] >= m[1][1] { if m[0][0] >= m[2][2] { 0 } else { 2 } } else { if m[1][1] >= m[2][2] { 1 } else { 2 } };
        let j = (i + 1) % 3;
        let k = (i + 2) % 3;
        let s_init = 1.0 + m[i][i] - m[j][j] - m[k][k];
        if s_init <= 0.0 { return Self::from_components(1.0, Vector::new(0.0, 0.0, 0.0)); }
        let r = s_init.sqrt();
        let s = 0.5 / r;
        let mut q = [0.0_f64; 3];
        q[i] = 0.5 * r;
        q[j] = s * (m[i][j] + m[j][i]);
        q[k] = s * (m[k][i] + m[i][k]);
        Self::from_components(s * (m[k][j] - m[j][k]), Vector::new(q[0], q[1], q[2]))
    }

    /// Apply this rotation to the world XY plane and return the resulting Plane.
    /// Use this to visualize a quaternion as a frame in 3D, or to convert a
    /// stored quaternion into a Plane for frame-based APIs in the rest of the
    /// kernel. Inverse: from_rotation(xy_plane(), result).
    /// (Rhino: Quaternion.GetRotation(out plane))
    pub fn get_rotation(&self) -> Plane {
        let a = self.scalar; let b = self.vector[0]; let c = self.vector[1]; let d = self.vector[2];
        let xaxis = Vector::new(a*a + b*b - c*c - d*d, 2.0*(a*d + b*c),       2.0*(b*d - a*c));
        let yaxis = Vector::new(2.0*(b*c - a*d),       a*a - b*b + c*c - d*d, 2.0*(a*b + c*d));
        Plane::new(Point::new(0.0, 0.0, 0.0), xaxis, yaxis)
    }

    /// Deep copy this quaternion with a new GUID.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = std::sync::OnceLock::new();
        copy
    }

    /// Apply this rotation to a 3D vector and return the rotated vector.
    /// Use this when you have a quaternion orientation and need to know where
    /// a specific direction points after the rotation - the camera's forward
    /// axis, a bone's tip, the normal of a rotated face. Math: q*v_pure*q^-1.
    pub fn rotate_vector(&self, v: Vector) -> Vector {
        let uv = self.vector.cross(&v);
        let uuv = self.vector.cross(&uv);
        v + (uv * self.scalar + uuv) * 2.0
    }

    /// Euclidean norm
    pub fn magnitude(&self) -> f64 {
        self.magnitude_squared().sqrt()
    }

    /// Squared magnitude
    pub fn magnitude_squared(&self) -> f64 {
        self.scalar * self.scalar + self.vector[0] * self.vector[0] + self.vector[1] * self.vector[1] + self.vector[2] * self.vector[2]
    }

    /// Return a unit-length copy. Use periodically after composing many
    /// rotations - floating-point drift slowly makes a quaternion non-unit,
    /// and a non-unit quaternion no longer represents a valid rotation.
    pub fn normalized(&self) -> Self {
        let mag = self.magnitude();
        if mag > 1e-10 {
            self.apply(self.scalar / mag, self.vector.clone() / mag)
        } else {
            Self::identity()
        }
    }

    /// Flip the sign of the vector part: (s, v) -> (s, -v). For UNIT
    /// quaternions this equals the inverse - the opposite rotation. Use as
    /// the cheap inverse when you KNOW the quaternion is unit-length.
    pub fn conjugate(&self) -> Self {
        self.apply(self.scalar, self.vector.clone() * -1.0)
    }

    /// True multiplicative inverse: conjugate / magnitude_squared. Works for
    /// non-unit quaternions too. Use as the safe inverse when the quaternion
    /// may not be unit-length. q * q.invert() always equals identity.
    pub fn invert(&self) -> Self {
        let mag2 = self.magnitude_squared();
        if mag2 < 1e-20 {
            return Self::identity();
        }
        self.apply(self.scalar / mag2, self.vector.clone() * (-1.0 / mag2))
    }

    /// Algebraic 4D dot product (NOT a geometric operation). Used inside
    /// slerp implementations and as a similarity measure between two unit
    /// quaternions (1 = same, 0 = 90 deg apart).
    pub fn dot(&self, other: &Self) -> f64 {
        self.scalar * other.scalar + self.vector.dot(&other.vector)
    }

    /// Spherical Linear intERPolation along the shortest great-circle path
    /// on S^3. Constant angular velocity. Use for high-quality animation
    /// between two orientations - camera transitions, character bones,
    /// anything where smoothness matters more than raw speed.
    pub fn slerp(&self, other: &Self, amount: f64) -> Self {
        let dot_val = self.dot(other);
        if dot_val > 0.9995 {
            return (self.clone() + (other.clone() - self.clone()) * amount).normalized();
        }
        let robust_dot = dot_val.max(-1.0).min(1.0);
        let theta = robust_dot.acos();
        let scale1 = (theta * (1.0 - amount)).sin();
        let scale2 = (theta * amount).sin();
        let sin_theta = theta.sin();
        (self.clone() * scale1 + other.clone() * scale2) * (1.0 / sin_theta)
    }

    /// Normalized Linear intERPolation. Cheaper than slerp but the angular
    /// velocity isn't perfectly uniform. Use in real-time loops where every
    /// microsecond matters and the visual difference from slerp is negligible.
    pub fn nlerp(&self, other: &Self, amount: f64) -> Self {
        (self.clone() * (1.0 - amount) + other.clone() * amount).normalized()
    }

    /// Simple string form (like Python __str__): scalar + vector components.
    pub fn str(&self) -> String {
        use crate::tolerance::TOLERANCE;
        let prec = crate::tolerance::Tolerance::ROUNDING;
        format!(
            "{}, {}, {}, {}",
            TOLERANCE.format_number(self.scalar, prec),
            TOLERANCE.format_number(self.vector[0], prec),
            TOLERANCE.format_number(self.vector[1], prec),
            TOLERANCE.format_number(self.vector[2], prec),
        )
    }

    /// Detailed representation (like Python __repr__).
    pub fn repr(&self) -> String {
        use crate::tolerance::TOLERANCE;
        let prec = crate::tolerance::Tolerance::ROUNDING;
        format!(
            "Quaternion({}, {}, {}, {}, {})",
            self.name,
            TOLERANCE.format_number(self.scalar, prec),
            TOLERANCE.format_number(self.vector[0], prec),
            TOLERANCE.format_number(self.vector[1], prec),
            TOLERANCE.format_number(self.vector[2], prec),
        )
    }

    /// Serialize to JSON string
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from JSON string
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Convert to JSON string
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    /// Load from JSON string
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_else(|_| Self::identity())
    }

    /// Write JSON to file
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    /// Read JSON from file
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json_data = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json_data)
    }

    /// Convert to protobuf binary format
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        let proto = crate::proto::Quaternion {
            a: self.scalar as f64,
            b: self.vector[0] as f64,
            c: self.vector[1] as f64,
            d: self.vector[2] as f64,
            name: self.name.clone(),
        };
        proto.encode_to_vec()
    }

    /// Create Quaternion from protobuf binary data
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Quaternion::decode(data)?;
        let mut q = Self::from_components(proto.a as f64, Vector::new(proto.b as f64, proto.c as f64, proto.d as f64));
        q.name = proto.name;
        Ok(q)
    }

    /// Write protobuf to file
    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    /// Read protobuf from file
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // WGPU
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// GPU-ready `[x, y, z, w]` as f32 — the standard xyzw quaternion layout for GPU
    /// upload (camera / orientation rows). Mirrors [`crate::Xform::to_f32`]; kernel stays f64.
    pub fn to_f32(&self) -> [f32; 4] {
        [
            self.vector[0] as f32,
            self.vector[1] as f32,
            self.vector[2] as f32,
            self.scalar as f32,
        ]
    }
}

impl Index<usize> for Quaternion {
    type Output = f64;

    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.scalar,
            1 => &self.vector[0],
            2 => &self.vector[1],
            3 => &self.vector[2],
            _ => panic!("Index out of range"),
        }
    }
}

impl IndexMut<usize> for Quaternion {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.scalar,
            1 => &mut self.vector[0],
            2 => &mut self.vector[1],
            3 => &mut self.vector[2],
            _ => panic!("Index out of range"),
        }
    }
}

/// Quaternion multiplication (composition)
impl Mul<Quaternion> for Quaternion {
    type Output = Quaternion;

    fn mul(self, rhs: Quaternion) -> Self::Output {
        let new_s = self.scalar * rhs.scalar - self.vector.dot(&rhs.vector);
        let new_v = rhs.vector.clone() * self.scalar + self.vector.clone() * rhs.scalar + self.vector.cross(&rhs.vector);
        Self::from_components(new_s, new_v)
    }
}

/// Scalar multiplication
impl Mul<f64> for Quaternion {
    type Output = Quaternion;

    fn mul(self, t: f64) -> Self::Output {
        Self::from_components(self.scalar * t, self.vector * t)
    }
}

/// Component-wise addition
impl Add<Quaternion> for Quaternion {
    type Output = Quaternion;

    fn add(self, rhs: Quaternion) -> Self::Output {
        Self::from_components(self.scalar + rhs.scalar, self.vector + rhs.vector)
    }
}

/// Component-wise subtraction
impl Sub<Quaternion> for Quaternion {
    type Output = Quaternion;

    fn sub(self, rhs: Quaternion) -> Self::Output {
        Self::from_components(self.scalar - rhs.scalar, self.vector - rhs.vector)
    }
}

/// Negation
impl Neg for Quaternion {
    type Output = Quaternion;

    fn neg(self) -> Self::Output {
        Self::from_components(-self.scalar, self.vector * -1.0)
    }
}

impl PartialEq for Quaternion {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && (self.scalar * 1000000.0).round() == (other.scalar * 1000000.0).round()
            && (self.vector[0] * 1000000.0).round() == (other.vector[0] * 1000000.0).round()
            && (self.vector[1] * 1000000.0).round() == (other.vector[1] * 1000000.0).round()
            && (self.vector[2] * 1000000.0).round() == (other.vector[2] * 1000000.0).round()
    }
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::identity()
    }
}

#[cfg(test)]
#[path = "quaternion_test.rs"]
mod tests;
