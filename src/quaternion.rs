use crate::Vector;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::ops::{Add, Mul, Neg, Sub};

#[derive(Debug, Clone)]
pub struct Quaternion {
    pub typ: String,
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub s: f64,
    pub v: Vector,
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
        state.serialize_field("s", &self.s)?;
        state.serialize_field("x", &self.v[0])?;
        state.serialize_field("y", &self.v[1])?;
        state.serialize_field("z", &self.v[2])?;
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
            s: helper.s,
            v: Vector::new(helper.x, helper.y, helper.z),
        })
    }
}

impl Quaternion {
    fn apply(&self, s: f64, v: Vector) -> Self {
        Quaternion { typ: self.typ.clone(), guid: std::sync::OnceLock::new(), name: self.name.clone(), s, v }
    }

    /// cgmath-equivalent: `Quaternion::new(w, xi, yj, zk)`
    pub fn new(w: f64, xi: f64, yj: f64, zk: f64) -> Self {
        Quaternion {
            typ: "Quaternion".to_string(),
            guid: std::sync::OnceLock::new(),
            name: "my_quaternion".to_string(),
            s: w,
            v: Vector::new(xi, yj, zk),
        }
    }

    /// cgmath-equivalent: `Quaternion::from_sv(s, v)`
    pub fn from_sv(s: f64, v: Vector) -> Self {
        Quaternion {
            typ: "Quaternion".to_string(),
            guid: std::sync::OnceLock::new(),
            name: "my_quaternion".to_string(),
            s,
            v,
        }
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    pub fn identity() -> Self {
        Self::from_sv(1.0, Vector::new(0.0, 0.0, 0.0))
    }

    pub fn from_axis_angle(axis: Vector, angle: f64) -> Self {
        let ax = axis.normalized();
        let half = angle * 0.5;
        Self::from_sv(half.cos(), ax * half.sin())
    }

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
                return Self::from_axis_angle(perp.normalized(), std::f64::consts::PI);
            }
            return Self::identity();
        }
        Self::from_sv(1.0 + dot_val, cross).normalized()
    }

    pub fn from_euler(x: f64, y: f64, z: f64) -> Self {
        let (s1, c1) = ((x * 0.5).sin(), (x * 0.5).cos());
        let (s2, c2) = ((y * 0.5).sin(), (y * 0.5).cos());
        let (s3, c3) = ((z * 0.5).sin(), (z * 0.5).cos());
        Self::from_sv(
            -s1 * s2 * s3 + c1 * c2 * c3,
            Vector::new(
                s1 * c2 * c3 + s2 * s3 * c1,
                -s1 * s3 * c2 + s2 * c1 * c3,
                s1 * s2 * c3 + s3 * c1 * c2,
            ),
        )
    }

    pub fn rotate_vector(&self, v: Vector) -> Vector {
        let uv = self.v.cross(&v);
        let uuv = self.v.cross(&uv);
        v + (uv * self.s + uuv) * 2.0
    }

    pub fn magnitude(&self) -> f64 {
        self.magnitude2().sqrt()
    }

    pub fn magnitude2(&self) -> f64 {
        self.s * self.s + self.v[0] * self.v[0] + self.v[1] * self.v[1] + self.v[2] * self.v[2]
    }

    pub fn normalized(&self) -> Self {
        let mag = self.magnitude();
        if mag > 1e-10 {
            self.apply(self.s / mag, self.v.clone() / mag)
        } else {
            Self::identity()
        }
    }

    pub fn conjugate(&self) -> Self {
        self.apply(self.s, self.v.clone() * -1.0)
    }

    pub fn invert(&self) -> Self {
        let mag2 = self.magnitude2();
        if mag2 < 1e-20 {
            return Self::identity();
        }
        self.apply(self.s / mag2, self.v.clone() * (-1.0 / mag2))
    }

    pub fn dot(&self, other: &Self) -> f64 {
        self.s * other.s + self.v.dot(&other.v)
    }

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

    pub fn nlerp(&self, other: &Self, amount: f64) -> Self {
        (self.clone() * (1.0 - amount) + other.clone() * amount).normalized()
    }

    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        Ok(serde_json::to_string_pretty(self)?)
    }

    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    pub fn to_json(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    pub fn from_json(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json_data = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json_data)
    }
}

impl Mul<Quaternion> for Quaternion {
    type Output = Quaternion;

    fn mul(self, rhs: Quaternion) -> Self::Output {
        let new_s = self.s * rhs.s - self.v.dot(&rhs.v);
        let new_v = rhs.v.clone() * self.s + self.v.clone() * rhs.s + self.v.cross(&rhs.v);
        Self::from_sv(new_s, new_v)
    }
}

impl Mul<f64> for Quaternion {
    type Output = Quaternion;

    fn mul(self, t: f64) -> Self::Output {
        Self::from_sv(self.s * t, self.v * t)
    }
}

impl Add<Quaternion> for Quaternion {
    type Output = Quaternion;

    fn add(self, rhs: Quaternion) -> Self::Output {
        Self::from_sv(self.s + rhs.s, self.v + rhs.v)
    }
}

impl Sub<Quaternion> for Quaternion {
    type Output = Quaternion;

    fn sub(self, rhs: Quaternion) -> Self::Output {
        Self::from_sv(self.s - rhs.s, self.v - rhs.v)
    }
}

impl Neg for Quaternion {
    type Output = Quaternion;

    fn neg(self) -> Self::Output {
        Self::from_sv(-self.s, self.v * -1.0)
    }
}

impl PartialEq for Quaternion {
    fn eq(&self, other: &Self) -> bool {
        self.typ == other.typ && self.name == other.name && self.s == other.s && self.v == other.v
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
