use crate::{Point, Vector};
use once_cell::sync::Lazy;
use parking_lot::RwLock;
use serde::Deserialize;
use serde::Serialize;
use std::f64::consts::PI as STD_PI;

pub const PI: f64 = STD_PI; // Circle constant.
pub const TWO_PI: f64 = 2.0 * STD_PI; // Full turn in radians.
pub const HALF_PI: f64 = STD_PI / 2.0; // Quarter turn in radians.
pub const TO_DEGREES: f64 = 180.0 / STD_PI; // Radian-to-degree factor.
pub const TO_RADIANS: f64 = STD_PI / 180.0; // Degree-to-radian factor.
pub const SCALE: f64 = 1e6; // Default coordinate-key scale.

/// Tolerance settings for geometric comparisons
#[derive(Debug, Clone)]
pub struct Tolerance {
    _unit: String,
    _absolute: Option<f64>,
    _relative: Option<f64>,
    _angular: Option<f64>,
    _approximation: Option<f64>,
    _precision: Option<i32>,
    _lineardeflection: Option<f64>,
    _angulardeflection: Option<f64>,
}

#[derive(Deserialize, Serialize)]
struct ToleranceData {
    absolute: f64,
    angular: f64,
    angulardeflection: f64,
    approximation: f64,
    lineardeflection: f64,
    precision: i32,
    relative: f64,
    #[serde(rename = "type")]
    kind: String,
    unit: String,
}

struct ToleranceReset<'a> {
    target: &'a mut Tolerance,
    saved: Tolerance,
}

impl Drop for ToleranceReset<'_> {
    fn drop(&mut self) {
        std::mem::swap(self.target, &mut self.saved);
    }
}

impl Tolerance {
    pub const PI: f64 = STD_PI; // Circle constant.
    pub const TWO_PI: f64 = 2.0 * STD_PI; // Full turn in radians.
    pub const HALF_PI: f64 = STD_PI / 2.0; // Quarter turn in radians.
    pub const TO_DEGREES: f64 = 180.0 / STD_PI; // Radian-to-degree factor.
    pub const TO_RADIANS: f64 = STD_PI / 180.0; // Degree-to-radian factor.

    pub const ABSOLUTE: f64 = 1e-9; // Default absolute tolerance.
    pub const RELATIVE: f64 = 1e-6; // Default relative tolerance.
    pub const ANGULAR: f64 = 1e-6; // Default angular tolerance.
    pub const APPROXIMATION: f64 = 1e-3; // Default approximation tolerance.
    pub const PRECISION: i32 = 3; // Default decimal precision.
    pub const LINEARDEFLECTION: f64 = 1e-3; // Default linear deflection.
    pub const ANGULARDEFLECTION: f64 = 1e-1; // Default angular deflection.
    pub const ANGLE_TOLERANCE_DEGREES: f64 = 0.11; // Angular tolerance in degrees.
    pub const ZERO_TOLERANCE: f64 = 1e-12; // Used heavily by algorithms; do not change.
    pub const ROUNDING: i32 = 6; // Default coordinate-key rounding.

    /// Construct tolerance with a unit system ("M" or "MM")
    pub fn new(unit: &str) -> Self {
        Self {
            _unit: unit.to_string(),
            _absolute: None,
            _relative: None,
            _angular: None,
            _approximation: None,
            _precision: None,
            _lineardeflection: None,
            _angulardeflection: None,
        }
    }

    /// Reset all overrides to default constants
    pub fn reset(&mut self) {
        self._absolute = None;
        self._relative = None;
        self._angular = None;
        self._approximation = None;
        self._precision = None;
        self._lineardeflection = None;
        self._angulardeflection = None;
    }

    /// Current unit system
    pub fn unit(&self) -> String {
        self._unit.clone()
    }

    /// Absolute tolerance value (or default ABSOLUTE)
    pub fn absolute(&self) -> f64 {
        self._absolute.unwrap_or(Self::ABSOLUTE)
    }

    /// Relative tolerance value (or default RELATIVE)
    pub fn relative(&self) -> f64 {
        self._relative.unwrap_or(Self::RELATIVE)
    }

    /// Angular tolerance value in radians (or default ANGULAR)
    pub fn angular(&self) -> f64 {
        self._angular.unwrap_or(Self::ANGULAR)
    }

    /// Approximation tolerance (or default APPROXIMATION)
    pub fn approximation(&self) -> f64 {
        self._approximation.unwrap_or(Self::APPROXIMATION)
    }

    /// Decimal precision used for formatting (or default PRECISION)
    pub fn precision(&self) -> i32 {
        self._precision.unwrap_or(Self::PRECISION)
    }

    /// Linear deflection value (or default LINEARDEFLECTION)
    pub fn lineardeflection(&self) -> f64 {
        self._lineardeflection.unwrap_or(Self::LINEARDEFLECTION)
    }

    /// Angular deflection value (or default ANGULARDEFLECTION)
    pub fn angulardeflection(&self) -> f64 {
        self._angulardeflection.unwrap_or(Self::ANGULARDEFLECTION)
    }

    /// Set current unit system
    pub fn set_unit(&mut self, value: &str) {
        if value != "M" && value != "MM" {
            panic!("Invalid unit: {}", value);
        }

        self._unit = value.to_string();
    }

    /// Override absolute tolerance
    pub fn set_absolute(&mut self, value: f64) {
        self._absolute = Some(value);
    }

    /// Override relative tolerance
    pub fn set_relative(&mut self, value: f64) {
        self._relative = Some(value);
    }

    /// Override angular tolerance (radians)
    pub fn set_angular(&mut self, value: f64) {
        self._angular = Some(value);
    }

    /// Override approximation tolerance
    pub fn set_approximation(&mut self, value: f64) {
        self._approximation = Some(value);
    }

    /// Override decimal precision for formatting
    pub fn set_precision(&mut self, value: i32) {
        if value == 0 {
            panic!("Precision cannot be zero.");
        }

        self._precision = Some(value);
    }

    /// Override linear deflection
    pub fn set_lineardeflection(&mut self, value: f64) {
        self._lineardeflection = Some(value);
    }

    /// Override angular deflection
    pub fn set_angulardeflection(&mut self, value: f64) {
        self._angulardeflection = Some(value);
    }

    /// Compute combined tolerance from relative and absolute components
    pub fn tolerance(&self, truevalue: f64, rtol: f64, atol: f64) -> f64 {
        rtol * truevalue.abs() + atol
    }

    /// Compare two values within tolerance
    pub fn compare(&self, a: f64, b: f64, rtol: f64, atol: f64) -> bool {
        (a - b).abs() <= self.tolerance(b, rtol, atol)
    }

    /// Check if value is within zero tolerance
    pub fn is_zero(&self, a: f64) -> bool {
        a.abs() <= self.absolute()
    }

    /// Check if value is positive within tolerance
    pub fn is_positive(&self, a: f64) -> bool {
        a > self.absolute()
    }

    /// Check if value is negative within tolerance
    pub fn is_negative(&self, a: f64) -> bool {
        a < -self.absolute()
    }

    /// Check if value is within a range with absolute tolerance
    pub fn is_between(&self, value: f64, minval: f64, maxval: f64) -> bool {
        let atol = self.absolute();
        minval - atol <= value && value <= maxval + atol
    }

    /// Check closeness between two values using rtol/atol
    pub fn is_close(&self, a: f64, b: f64) -> bool {
        self.compare(a, b, self.relative(), self.absolute())
    }

    /// Check if an angle is effectively zero (radians)
    pub fn is_angle_zero(&self, a: f64) -> bool {
        a.abs() <= self.angular()
    }

    /// Check if two angles are close (radians)
    pub fn is_angles_close(&self, a: f64, b: f64) -> bool {
        (a - b).abs() <= self.angular()
    }

    /// Check if two 3D points are equal within absolute tolerance
    pub fn is_point_close(&self, a: &Point, b: &Point) -> bool {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let dz = b[2] - a[2];
        dx * dx + dy * dy + dz * dz <= self.absolute() * self.absolute()
    }

    /// Check if two 3D vectors are equal within absolute tolerance
    pub fn is_vector_close(&self, a: &Vector, b: &Vector) -> bool {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let dz = b[2] - a[2];
        dx * dx + dy * dy + dz * dz <= self.absolute() * self.absolute()
    }

    /// Check if two lists of values are element-wise close
    pub fn is_allclose(&self, a: &[f64], b: &[f64]) -> bool {
        if a.len() != b.len() {
            return false;
        }

        let rtol = self.relative();
        let atol = self.absolute();

        for i in 0..a.len() {
            if !self.compare(a[i], b[i], rtol, atol) {
                return false;
            }
        }

        true
    }

    /// Run a closure on a copy-restored tolerance
    pub fn temporary<F, R>(&mut self, f: F) -> R
    where
        F: FnOnce(&mut Tolerance) -> R,
    {
        let saved = self.clone();
        let guard = ToleranceReset {
            target: self,
            saved,
        };
        f(&mut *guard.target)
    }

    /// Create a geometric key string for 3D point with optional precision
    pub fn key(&self, mut x: f64, mut y: f64, mut z: f64, precision: i32) -> String {
        let prec = if precision != -999 {
            precision
        } else {
            self.precision()
        };

        if prec == 0 {
            panic!("Precision cannot be zero.");
        }

        if prec == -1 {
            return format!("{},{},{}", x as i32, y as i32, z as i32);
        }

        if prec < -1 {
            let factor = 10.0_f64.powi(-prec - 1);

            return format!(
                "{},{},{}",
                ((x / factor).round() * factor) as i32,
                ((y / factor).round() * factor) as i32,
                ((z / factor).round() * factor) as i32
            );
        }

        let threshold = 10.0_f64.powi(-prec) * 0.5;

        if x.abs() < threshold {
            x = 0.0;
        }

        if y.abs() < threshold {
            y = 0.0;
        }

        if z.abs() < threshold {
            z = 0.0;
        }

        format!("{:.p$},{:.p$},{:.p$}", x, y, z, p = prec as usize)
    }

    /// Create a geometric key string for 2D point with optional precision
    pub fn key_xy(&self, mut x: f64, mut y: f64, precision: i32) -> String {
        let prec = if precision != -999 {
            precision
        } else {
            self.precision()
        };

        if prec == 0 {
            panic!("Precision cannot be zero.");
        }

        if prec == -1 {
            return format!("{},{}", x as i32, y as i32);
        }

        if prec < -1 {
            let factor = 10.0_f64.powi(-prec - 1);

            return format!(
                "{},{}",
                ((x / factor).round() * factor) as i32,
                ((y / factor).round() * factor) as i32
            );
        }

        let threshold = 10.0_f64.powi(-prec) * 0.5;

        if x.abs() < threshold {
            x = 0.0;
        }

        if y.abs() < threshold {
            y = 0.0;
        }

        format!("{:.p$},{:.p$}", x, y, p = prec as usize)
    }

    /// Format a number with optional precision override
    pub fn format_number(&self, number: f64, precision: i32) -> String {
        let prec = if precision != -999 {
            precision
        } else {
            self.precision()
        };

        if prec == 0 {
            panic!("Precision cannot be zero.");
        }

        if prec == -1 {
            return format!("{}", number.round() as i32);
        }

        if prec < -1 {
            let factor = 10.0_f64.powi(-prec - 1);

            return format!("{}", ((number / factor).round() * factor) as i32);
        }

        format!("{:.p$}", number, p = prec as usize)
    }

    /// Determine decimal precision from a tolerance value
    pub fn precision_from_tolerance(&self, tol: f64) -> i32 {
        let value = if tol >= 0.0 { tol } else { self.absolute() };

        if value >= 1.0 {
            return 0;
        }

        let text = format!("{:e}", value);
        let pos = match text.find("e-") {
            Some(pos) => pos,
            None => return 0,
        };
        text[pos + 2..].parse::<i32>().unwrap_or(0)
    }

    /// Serialize to sorted JSON.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(&ToleranceData {
            absolute: self.absolute(),
            angular: self.angular(),
            angulardeflection: self.angulardeflection(),
            approximation: self.approximation(),
            lineardeflection: self.lineardeflection(),
            precision: self.precision(),
            relative: self.relative(),
            kind: "Tolerance".to_string(),
            unit: self.unit(),
        })
    }

    /// Deserialize from JSON.
    pub fn jsonload(data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let data: ToleranceData = serde_json::from_str(data)?;
        let mut tolerance = Self::new(&data.unit);
        tolerance.set_absolute(data.absolute);
        tolerance.set_angular(data.angular);
        tolerance.set_angulardeflection(data.angulardeflection);
        tolerance.set_approximation(data.approximation);
        tolerance.set_lineardeflection(data.lineardeflection);
        tolerance.set_precision(data.precision);
        tolerance.set_relative(data.relative);
        Ok(tolerance)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> Result<String, Box<dyn std::error::Error>> {
        self.jsondump()
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(json_string)
    }

    /// Write JSON to a file.
    pub fn file_json_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filename, self.jsondump()?)?;
        Ok(())
    }

    /// Read JSON from a file.
    pub fn file_json_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filename)?)
    }

    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::Tolerance {
        crate::proto::Tolerance {
            unit: self.unit(),
            absolute: self.absolute(),
            relative: self.relative(),
            angular: self.angular(),
            approximation: self.approximation(),
            precision: self.precision(),
            lineardeflection: self.lineardeflection(),
            angulardeflection: self.angulardeflection(),
        }
    }

    /// Construct from the protobuf message.
    pub fn from_proto(proto: crate::proto::Tolerance) -> Self {
        let mut tolerance = Self::new(&proto.unit);
        tolerance.set_absolute(proto.absolute);
        tolerance.set_relative(proto.relative);
        tolerance.set_angular(proto.angular);
        tolerance.set_approximation(proto.approximation);
        tolerance.set_precision(proto.precision);
        tolerance.set_lineardeflection(proto.lineardeflection);
        tolerance.set_angulardeflection(proto.angulardeflection);
        tolerance
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        Ok(Self::from_proto(crate::proto::Tolerance::decode(data)?))
    }

    /// Write protobuf bytes to a file.
    pub fn pb_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filename, self.pb_dumps())?;
        Ok(())
    }

    /// Read protobuf bytes from a file.
    pub fn pb_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::pb_loads(&std::fs::read(filename)?)
    }

    /// Convert degrees to radians
    pub fn to_radians(degrees: f64) -> f64 {
        degrees * Self::TO_RADIANS
    }

    /// Convert radians to degrees
    pub fn to_degrees(radians: f64) -> f64 {
        radians * Self::TO_DEGREES
    }

    /// Round a value to a given number of decimal places
    pub fn round_to(value: f64, ndigits: i32) -> f64 {
        let factor = 10.0_f64.powi(ndigits);
        (value * factor).round() / factor
    }
}

impl Default for Tolerance {
    fn default() -> Self {
        Self::new("M")
    }
}

/// Thread-safe global Tolerance with transparent method access
pub struct GlobalTolerance {
    inner: RwLock<Tolerance>,
}

impl GlobalTolerance {
    /// Creates shared tolerance settings with default values.
    pub fn new() -> Self {
        Self {
            inner: RwLock::new(Tolerance::default()),
        }
    }

    /// Restores every shared tolerance setting to its default.
    pub fn reset(&self) {
        self.inner.write().reset();
    }

    /// Returns the current unit system.
    pub fn unit(&self) -> String {
        self.inner.read().unit()
    }

    /// Returns the active absolute tolerance.
    pub fn absolute(&self) -> f64 {
        self.inner.read().absolute()
    }

    /// Returns the active relative tolerance.
    pub fn relative(&self) -> f64 {
        self.inner.read().relative()
    }

    /// Returns the active angular tolerance.
    pub fn angular(&self) -> f64 {
        self.inner.read().angular()
    }

    /// Returns the active approximation tolerance.
    pub fn approximation(&self) -> f64 {
        self.inner.read().approximation()
    }

    /// Returns the active decimal precision.
    pub fn precision(&self) -> i32 {
        self.inner.read().precision()
    }

    /// Returns the active linear deflection.
    pub fn lineardeflection(&self) -> f64 {
        self.inner.read().lineardeflection()
    }

    /// Returns the active angular deflection.
    pub fn angulardeflection(&self) -> f64 {
        self.inner.read().angulardeflection()
    }

    /// Sets the shared unit system.
    pub fn set_unit(&self, value: &str) {
        self.inner.write().set_unit(value);
    }

    /// Sets the shared absolute tolerance.
    pub fn set_absolute(&self, value: f64) {
        self.inner.write().set_absolute(value);
    }

    /// Sets the shared relative tolerance.
    pub fn set_relative(&self, value: f64) {
        self.inner.write().set_relative(value);
    }

    /// Sets the shared angular tolerance.
    pub fn set_angular(&self, value: f64) {
        self.inner.write().set_angular(value);
    }

    /// Sets the shared approximation tolerance.
    pub fn set_approximation(&self, value: f64) {
        self.inner.write().set_approximation(value);
    }

    /// Sets the shared decimal precision.
    pub fn set_precision(&self, value: i32) {
        self.inner.write().set_precision(value);
    }

    /// Sets the shared linear deflection.
    pub fn set_lineardeflection(&self, value: f64) {
        self.inner.write().set_lineardeflection(value);
    }

    /// Sets the shared angular deflection.
    pub fn set_angulardeflection(&self, value: f64) {
        self.inner.write().set_angulardeflection(value);
    }

    /// Returns the combined relative and absolute tolerance.
    pub fn tolerance(&self, truevalue: f64, rtol: f64, atol: f64) -> f64 {
        self.inner.read().tolerance(truevalue, rtol, atol)
    }

    /// Returns whether two values differ by at most the requested tolerance.
    pub fn compare(&self, a: f64, b: f64, rtol: f64, atol: f64) -> bool {
        self.inner.read().compare(a, b, rtol, atol)
    }

    /// Returns whether a value is within the active absolute tolerance of zero.
    pub fn is_zero(&self, a: f64) -> bool {
        self.inner.read().is_zero(a)
    }

    /// Returns whether a value is positive beyond the active absolute tolerance.
    pub fn is_positive(&self, a: f64) -> bool {
        self.inner.read().is_positive(a)
    }

    /// Returns whether a value is negative beyond the active absolute tolerance.
    pub fn is_negative(&self, a: f64) -> bool {
        self.inner.read().is_negative(a)
    }

    /// Returns whether a value lies in a range within the active absolute tolerance.
    pub fn is_between(&self, value: f64, minval: f64, maxval: f64) -> bool {
        self.inner.read().is_between(value, minval, maxval)
    }

    /// Returns whether two values are close under the active tolerances.
    pub fn is_close(&self, a: f64, b: f64) -> bool {
        self.inner.read().is_close(a, b)
    }

    /// Returns whether an angle is within the active angular tolerance of zero.
    pub fn is_angle_zero(&self, a: f64) -> bool {
        self.inner.read().is_angle_zero(a)
    }

    /// Returns whether two angles are close under the active angular tolerance.
    pub fn is_angles_close(&self, a: f64, b: f64) -> bool {
        self.inner.read().is_angles_close(a, b)
    }

    /// Returns whether two points are close under the active absolute tolerance.
    pub fn is_point_close(&self, a: &Point, b: &Point) -> bool {
        self.inner.read().is_point_close(a, b)
    }

    /// Returns whether two vectors are close under the active absolute tolerance.
    pub fn is_vector_close(&self, a: &Vector, b: &Vector) -> bool {
        self.inner.read().is_vector_close(a, b)
    }

    /// Returns whether corresponding values are close under the active tolerances.
    pub fn is_allclose(&self, a: &[f64], b: &[f64]) -> bool {
        self.inner.read().is_allclose(a, b)
    }

    /// Runs a callback with mutable shared settings and restores them afterward.
    pub fn temporary<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut Tolerance) -> R,
    {
        self.inner.write().temporary(f)
    }

    /// Returns a tolerance-rounded key for three coordinates.
    pub fn key(&self, x: f64, y: f64, z: f64, precision: i32) -> String {
        self.inner.read().key(x, y, z, precision)
    }

    /// Returns a tolerance-rounded key for two coordinates.
    pub fn key_xy(&self, x: f64, y: f64, precision: i32) -> String {
        self.inner.read().key_xy(x, y, precision)
    }

    /// Formats a number using the requested or active precision.
    pub fn format_number(&self, number: f64, precision: i32) -> String {
        self.inner.read().format_number(number, precision)
    }

    /// Returns the decimal precision represented by a tolerance.
    pub fn precision_from_tolerance(&self, tol: f64) -> i32 {
        self.inner.read().precision_from_tolerance(tol)
    }
}

impl Default for GlobalTolerance {
    fn default() -> Self {
        Self::new()
    }
}

/// Global tolerance instance
pub static TOLERANCE: Lazy<GlobalTolerance> = Lazy::new(GlobalTolerance::new);

// ═══════════════════════════════════════════════════════════════════════════
// Utilities
// ═══════════════════════════════════════════════════════════════════════════

/// Check if a number is finite
pub fn is_finite(x: f64) -> bool {
    x.is_finite()
}

/// Order-independent key from two ints: larger in the high 32 bits
pub fn unique_from_two_int(a: i32, b: i32) -> u64 {
    let lo = a.min(b) as u64;
    let hi = a.max(b) as u64;
    (hi << 32) | lo
}

/// Signed modulo into [0, n-1]; 0 when n == 0
pub fn wrap_index(index: i32, n: i32) -> i32 {
    if n == 0 {
        return 0;
    }

    ((index % n) + n) % n
}

/// Opposite side of a right triangle: edge_length * tan(angle_deg)
pub fn triangle_edge_by_angle(edge_length: f64, angle_deg: f64) -> f64 {
    edge_length * Tolerance::to_radians(angle_deg).tan()
}

/// Convert radians to degrees
pub fn rad_to_deg(radians: f64) -> f64 {
    radians * Tolerance::TO_DEGREES
}

/// Convert degrees to radians
pub fn deg_to_rad(degrees: f64) -> f64 {
    degrees * Tolerance::TO_RADIANS
}

/// Number of decimal digits of the integer part of |n|; 0 when |n| < 1
pub fn count_digits(n: f64) -> i32 {
    let value = n.abs();

    if value < 1.0 {
        return 0;
    }

    value.log10() as i32 + 1
}
