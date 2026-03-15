use crate::point::Point;
use crate::vector::Vector;
use once_cell::sync::Lazy;
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI as STD_PI;

/// Mathematical constants
pub const PI: f64 = STD_PI;
pub const TWO_PI:  f64 = 2.0 * STD_PI;
pub const HALF_PI: f64 = STD_PI / 2.0;
pub const TO_DEGREES: f64 = 180.0 / STD_PI;
pub const TO_RADIANS: f64 = STD_PI / 180.0;

/// Scale factor
pub const SCALE: f64 = 1e6;

/// Tolerance settings for geometric operations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Tolerance {
    pub unit: String,
    absolute: Option<f64>,
    relative: Option<f64>,
    angular: Option<f64>,
    approximation: Option<f64>,
    precision: Option<i32>,
    lineardeflection: Option<f64>,
    angulardeflection: Option<f64>,
}

impl Tolerance {
    /// Mathematical constants
    pub const PI: f64 = STD_PI;
    pub const TWO_PI:  f64 = 2.0 * STD_PI;
    pub const HALF_PI: f64 = STD_PI / 2.0;
    pub const TO_DEGREES: f64 = 180.0 / STD_PI;
    pub const TO_RADIANS: f64 = STD_PI / 180.0;

    /// Default tolerance values (f64 only)
    pub const ABSOLUTE: f64 = 1e-9;
    pub const RELATIVE: f64 = 1e-6;
    pub const ANGULAR: f64 = 1e-6;
    pub const APPROXIMATION: f64 = 1e-3;
    pub const PRECISION: i32 = 3;
    pub const LINEARDEFLECTION: f64 = 1e-3;
    pub const ANGULARDEFLECTION: f64 = 1e-1;
    pub const ANGLE_TOLERANCE_DEGREES: f64 = 0.11;
    pub const ROUNDING: i32 = 6;
    pub const ZERO_TOLERANCE: f64 = 1e-12;

    pub fn new(unit: &str) -> Self {
        Self {
            unit: unit.to_string(),
            absolute: None,
            relative: None,
            angular: None,
            approximation: None,
            precision: None,
            lineardeflection: None,
            angulardeflection: None,
        }
    }

    pub fn reset(&mut self) {
        self.absolute = None;
        self.relative = None;
        self.angular = None;
        self.approximation = None;
        self.precision = None;
        self.lineardeflection = None;
        self.angulardeflection = None;
    }

    pub fn absolute(&self) -> f64 {
        self.absolute.unwrap_or(Self::ABSOLUTE)
    }

    pub fn set_absolute(&mut self, value: f64) {
        self.absolute = Some(value);
    }

    pub fn relative(&self) -> f64 {
        self.relative.unwrap_or(Self::RELATIVE)
    }

    pub fn set_relative(&mut self, value: f64) {
        self.relative = Some(value);
    }

    pub fn angular(&self) -> f64 {
        self.angular.unwrap_or(Self::ANGULAR)
    }

    pub fn set_angular(&mut self, value: f64) {
        self.angular = Some(value);
    }

    pub fn approximation(&self) -> f64 {
        self.approximation.unwrap_or(Self::APPROXIMATION)
    }

    pub fn set_approximation(&mut self, value: f64) {
        self.approximation = Some(value);
    }

    pub fn precision(&self) -> i32 {
        self.precision.unwrap_or(Self::PRECISION)
    }

    pub fn set_precision(&mut self, value: i32) {
        self.precision = Some(value);
    }

    pub fn lineardeflection(&self) -> f64 {
        self.lineardeflection.unwrap_or(Self::LINEARDEFLECTION)
    }

    pub fn set_lineardeflection(&mut self, value: f64) {
        self.lineardeflection = Some(value);
    }

    pub fn angulardeflection(&self) -> f64 {
        self.angulardeflection.unwrap_or(Self::ANGULARDEFLECTION)
    }

    pub fn set_angulardeflection(&mut self, value: f64) {
        self.angulardeflection = Some(value);
    }

    pub fn tolerance(&self, truevalue: f64, rtol: f64, atol: f64) -> f64 {
        rtol * truevalue.abs() + atol
    }

    pub fn compare(&self, a: f64, b: f64, rtol: f64, atol: f64) -> bool {
        (a - b).abs() <= self.tolerance(b, rtol, atol)
    }

    pub fn is_zero(&self, a: f64) -> bool {
        a.abs() <= self.absolute()
    }

    pub fn is_positive(&self, a: f64) -> bool {
        a > self.absolute()
    }

    pub fn is_negative(&self, a: f64) -> bool {
        a < -self.absolute()
    }

    pub fn is_between(&self, value: f64, minval: f64, maxval: f64) -> bool {
        let atol = self.absolute();
        minval - atol <= value && value <= maxval + atol
    }

    pub fn is_close(&self, a: f64, b: f64) -> bool {
        self.compare(a, b, self.relative(), self.absolute())
    }

    pub fn is_allclose(&self, a: &[f64], b: &[f64]) -> bool {
        let rtol = self.relative();
        let atol = self.absolute();
        a.iter()
            .zip(b.iter())
            .all(|(x, y)| self.compare(*x, *y, rtol, atol))
    }

    pub fn is_angle_zero(&self, a: f64) -> bool {
        a.abs() <= self.angular()
    }

    pub fn is_angles_close(&self, a: f64, b: f64) -> bool {
        (a - b).abs() <= self.angular()
    }

    pub fn is_point_close(&self, a: &Point, b: &Point) -> bool {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let dz = b[2] - a[2];
        (dx * dx + dy * dy + dz * dz) <= self.absolute() * self.absolute()
    }

    pub fn is_vector_close(&self, a: &Vector, b: &Vector) -> bool {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let dz = b[2] - a[2];
        (dx * dx + dy * dy + dz * dz) <= self.absolute() * self.absolute()
    }

    pub fn key(&self, xyz: [f64; 3], precision: i32) -> String {
        let precision = if precision == -999 { self.precision() } else { precision };
        let [mut x, mut y, mut z] = xyz;

        if precision == -1 {
            return format!("{},{},{}", x as i64, y as i64, z as i64);
        }

        if precision < -1 {
            let p = (-precision - 1) as u32;
            let factor = 10_f64.powi(p as i32);
            return format!(
                "{},{},{}",
                ((x / factor).round() * factor) as i64,
                ((y / factor).round() * factor) as i64,
                ((z / factor).round() * factor) as i64
            );
        }

        let minzero = format!("-{:.prec$}", 0.0, prec = precision as usize);
        if format!("{:.prec$}", x, prec = precision as usize) == minzero {
            x = 0.0;
        }
        if format!("{:.prec$}", y, prec = precision as usize) == minzero {
            y = 0.0;
        }
        if format!("{:.prec$}", z, prec = precision as usize) == minzero {
            z = 0.0;
        }

        format!(
            "{:.prec$},{:.prec$},{:.prec$}",
            x,
            y,
            z,
            prec = precision as usize
        )
    }

    pub fn key_xy(&self, xy: [f64; 2], precision: i32) -> String {
        let precision = if precision == -999 { self.precision() } else { precision };
        let [mut x, mut y] = xy;

        if precision == -1 {
            return format!("{},{}", x as i64, y as i64);
        }

        if precision < -1 {
            let p = (-precision - 1) as u32;
            let factor = 10_f64.powi(p as i32);
            return format!(
                "{},{}",
                ((x / factor).round() * factor) as i64,
                ((y / factor).round() * factor) as i64
            );
        }

        let minzero = format!("-{:.prec$}", 0.0, prec = precision as usize);
        if format!("{:.prec$}", x, prec = precision as usize) == minzero {
            x = 0.0;
        }
        if format!("{:.prec$}", y, prec = precision as usize) == minzero {
            y = 0.0;
        }

        format!("{:.prec$},{:.prec$}", x, y, prec = precision as usize)
    }

    pub fn format_number(&self, number: f64, precision: i32) -> String {
        let precision = if precision == -999 { self.precision() } else { precision };

        if precision == -1 {
            return format!("{}", number.round() as i64);
        }

        if precision < -1 {
            let p = (-precision - 1) as u32;
            let factor = 10_f64.powi(p as i32);
            return format!("{}", ((number / factor).round() * factor) as i64);
        }

        format!("{:.prec$}", number, prec = precision as usize)
    }

    pub fn precision_from_tolerance(&self, tol: Option<f64>) -> i32 {
        let tol = tol.unwrap_or_else(|| self.absolute());
        if tol < 1.0 {
            let s = format!("{tol:e}");
            if let Some(exp_pos) = s.find("e-") {
                if let Ok(exp) = s[exp_pos + 2..].parse::<i32>() {
                    return exp;
                }
            }
        }
        0
    }

    /// Temporarily modify tolerance, restoring original state after closure returns
    pub fn temporary<F, R>(&mut self, f: F) -> R
    where
        F: FnOnce(&mut Tolerance) -> R,
    {
        let saved = self.clone();
        let result = f(self);
        *self = saved;
        result
    }

    /// Round a value to a given number of decimal places (like Python's round(value, ndigits))
    pub fn round_to(value: f64, ndigits: i32) -> f64 {
        let factor = 10f64.powi(ndigits);
        (value * factor).round() / factor
    }
}

impl Default for Tolerance {
    fn default() -> Self {
        Self::new("M")
    }
}

/// Thread-safe wrapper for global Tolerance with transparent method access
pub struct GlobalTolerance {
    inner: RwLock<Tolerance>,
}

impl GlobalTolerance {
    pub fn new() -> Self {
        Self {
            inner: RwLock::new(Tolerance::default()),
        }
    }

    // Setters (require write lock)
    pub fn set_absolute(&self, value: f64) { self.inner.write().set_absolute(value); }
    pub fn set_relative(&self, value: f64) { self.inner.write().set_relative(value); }
    pub fn set_angular(&self, value: f64) { self.inner.write().set_angular(value); }
    pub fn set_approximation(&self, value: f64) { self.inner.write().set_approximation(value); }
    pub fn set_precision(&self, value: i32) { self.inner.write().set_precision(value); }
    pub fn set_lineardeflection(&self, value: f64) { self.inner.write().set_lineardeflection(value); }
    pub fn set_angulardeflection(&self, value: f64) { self.inner.write().set_angulardeflection(value); }
    pub fn reset(&self) { self.inner.write().reset(); }

    // Getters (require read lock)
    pub fn absolute(&self) -> f64 { self.inner.read().absolute() }
    pub fn relative(&self) -> f64 { self.inner.read().relative() }
    pub fn angular(&self) -> f64 { self.inner.read().angular() }
    pub fn approximation(&self) -> f64 { self.inner.read().approximation() }
    pub fn precision(&self) -> i32 { self.inner.read().precision() }
    pub fn lineardeflection(&self) -> f64 { self.inner.read().lineardeflection() }
    pub fn angulardeflection(&self) -> f64 { self.inner.read().angulardeflection() }

    // Operations (require read lock)
    pub fn tolerance(&self, truevalue: f64, rtol: f64, atol: f64) -> f64 {
        self.inner.read().tolerance(truevalue, rtol, atol)
    }
    pub fn compare(&self, a: f64, b: f64, rtol: f64, atol: f64) -> bool {
        self.inner.read().compare(a, b, rtol, atol)
    }
    pub fn is_zero(&self, a: f64) -> bool { self.inner.read().is_zero(a) }
    pub fn is_positive(&self, a: f64) -> bool { self.inner.read().is_positive(a) }
    pub fn is_negative(&self, a: f64) -> bool { self.inner.read().is_negative(a) }
    pub fn is_between(&self, value: f64, minval: f64, maxval: f64) -> bool {
        self.inner.read().is_between(value, minval, maxval)
    }
    pub fn is_close(&self, a: f64, b: f64) -> bool { self.inner.read().is_close(a, b) }
    pub fn is_allclose(&self, a: &[f64], b: &[f64]) -> bool { self.inner.read().is_allclose(a, b) }
    pub fn is_angle_zero(&self, a: f64) -> bool { self.inner.read().is_angle_zero(a) }
    pub fn is_angles_close(&self, a: f64, b: f64) -> bool { self.inner.read().is_angles_close(a, b) }
    pub fn is_point_close(&self, a: &Point, b: &Point) -> bool { self.inner.read().is_point_close(a, b) }
    pub fn is_vector_close(&self, a: &Vector, b: &Vector) -> bool { self.inner.read().is_vector_close(a, b) }
    pub fn key(&self, xyz: [f64; 3], precision: i32) -> String { self.inner.read().key(xyz, precision) }
    pub fn key_xy(&self, xy: [f64; 2], precision: i32) -> String { self.inner.read().key_xy(xy, precision) }
    pub fn format_number(&self, number: f64, precision: i32) -> String {
        self.inner.read().format_number(number, precision)
    }
    pub fn precision_from_tolerance(&self, tol: Option<f64>) -> i32 {
        self.inner.read().precision_from_tolerance(tol)
    }

    /// Temporarily modify tolerance, restoring original state after closure returns
    pub fn temporary<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut Tolerance) -> R,
    {
        let saved = self.inner.read().clone();
        let result = f(&mut self.inner.write());
        *self.inner.write() = saved;
        result
    }
}

pub static TOLERANCE: Lazy<GlobalTolerance> = Lazy::new(GlobalTolerance::new);

/// Helper to read from global TOLERANCE
pub fn with_tolerance<F, R>(f: F) -> R
where
    F: FnOnce(&Tolerance) -> R,
{
    f(&TOLERANCE.inner.read())
}

/// Helper to write to global TOLERANCE
pub fn with_tolerance_mut<F, R>(f: F) -> R
where
    F: FnOnce(&mut Tolerance) -> R,
{
    f(&mut TOLERANCE.inner.write())
}

#[cfg(test)]
#[path = "tolerance_test.rs"]
mod tolerance_test;
