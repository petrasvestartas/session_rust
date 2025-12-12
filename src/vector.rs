use crate::point::Point;
use crate::tolerance::{Tolerance, TO_DEGREES, TO_RADIANS};
use serde::{ser::Serialize as SerTrait, Deserialize, Serialize};
use std::cell::Cell;
use std::fmt;
use std::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};
use uuid::Uuid;

/// A 3D vector with visual properties and JSON serialization support.
///
/// The Vector struct represents a mathematical vector in 3D space with
/// x, y, z components. It includes caching for magnitude calculations and
/// supports various geometric operations like dot product, cross product,
/// normalization, and angle calculations.
///
/// # Attributes
///
/// * `guid` - Unique identifier for the vector.
/// * `name` - Name of the vector.
/// * `_x` - X component (private, use indexing to access).
/// * `_y` - Y component (private, use indexing to access).
/// * `_z` - Z component (private, use indexing to access).
///
/// # Examples
///
/// ```
/// # use session_rust::Vector;
/// let v = Vector::new(1.0, 2.0, 3.0);
/// let unit = v.normalized();
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Vector")]
pub struct Vector {
    pub guid: String,
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
        Self{
            _x: 0.0,
            _y: 0.0,
            _z: 0.0,
            guid: Uuid::new_v4().to_string(),
            name: "my_vector".to_string(),
            _magnitude: Cell::new(0.0),
            _has_magnitude: Cell::new(false),
        }
    }
}

impl Vector {
    /// Creates a new Vector with specified coordinates.
    ///
    /// # Arguments
    ///
    /// * `x` - X component of the vector.
    /// * `y` - Y component of the vector.
    /// * `z` - Z component of the vector.
    ///
    /// # Returns
    ///
    /// A new Vector with the specified coordinates and a unique GUID.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            _x: x,
            _y: y,
            _z: z,
            ..Default::default()
        }
    }

    /// Creates a new Vector with specified coordinates and name.
    ///
    /// # Arguments
    ///
    /// * `x` - X component of the vector.
    /// * `y` - Y component of the vector.
    /// * `z` - Z component of the vector.
    /// * `name` - Name for the vector.
    ///
    /// # Returns
    ///
    /// A new Vector with the specified coordinates, name, and a unique GUID.
    pub fn with_name(x: f64, y: f64, z: f64, name: &str) -> Self {
        Self {
            _x: x,
            _y: y,
            _z: z,
            name: name.to_string(),
            ..Default::default()
        }
    }

    /// Deep copy this vector with a new GUID.
    ///
    /// Creates a clone of this vector with all properties copied
    /// but assigns a new unique GUID.
    ///
    /// # Returns
    ///
    /// A new Vector with identical values but a different GUID.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = Uuid::new_v4().to_string();
        copy
    }


    /// Simple string form (like Python __str__): just coordinates.
    ///
    /// # Returns
    ///
    /// A string in the format "x, y, z".
    pub fn str(&self) -> String {
        use crate::tolerance::TOLERANCE;
        let prec = crate::tolerance::Tolerance::ROUNDING;
        format!(
            "{}, {}, {}",
            TOLERANCE.format_number(self._x, prec),
            TOLERANCE.format_number(self._y, prec),
            TOLERANCE.format_number(self._z, prec),
        )
    }

    /// Detailed representation (like Python __repr__).
    ///
    /// # Returns
    ///
    /// A string with full vector details including name, coordinates, magnitude.
    pub fn repr(&mut self) -> String {
        use crate::tolerance::TOLERANCE;
        let prec = crate::tolerance::Tolerance::ROUNDING;
        let mag = self.magnitude(); // compute first to avoid borrow conflict
        format!(
            "Vector({}, {}, {}, {}, {})",
            self.name,
            TOLERANCE.format_number(self._x, prec),
            TOLERANCE.format_number(self._y, prec),
            TOLERANCE.format_number(self._z, prec),
            TOLERANCE.format_number(mag, prec),
        )
    }

    /// Creates a zero vector.
    ///
    /// # Returns
    ///
    /// A Vector with all components set to 0.0.
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// Creates a unit vector along X-axis.
    ///
    /// Returns
    /// -------
    /// Vector
    ///     Unit vector (1, 0, 0).
    pub fn x_axis() -> Self {
        Self::new(1.0, 0.0, 0.0)
    }

    /// Creates a unit vector along Y-axis.
    ///
    /// Returns
    /// -------
    /// Vector
    ///     Unit vector (0, 1, 0).
    pub fn y_axis() -> Self {
        Self::new(0.0, 1.0, 0.0)
    }

    /// Creates a unit vector along Z-axis.
    ///
    /// Returns
    /// -------
    /// Vector
    ///     Unit vector (0, 0, 1).
    pub fn z_axis() -> Self {
        Self::new(0.0, 0.0, 1.0)
    }

    /// Creates a vector from two points (p1 - p0).
    ///
    /// Parameters
    /// ----------
    /// p0 : &Point
    ///     The start point.
    /// p1 : &Point
    ///     The end point.
    ///
    /// Returns
    /// -------
    /// Vector
    ///     The vector from p0 to p1 (p1 - p0).
    pub fn from_points(p0: &Point, p1: &Point) -> Self {
        Self::new(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Vector Operations
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Invalidates the cached magnitude when coordinates change.
    fn invalidate_magnitude_cache(&self) {
        self._has_magnitude.set(false);
    }

    /// Computes the magnitude of the vector (internal, not cached).
    fn compute_magnitude(&self) -> f64 {
        (self._x * self._x + self._y * self._y + self._z * self._z).sqrt()
    }

    /// Gets the magnitude of the vector (cached).
    ///
    /// The magnitude is computed once and cached. When coordinates change,
    /// the cache is automatically invalidated and recomputed on next call.
    ///
    /// # Returns
    ///
    /// The magnitude (length) of the vector.
    pub fn magnitude(&self) -> f64 {
        if !self._has_magnitude.get() {
            self._magnitude.set(self.compute_magnitude());
            self._has_magnitude.set(true);
        }
        self._magnitude.get()
    }

    /// Computes the squared magnitude of the vector (avoids sqrt for performance).
    ///
    /// This is more efficient than `magnitude()` when only comparing magnitudes,
    /// as it avoids the square root computation.
    ///
    /// # Returns
    ///
    /// The squared magnitude of the vector (x² + y² + z²).
    pub fn magnitude_squared(&self) -> f64 {
        self._x * self._x + self._y * self._y + self._z * self._z
    }

    /// Normalizes the vector in place.
    ///
    /// Modifies this vector to have unit magnitude (magnitude = 1.0).
    /// If the vector has zero magnitude, it remains unchanged.
    pub fn normalize(&mut self) {
        let len = self.compute_magnitude();
        if len > Tolerance::ZERO_TOLERANCE {
            self._x /= len;
            self._y /= len;
            self._z /= len;
            self.invalidate_magnitude_cache();
        }
    }

    /// Returns a normalized copy of the vector.
    ///
    /// # Returns
    ///
    /// A new Vector with unit magnitude pointing in the same direction.
    pub fn normalized(&self) -> Self {
        let mut result = self.clone();
        result.normalize();
        result
    }

    /// Reverses the vector direction in place.
    ///
    /// Negates all components of the vector, effectively pointing it
    /// in the opposite direction. The magnitude remains unchanged.
    pub fn reverse(&mut self) {
        self._x = -self._x;
        self._y = -self._y;
        self._z = -self._z;
        // Length magnitude stays the same, no need to invalidate cache
    }


    /// Computes the dot product with another vector.
    ///
    /// Parameters
    /// ----------
    /// other : &Vector
    ///     Other vector.
    ///
    /// Returns
    /// -------
    /// f64
    ///     Dot product value.
    pub fn dot(&self, other: &Vector) -> f64 {
        self._x * other._x + self._y * other._y + self._z * other._z
    }

    /// Computes the cross product with another vector.
    ///
    /// Parameters
    /// ----------
    /// other : &Vector
    ///     Other vector.
    ///
    /// Returns
    /// -------
    /// Vector
    ///     Cross product vector (orthogonal to inputs).
    pub fn cross(&self, other: &Vector) -> Vector {
        Vector::new(
            self._y * other._z - self._z * other._y,
            self._z * other._x - self._x * other._z,
            self._x * other._y - self._y * other._x,
        )
    }

    /// Computes the angle between this vector and another in degrees.
    ///
    /// # Arguments
    ///
    /// * `other` - The other vector to compute angle with.
    /// * `sign_by_cross_product` - If true, the sign of the angle is determined
    ///   by the z-component of the cross product.
    ///
    /// # Returns
    ///
    /// The angle between the vectors in degrees. Returns 0.0 if either vector
    /// has zero length.
    pub fn angle(&self, other: &Vector, sign_by_cross_product: bool) -> f64 {
        let dotp = self.dot(other);
        let len_product = self.compute_magnitude() * other.compute_magnitude();

        if len_product < Tolerance::ZERO_TOLERANCE {
            return 0.0;
        }

        let cos_angle = (dotp / len_product).clamp(-1.0, 1.0);
        let mut angle = cos_angle.acos() * TO_DEGREES;

        if sign_by_cross_product {
            let cp = self.cross(other);
            if cp[2] < 0.0 {
                angle = -angle;
            }
        }

        angle
    }


    /// Computes the angle between vector XY components in degrees.
    ///
    /// Calculates atan2(y, x) to get the angle of the vector's projection
    /// onto the XY plane, measured from the positive X-axis.
    ///
    /// # Arguments
    ///
    /// * `vector` - The vector to compute the angle for.
    ///
    /// # Returns
    ///
    /// The angle in degrees.
    pub fn angle_between_vector_xy_components(vector: &Vector) -> f64 {
        vector._y.atan2(vector._x) * TO_DEGREES
    }

    /// Projects this vector onto another vector and returns detailed results.
    ///
    /// Returns a tuple of:
    /// - projection vector of `self` onto `onto`
    /// - projected length (scalar projection)
    /// - perpendicular projected vector (self - projection)
    /// - perpendicular projected vector length
    pub fn projection(&self, onto: &Vector) -> (Vector, f64, Vector, f64) {
        let onto_len_sq = onto.magnitude_squared();

        if onto_len_sq < Tolerance::ZERO_TOLERANCE {
            return (Vector::zero(), 0.0, Vector::zero(), 0.0);
        }

        // Unit vector along 'onto'
        let onto_len = onto_len_sq.sqrt();
        let onto_unit = Vector::new(onto._x / onto_len, onto._y / onto_len, onto._z / onto_len);

        // Scalar projection and projected vector
        let projected_len = self.dot(&onto_unit);
        let projection_vec = Vector::new(
            onto_unit._x * projected_len,
            onto_unit._y * projected_len,
            onto_unit._z * projected_len,
        );

        // Perpendicular component and its length
        let perp_vec = Vector::new(
            self._x - projection_vec._x,
            self._y - projection_vec._y,
            self._z - projection_vec._z,
        );
        let perp_len = perp_vec.compute_magnitude();

        (projection_vec, projected_len, perp_vec, perp_len)
    }

    /// Checks if this vector is parallel to another vector.
    ///
    /// # Arguments
    ///
    /// * `other` - The other vector to check parallelism with.
    ///
    /// # Returns
    ///
    /// * `1` - Vectors are parallel (same direction).
    /// * `-1` - Vectors are antiparallel (opposite direction).
    /// * `0` - Vectors are not parallel.
    pub fn is_parallel_to(&self, other: &Vector) -> i32 {
        let len_product = self.compute_magnitude() * other.compute_magnitude();

        if len_product <= 0.0 {
            return 0;
        }

        let cos_angle = self.dot(other) / len_product;
        let angle_in_radians = Tolerance::ANGLE_TOLERANCE_DEGREES * TO_RADIANS;
        let cos_tolerance = angle_in_radians.cos();

        if cos_angle >= cos_tolerance {
            1 // Parallel
        } else if cos_angle <= -cos_tolerance {
            -1 // Antiparallel
        } else {
            0 // Not parallel
        }
    }



    /// Checks if this vector is perpendicular to another vector.
    ///
    /// Two vectors are perpendicular if their dot product is zero
    /// (within tolerance).
    ///
    /// # Arguments
    ///
    /// * `other` - The other vector to check against.
    ///
    /// # Returns
    ///
    /// `true` if the vectors are perpendicular, `false` otherwise.
    pub fn is_perpendicular_to(&self, other: &Vector) -> bool {
        self.dot(other).abs() < Tolerance::ZERO_TOLERANCE
    }



    /// Set this vector to be perpendicular to the given vector.
    ///
    /// Modifies this vector to be perpendicular to `v`. The algorithm
    /// chooses the perpendicular direction based on the dominant component
    /// of the input vector.
    ///
    /// # Arguments
    ///
    /// * `v` - The reference vector to be perpendicular to.
    ///
    /// # Returns
    ///
    /// `true` if successful, `false` if the input vector is zero.
    pub fn perpendicular_to(&mut self, v: &Vector) -> bool {
        // Ported from Python implementation to ensure identical behavior
        let i: usize;
        let j: usize;
        let k: usize;
        let a: f64;
        let b: f64;

        if v[1].abs() > v[0].abs() {
            if v[2].abs() > v[1].abs() {
                // |v.z| > |v.y| > |v.x|
                i = 2;
                j = 1;
                k = 0;
                a = v[2];
                b = -v[1];
            } else if v[2].abs() >= v[0].abs() {
                // |v.y| >= |v.z| >= |v.x|
                i = 1;
                j = 2;
                k = 0;
                a = v[1];
                b = -v[2];
            } else {
                // |v.y| > |v.x| > |v.z|
                i = 1;
                j = 0;
                k = 2;
                a = v[1];
                b = -v[0];
            }
        } else if v[2].abs() > v[0].abs() {
            // |v.z| > |v.x| >= |v.y|
            i = 2;
            j = 0;
            k = 1;
            a = v[2];
            b = -v[0];
        } else if v[2].abs() > v[1].abs() {
            // |v.x| >= |v.z| > |v.y|
            i = 0;
            j = 2;
            k = 1;
            a = v[0];
            b = -v[2];
        } else {
            // |v.x| >= |v.y| >= |v.z|
            i = 0;
            j = 1;
            k = 2;
            a = v[0];
            b = -v[1];
        }

        let mut coords = [0.0, 0.0, 0.0];
        coords[i] = b;
        coords[j] = a;
        coords[k] = 0.0;

        self._x = coords[0];
        self._y = coords[1];
        self._z = coords[2];
        self.invalidate_magnitude_cache();

        a != 0.0
    }


    /// Gets a leveled vector scaled by vertical height.
    ///
    /// Creates a normalized copy of this vector and scales it based on
    /// the vertical height and the angle with the Z-axis.
    ///
    /// # Arguments
    ///
    /// * `vertical_height` - The target vertical height for scaling.
    ///
    /// # Returns
    ///
    /// A new Vector scaled according to the vertical height.
    ///
    /// # Note
    ///
    /// Computes the inclined distance needed to achieve a given vertical rise.
    pub fn get_leveled_vector(&self, vertical_height: f64) -> Vector {
        let mut copy = self.clone();
        copy.normalize();

        if vertical_height != 0.0 {
            let reference = Vector::z_axis();
            let angle_deg = copy.angle(&reference, false); // returns degrees (unsigned)
            let angle_rad = angle_deg * TO_RADIANS;
            let inclined_offset_by_vertical_distance = vertical_height / angle_rad.cos();
            copy *= inclined_offset_by_vertical_distance;
        }

        copy
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Static Methods
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Computes the third side of a triangle using the cosine law.
    ///
    /// Given two sides and the angle between them, calculates the length
    /// of the third side: c² = a² + b² - 2ab·cos(C)
    ///
    /// # Arguments
    ///
    /// * `triangle_edge_length_a` - Length of side a.
    /// * `triangle_edge_length_b` - Length of side b.
    /// * `angle_in_degrees_between_edges` - Angle between sides a and b.
    /// * `degrees` - If true, angle is in degrees; otherwise radians.
    ///
    /// # Returns
    ///
    /// Length of the third side.
    pub fn cosine_law(
        triangle_edge_length_a: f64,
        triangle_edge_length_b: f64,
        angle_in_degrees_between_edges: f64,
        degrees: bool,
    ) -> f64 {
        let angle = if degrees {
            angle_in_degrees_between_edges * TO_RADIANS
        } else {
            angle_in_degrees_between_edges
        };

        (triangle_edge_length_a.powi(2) + triangle_edge_length_b.powi(2)
            - 2.0 * triangle_edge_length_a * triangle_edge_length_b * angle.cos())
        .sqrt()
    }

    /// Computes an angle of a triangle using the inverse cosine law.
    ///
    /// Given all three sides, calculates the angle opposite to the third side:
    /// cos(C) = (a² + b² - c²) / (2ab)
    ///
    /// # Arguments
    ///
    /// * `triangle_edge_length_a` - Length of side a (adjacent to angle C).
    /// * `triangle_edge_length_b` - Length of side b (adjacent to angle C).
    /// * `triangle_edge_length_c` - Length of side c (opposite to angle C).
    /// * `degrees` - If true, returns angle in degrees; otherwise radians.
    ///
    /// # Returns
    ///
    /// The angle opposite to side c.
    pub fn angle_from_cosine_law(
        triangle_edge_length_a: f64,
        triangle_edge_length_b: f64,
        triangle_edge_length_c: f64,
        degrees: bool,
    ) -> f64 {
        let a = triangle_edge_length_a;
        let b = triangle_edge_length_b;
        let c = triangle_edge_length_c;

        // cos(C) = (a² + b² - c²) / (2ab)
        let cos_c = (a.powi(2) + b.powi(2) - c.powi(2)) / (2.0 * a * b);
        let angle_rad = cos_c.acos();

        if degrees {
            angle_rad * TO_DEGREES
        } else {
            angle_rad
        }
    }

    /// Computes an angle of a triangle using the sine law.
    ///
    /// Given two sides and the angle opposite to one of them, calculates
    /// the angle opposite to the other side: sin(A)/a = sin(B)/b
    ///
    /// # Arguments
    ///
    /// * `triangle_edge_length_a` - Length of side a.
    /// * `angle_in_degrees_in_front_of_a` - Angle opposite to side a.
    /// * `triangle_edge_length_b` - Length of side b.
    /// * `degrees` - If true, angles are in degrees; otherwise radians.
    ///
    /// # Returns
    ///
    /// The angle opposite to side b.
    pub fn sine_law_angle(
        triangle_edge_length_a: f64,
        angle_in_degrees_in_front_of_a: f64,
        triangle_edge_length_b: f64,
        degrees: bool,
    ) -> f64 {
        let angle_a = if degrees {
            angle_in_degrees_in_front_of_a * TO_RADIANS
        } else {
            angle_in_degrees_in_front_of_a
        };

        let sin_b = (triangle_edge_length_b * angle_a.sin()) / triangle_edge_length_a;
        let angle_b = sin_b.asin();

        if degrees {
            angle_b * TO_DEGREES
        } else {
            angle_b
        }
    }

    /// Computes a side length of a triangle using the sine law.
    ///
    /// Given one side and two angles, calculates the length of another side:
    /// a/sin(A) = b/sin(B)
    ///
    /// # Arguments
    ///
    /// * `triangle_edge_length_a` - Length of side a.
    /// * `angle_in_degrees_in_front_of_a` - Angle opposite to side a.
    /// * `angle_in_degrees_in_front_of_b` - Angle opposite to side b.
    /// * `degrees` - If true, angles are in degrees; otherwise radians.
    ///
    /// # Returns
    ///
    /// Length of side b.
    pub fn sine_law_length(
        triangle_edge_length_a: f64,
        angle_in_degrees_in_front_of_a: f64,
        angle_in_degrees_in_front_of_b: f64,
        degrees: bool,
    ) -> f64 {
        let angle_a = if degrees {
            angle_in_degrees_in_front_of_a * TO_RADIANS
        } else {
            angle_in_degrees_in_front_of_a
        };

        let angle_b = if degrees {
            angle_in_degrees_in_front_of_b * TO_RADIANS
        } else {
            angle_in_degrees_in_front_of_b
        };

        (triangle_edge_length_a * angle_b.sin()) / angle_a.sin()
    }

    /// Computes a side length of a triangle using the sine law (alternative signature).
    ///
    /// Given two angles and one side, calculates the length of another side:
    /// a/sin(A) = b/sin(B)  =>  a = b·sin(A)/sin(B)
    ///
    /// # Arguments
    ///
    /// * `angle_in_front_of_result_side` - Angle opposite to the side we want to find.
    /// * `angle_in_front_of_known_side` - Angle opposite to the known side.
    /// * `known_side_length` - Length of the known side.
    /// * `degrees` - If true, angles are in degrees; otherwise radians.
    ///
    /// # Returns
    ///
    /// Length of the side opposite to the first angle.
    pub fn side_from_sine_law(
        angle_in_front_of_result_side: f64,
        angle_in_front_of_known_side: f64,
        known_side_length: f64,
        degrees: bool,
    ) -> f64 {
        let angle_a = if degrees {
            angle_in_front_of_result_side * TO_RADIANS
        } else {
            angle_in_front_of_result_side
        };

        let angle_b = if degrees {
            angle_in_front_of_known_side * TO_RADIANS
        } else {
            angle_in_front_of_known_side
        };

        // a = b·sin(A)/sin(B)
        (known_side_length * angle_a.sin()) / angle_b.sin()
    }

    /// Sums a collection of vectors component-wise.
    ///
    /// # Arguments
    ///
    /// * `vectors` - Slice of vectors to sum.
    ///
    /// # Returns
    ///
    /// A new Vector containing the component-wise sum of all input vectors.
    pub fn sum_of_vectors(vectors: &[Vector]) -> Vector {
        let mut result = Vector::zero();
        for vector in vectors {
            result._x += vector._x;
            result._y += vector._y;
            result._z += vector._z;
        }
        result
    }

    /// Computes the average of a collection of vectors.
    ///
    /// # Arguments
    ///
    /// * `vectors` - Slice of vectors to average.
    ///
    /// # Returns
    ///
    /// A new Vector representing the component-wise average.
    /// Returns a zero vector if the slice is empty.
    pub fn average(vectors: &[Vector]) -> Vector {
        if vectors.is_empty() {
            return Vector::zero();
        }
        let sum = Self::sum_of_vectors(vectors);
        let count = vectors.len() as f64;
        Vector::new(sum._x / count, sum._y / count, sum._z / count)
    }

    /// Checks if this vector is a zero vector.
    ///
    /// A vector is considered zero if its length is less than the
    /// zero tolerance.
    ///
    /// # Returns
    ///
    /// `true` if the vector is effectively zero, `false` otherwise.
    pub fn is_zero(&self) -> bool {
        self.compute_magnitude() < Tolerance::ZERO_TOLERANCE
    }

    /// Computes coordinate direction angles (alpha, beta, gamma).
    ///
    /// These are the angles between the vector and each coordinate axis.
    ///
    /// # Arguments
    ///
    /// * `degrees` - If true, return angles in degrees; otherwise radians.
    ///
    /// # Returns
    ///
    /// An array [alpha, beta, gamma] representing angles with X, Y, Z axes.
    pub fn coordinate_direction_3angles(&self, degrees: bool) -> [f64; 3] {
        let length = self.compute_magnitude();
        if length < Tolerance::ZERO_TOLERANCE {
            return [0.0, 0.0, 0.0];
        }

        let cos_alpha = self._x / length;
        let cos_beta = self._y / length;
        let cos_gamma = self._z / length;

        let alpha = cos_alpha.acos();
        let beta = cos_beta.acos();
        let gamma = cos_gamma.acos();

        if degrees {
            [alpha * TO_DEGREES, beta * TO_DEGREES, gamma * TO_DEGREES]
        } else {
            [alpha, beta, gamma]
        }
    }

    /// Computes spherical coordinate angles (phi, theta).
    ///
    /// * phi - Azimuthal angle in the XY plane from the X-axis.
    /// * theta - Polar angle from the XY plane.
    ///
    /// # Arguments
    ///
    /// * `degrees` - If true, return angles in degrees; otherwise radians.
    ///
    /// # Returns
    ///
    /// An array [phi, theta] representing spherical coordinate angles.
    pub fn coordinate_direction_2angles(&self, degrees: bool) -> [f64; 2] {
        let length_xy = (self._x * self._x + self._y * self._y).sqrt();
        let length = self.compute_magnitude();

        if length < Tolerance::ZERO_TOLERANCE {
            return [0.0, 0.0];
        }

        let phi = self._y.atan2(self._x);
        let theta = length_xy.atan2(self._z);

        if degrees {
            [phi * TO_DEGREES, theta * TO_DEGREES]
        } else {
            [phi, theta]
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serializes the Vector to a JSON string.
    ///
    /// # Returns
    ///
    /// A Result containing the pretty-printed JSON string or an error.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let mut buf = Vec::new();
        let formatter = serde_json::ser::PrettyFormatter::with_indent(b"    ");
        let mut ser = serde_json::Serializer::with_formatter(&mut buf, formatter);
        SerTrait::serialize(self, &mut ser)?;
        Ok(String::from_utf8(buf)?)
    }

    /// Deserializes a Vector from a JSON string.
    ///
    /// # Arguments
    ///
    /// * `json_data` - JSON string containing vector data.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized Vector or an error.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serializes the Vector to a JSON file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the output file.
    ///
    /// # Returns
    ///
    /// A Result indicating success or an error.
    pub fn to_json(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    /// Alias for `to_json` to match C++ API naming convention.
    pub fn json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        self.to_json(filepath)
    }

    /// Deserializes a Vector from a JSON file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the JSON file.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized Vector or an error.
    pub fn from_json(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json)
    }

    /// Alias for `from_json` to match C++ API naming convention.
    pub fn json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::from_json(filepath)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    #[cfg(feature = "protobuf")]
    /// Convert to protobuf binary format.
    ///
    /// # Returns
    ///
    /// A Vec<u8> containing the serialized protobuf data.
    pub fn to_protobuf(&self) -> Vec<u8> {
        use prost::Message;
        
        let proto = crate::proto::Vector {
            x: self._x,
            y: self._y,
            z: self._z,
            name: self.name.clone(),
        };
        proto.encode_to_vec()
    }

    #[cfg(feature = "protobuf")]
    /// Create Vector from protobuf binary data.
    ///
    /// # Arguments
    ///
    /// * `data` - Byte slice containing protobuf-encoded vector data.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized Vector or an error.
    pub fn from_protobuf(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        
        let proto = crate::proto::Vector::decode(data)?;
        
        let mut v = Self::new(proto.x, proto.y, proto.z);
        v.name = proto.name;
        
        Ok(v)
    }

    #[cfg(feature = "protobuf")]
    /// Write protobuf to file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the output file.
    pub fn protobuf_dump(&self, filepath: &str) {
        let data = self.to_protobuf();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    #[cfg(feature = "protobuf")]
    /// Read protobuf from file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the protobuf file.
    ///
    /// # Returns
    ///
    /// The deserialized Vector.
    pub fn protobuf_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::from_protobuf(&data).expect("Failed to parse protobuf")
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

// Index trait for array-like access
impl Index<usize> for Vector {
    type Output = f64;

    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self._x,
            1 => &self._y,
            2 => &self._z,
            _ => panic!("Index out of bounds for Vector"),
        }
    }
}

impl IndexMut<usize> for Vector {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        self.invalidate_magnitude_cache();
        match index {
            0 => &mut self._x,
            1 => &mut self._y,
            2 => &mut self._z,
            _ => panic!("Index out of bounds for Vector"),
        }
    }
}

// Arithmetic operators
impl Add for Vector {
    type Output = Vector;

    fn add(self, other: Vector) -> Vector {
        Vector::new(
            self[0] + other[0],
            self[1] + other[1],
            self[2] + other[2],
        )
    }
}

impl Add for &Vector {
    type Output = Vector;

    fn add(self, other: &Vector) -> Vector {
        Vector::new(
            self[0] + other[0],
            self[1] + other[1],
            self[2] + other[2],
        )
    }
}

impl Add<Vector> for &Vector {
    type Output = Vector;

    fn add(self, other: Vector) -> Vector {
        Vector::new(
            self[0] + other[0],
            self[1] + other[1],
            self[2] + other[2],
        )
    }
}

impl Sub for Vector {
    type Output = Vector;

    fn sub(self, other: Vector) -> Vector {
        Vector::new(
            self[0] - other[0],
            self[1] - other[1],
            self[2] - other[2],
        )
    }
}

impl Sub for &Vector {
    type Output = Vector;

    fn sub(self, other: &Vector) -> Vector {
        Vector::new(
            self[0] - other[0],
            self[1] - other[1],
            self[2] - other[2],
        )
    }
}

impl Sub<Vector> for &Vector {
    type Output = Vector;

    fn sub(self, other: Vector) -> Vector {
        Vector::new(
            self[0] - other[0],
            self[1] - other[1],
            self[2] - other[2],
        )
    }
}

impl Mul<f64> for Vector {
    type Output = Vector;

    fn mul(self, scalar: f64) -> Vector {
        Vector::new(self[0] * scalar, self[1] * scalar, self[2] * scalar)
    }
}

impl Mul<f64> for &Vector {
    type Output = Vector;

    fn mul(self, scalar: f64) -> Vector {
        Vector::new(self[0] * scalar, self[1] * scalar, self[2] * scalar)
    }
}

impl Div<f64> for Vector {
    type Output = Vector;

    fn div(self, scalar: f64) -> Vector {
        Vector::new(self[0] / scalar, self[1] / scalar, self[2] / scalar)
    }
}

impl Div<f64> for &Vector {
    type Output = Vector;

    fn div(self, scalar: f64) -> Vector {
        Vector::new(self[0] / scalar, self[1] / scalar, self[2] / scalar)
    }
}

impl Neg for Vector {
    type Output = Vector;

    fn neg(self) -> Vector {
        Vector::new(-self[0], -self[1], -self[2])
    }
}

impl Neg for &Vector {
    type Output = Vector;

    fn neg(self) -> Vector {
        Vector::new(-self[0], -self[1], -self[2])
    }
}

// Compound assignment operators
impl AddAssign for Vector {
    fn add_assign(&mut self, other: Vector) {
        self[0] += other[0];
        self[1] += other[1];
        self[2] += other[2];
    }
}

impl AddAssign<&Vector> for Vector {
    fn add_assign(&mut self, other: &Vector) {
        self[0] += other[0];
        self[1] += other[1];
        self[2] += other[2];
    }
}

impl SubAssign for Vector {
    fn sub_assign(&mut self, other: Vector) {
        self[0] -= other[0];
        self[1] -= other[1];
        self[2] -= other[2];
    }
}

impl SubAssign<&Vector> for Vector {
    fn sub_assign(&mut self, other: &Vector) {
        self[0] -= other[0];
        self[1] -= other[1];
        self[2] -= other[2];
    }
}

impl MulAssign<f64> for Vector {
    fn mul_assign(&mut self, scalar: f64) {
        self[0] *= scalar;
        self[1] *= scalar;
        self[2] *= scalar;
    }
}

impl DivAssign<f64> for Vector {
    fn div_assign(&mut self, scalar: f64) {
        self[0] /= scalar;
        self[1] /= scalar;
        self[2] /= scalar;
    }
}

impl fmt::Display for Vector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Vector({}, {}, {}, {}, {})",
            self[0],
            self[1],
            self[2],
            self.guid,
            self.name
        )
    }
}

#[cfg(test)]
#[path = "vector_test.rs"]
mod tests;
