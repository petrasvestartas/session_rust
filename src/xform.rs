use crate::{Line, Plane, Point, Vector};
use crate::tolerance::Tolerance;
use serde::{Deserialize, Deserializer, Serializer};
use serde::ser::SerializeMap;
use std::fmt;
use std::ops::{Index, IndexMut, Mul, MulAssign};

/// A 4x4 column-major transformation matrix in 3D space
#[derive(Clone)]
pub struct Xform {
    pub typ: String,
    guid: std::sync::OnceLock<String>,
    pub name: String,
    /// The matrix elements stored in column-major order as a flattened array
    pub m: [f64; 16],
}

impl serde::Serialize for Xform {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut map = serializer.serialize_map(Some(4))?;
        map.serialize_entry("guid", self.guid())?;
        map.serialize_entry("m", &self.m)?;
        map.serialize_entry("name", &self.name)?;
        map.serialize_entry("type", "Xform")?;
        map.end()
    }
}

impl<'de> Deserialize<'de> for Xform {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct XformData {
            #[serde(default)]
            guid: Option<String>,
            #[serde(default)]
            name: Option<String>,
            #[serde(default = "default_matrix")]
            m: [f64; 16],
        }
        fn default_matrix() -> [f64; 16] {
            [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        }
        let data = XformData::deserialize(deserializer)?;
        let guid = std::sync::OnceLock::new();
        if let Some(g) = data.guid {
            let _ = guid.set(g);
        }
        Ok(Xform {
            typ: "Xform".to_string(),
            guid,
            name: data.name.unwrap_or_else(|| "my_xform".to_string()),
            m: data.m,
        })
    }
}

impl Xform {
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn new() -> Self {
        Self::identity()
    }

    pub fn from_matrix(matrix: [f64; 16]) -> Self {
        Xform {
            typ: "Xform".to_string(),
            guid: std::sync::OnceLock::new(),
            name: "my_xform".to_string(),
            m: matrix,
        }
    }

    pub fn identity() -> Self {
        use std::sync::OnceLock;
        static IDENTITY: OnceLock<Xform> = OnceLock::new();
        IDENTITY.get_or_init(|| {
            let mut m = [0.0f64; 16];
            m[0] = 1.0; m[5] = 1.0; m[10] = 1.0; m[15] = 1.0;
            Xform { typ: "Xform".to_string(), guid: std::sync::OnceLock::new(),
                    name: "my_xform".to_string(), m }
        }).clone()
    }

    pub fn from_cols(col_x: Vector, col_y: Vector, col_z: Vector) -> Self {
        let mut xform = Self::identity();
        xform.m[0] = col_x[0];
        xform.m[1] = col_x[1];
        xform.m[2] = col_x[2];
        xform.m[4] = col_y[0];
        xform.m[5] = col_y[1];
        xform.m[6] = col_y[2];
        xform.m[8] = col_z[0];
        xform.m[9] = col_z[1];
        xform.m[10] = col_z[2];
        xform
    }

    /// Build a pure rotation (no translation) from three column axis vectors.
    /// Port of wood `internal::rotation_in_xy_plane(x, y, z)`.
    pub fn from_axes(col_x: &Vector, col_y: &Vector, col_z: &Vector) -> Self {
        Self::from_cols(col_x.clone(), col_y.clone(), col_z.clone())
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformations
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn translation(x: f64, y: f64, z: f64) -> Self {
        let mut xform = Self::identity();
        xform.m[12] = x;
        xform.m[13] = y;
        xform.m[14] = z;
        xform
    }

    pub fn rotation_x(angle: f64, degrees: bool) -> Self {
        let angle = if degrees { angle * Tolerance::TO_RADIANS } else { angle };
        let mut xform = Self::identity();

        let cos_angle = angle.cos();
        let sin_angle = angle.sin();

        xform.m[5] = cos_angle;
        xform.m[6] = sin_angle;
        xform.m[9] = -sin_angle;
        xform.m[10] = cos_angle;

        xform
    }

    pub fn rotation_y(angle: f64, degrees: bool) -> Self {
        let angle = if degrees { angle * Tolerance::TO_RADIANS } else { angle };
        let mut xform = Self::identity();

        let cos_angle = angle.cos();
        let sin_angle = angle.sin();

        xform.m[0] = cos_angle;
        xform.m[2] = -sin_angle;
        xform.m[8] = sin_angle;
        xform.m[10] = cos_angle;

        xform
    }

    pub fn rotation_z(angle: f64, degrees: bool) -> Self {
        let angle = if degrees { angle * Tolerance::TO_RADIANS } else { angle };
        let mut xform = Self::identity();
        let cos_angle = angle.cos();
        let sin_angle = angle.sin();

        xform.m[0] = cos_angle;
        xform.m[1] = sin_angle;
        xform.m[4] = -sin_angle;
        xform.m[5] = cos_angle;

        xform
    }

    pub fn rotation(axis: &Vector, angle: f64, degrees: bool) -> Self {
        let angle = if degrees { angle * Tolerance::TO_RADIANS } else { angle };
        let axis = axis.normalized();

        let mut xform = Self::identity();
        let cos_angle = angle.cos();
        let sin_angle = angle.sin();
        let one_minus_cos = 1.0 - cos_angle;

        let xx = axis[0] * axis[0];
        let xy = axis[0] * axis[1];
        let xz = axis[0] * axis[2];
        let yy = axis[1] * axis[1];
        let yz = axis[1] * axis[2];
        let zz = axis[2] * axis[2];

        xform.m[0] = cos_angle + xx * one_minus_cos;
        xform.m[1] = xy * one_minus_cos + axis[2] * sin_angle;
        xform.m[2] = xz * one_minus_cos - axis[1] * sin_angle;

        xform.m[4] = xy * one_minus_cos - axis[2] * sin_angle;
        xform.m[5] = cos_angle + yy * one_minus_cos;
        xform.m[6] = yz * one_minus_cos + axis[0] * sin_angle;

        xform.m[8] = xz * one_minus_cos + axis[1] * sin_angle;
        xform.m[9] = yz * one_minus_cos - axis[0] * sin_angle;
        xform.m[10] = cos_angle + zz * one_minus_cos;

        xform
    }

    pub fn rotation_around_line(line: &Line, angle: f64, degrees: bool) -> Self {
        let p = line.start();
        let d = line.to_direction();
        let t0 = Self::translation(-p[0], -p[1], -p[2]);
        let r = Self::rotation(&d, angle, degrees);
        let t1 = Self::translation(p[0], p[1], p[2]);
        t1 * (r * t0)
    }

    pub fn look_at_right_handed(eye: &Point, target: &Point, up: &Vector) -> Self {
        let fx = target[0] - eye[0];
        let fy = target[1] - eye[1];
        let fz = target[2] - eye[2];
        let f_len = (fx * fx + fy * fy + fz * fz).sqrt();
        let f = Vector::new(fx / f_len, fy / f_len, fz / f_len);

        let s = f.cross(&up.normalized()).normalized();
        let u = s.cross(&f);

        let mut xform = Self::identity();

        xform.m[0] = s[0];
        xform.m[4] = s[1];
        xform.m[8] = s[2];

        xform.m[1] = u[0];
        xform.m[5] = u[1];
        xform.m[9] = u[2];

        xform.m[2] = -f[0];
        xform.m[6] = -f[1];
        xform.m[10] = -f[2];

        xform.m[12] = -s.dot(&Vector::new(eye[0], eye[1], eye[2]));
        xform.m[13] = -u.dot(&Vector::new(eye[0], eye[1], eye[2]));
        xform.m[14] = f.dot(&Vector::new(eye[0], eye[1], eye[2]));

        xform
    }

    pub fn look_to_right_handed(eye: &Point, direction: &Vector, up: &Vector) -> Self {
        let f = direction.normalized();
        let s = f.cross(&up.normalized()).normalized();
        let u = s.cross(&f);

        let mut xform = Self::identity();

        xform.m[0] = s[0];
        xform.m[4] = s[1];
        xform.m[8] = s[2];

        xform.m[1] = u[0];
        xform.m[5] = u[1];
        xform.m[9] = u[2];

        xform.m[2] = -f[0];
        xform.m[6] = -f[1];
        xform.m[10] = -f[2];

        xform.m[12] = -s.dot(&Vector::new(eye[0], eye[1], eye[2]));
        xform.m[13] = -u.dot(&Vector::new(eye[0], eye[1], eye[2]));
        xform.m[14] = f.dot(&Vector::new(eye[0], eye[1], eye[2]));

        xform
    }

    pub fn perspective(fov_y: f64, aspect: f64, near: f64, far: f64) -> Self {
        let f = 1.0 / (fov_y / 2.0).tan();
        let nf = near - far;
        let mut xform = Xform::new();
        xform.m = [0.0; 16];
        xform.m[0] = f / aspect;
        xform.m[5] = f;
        xform.m[10] = far / nf;
        xform.m[11] = -1.0;
        xform.m[14] = (near * far) / nf;
        xform
    }

    pub fn orthographic(left: f64, right: f64, bottom: f64, top: f64, near: f64, far: f64) -> Self {
        let rl = right - left;
        let tb = top - bottom;
        let nf = near - far;
        let mut xform = Xform::new();
        xform.m = [0.0; 16];
        xform.m[0] = 2.0 / rl;
        xform.m[5] = 2.0 / tb;
        xform.m[10] = 1.0 / nf;
        xform.m[12] = (left + right) / (left - right);
        xform.m[13] = (bottom + top) / (bottom - top);
        xform.m[14] = near / nf;
        xform.m[15] = 1.0;
        xform
    }

    pub fn project_to_plane(plane: &Plane) -> Self {
        let n = plane.z_axis();
        let o = plane.origin();
        let (nx, ny, nz) = (n[0], n[1], n[2]);
        let d = o[0] * nx + o[1] * ny + o[2] * nz;
        let mut xform = Xform::new();
        xform.m[0]  = 1.0 - nx * nx;  xform.m[4]  = -nx * ny;        xform.m[8]  = -nx * nz;        xform.m[12] = nx * d;
        xform.m[1]  = -ny * nx;       xform.m[5]  = 1.0 - ny * ny;   xform.m[9]  = -ny * nz;        xform.m[13] = ny * d;
        xform.m[2]  = -nz * nx;       xform.m[6]  = -nz * ny;        xform.m[10] = 1.0 - nz * nz;   xform.m[14] = nz * d;
        xform.m[3]  = 0.0;            xform.m[7]  = 0.0;             xform.m[11] = 0.0;              xform.m[15] = 1.0;
        xform
    }

    pub fn project_to_plane_by_axis(plane: &Plane, direction: &Vector) -> Self {
        let n = plane.z_axis();
        let o = plane.origin();
        let (nx, ny, nz) = (n[0], n[1], n[2]);
        let (dx, dy, dz) = (direction[0], direction[1], direction[2]);
        let dot_nd = nx * dx + ny * dy + nz * dz;
        let s = 1.0 / dot_nd;
        let d = o[0] * nx + o[1] * ny + o[2] * nz;
        let mut xform = Xform::new();
        xform.m[0]  = 1.0 - dx*s*nx;  xform.m[4]  = -dx*s*ny;        xform.m[8]  = -dx*s*nz;        xform.m[12] = dx*s*d;
        xform.m[1]  = -dy*s*nx;       xform.m[5]  = 1.0 - dy*s*ny;   xform.m[9]  = -dy*s*nz;        xform.m[13] = dy*s*d;
        xform.m[2]  = -dz*s*nx;       xform.m[6]  = -dz*s*ny;        xform.m[10] = 1.0 - dz*s*nz;   xform.m[14] = dz*s*d;
        xform.m[3]  = 0.0;            xform.m[7]  = 0.0;             xform.m[11] = 0.0;              xform.m[15] = 1.0;
        xform
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Details
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn inverse(&self) -> Option<Xform> {
        let a00 = self[(0, 0)];
        let a01 = self[(0, 1)];
        let a02 = self[(0, 2)];
        let a10 = self[(1, 0)];
        let a11 = self[(1, 1)];
        let a12 = self[(1, 2)];
        let a20 = self[(2, 0)];
        let a21 = self[(2, 1)];
        let a22 = self[(2, 2)];

        let det = a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20)
            + a02 * (a10 * a21 - a11 * a20);
        if det.abs() < 1e-12 {
            return None;
        }
        let inv_det = 1.0 / det;

        let m00 = (a11 * a22 - a12 * a21) * inv_det;
        let m01 = (a02 * a21 - a01 * a22) * inv_det;
        let m02 = (a01 * a12 - a02 * a11) * inv_det;
        let m10 = (a12 * a20 - a10 * a22) * inv_det;
        let m11 = (a00 * a22 - a02 * a20) * inv_det;
        let m12 = (a02 * a10 - a00 * a12) * inv_det;
        let m20 = (a10 * a21 - a11 * a20) * inv_det;
        let m21 = (a01 * a20 - a00 * a21) * inv_det;
        let m22 = (a00 * a11 - a01 * a10) * inv_det;

        let tx = self[(0, 3)];
        let ty = self[(1, 3)];
        let tz = self[(2, 3)];
        let itx = -(m00 * tx + m01 * ty + m02 * tz);
        let ity = -(m10 * tx + m11 * ty + m12 * tz);
        let itz = -(m20 * tx + m21 * ty + m22 * tz);

        let mut res = Xform::identity();
        res[(0, 0)] = m00;
        res[(0, 1)] = m01;
        res[(0, 2)] = m02;
        res[(1, 0)] = m10;
        res[(1, 1)] = m11;
        res[(1, 2)] = m12;
        res[(2, 0)] = m20;
        res[(2, 1)] = m21;
        res[(2, 2)] = m22;
        res[(0, 3)] = itx;
        res[(1, 3)] = ity;
        res[(2, 3)] = itz;
        Some(res)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Apply Transformations
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn x(&self) -> Vector {
        Vector::new(self.m[0], self.m[1], self.m[2])
    }

    pub fn y(&self) -> Vector {
        Vector::new(self.m[4], self.m[5], self.m[6])
    }

    pub fn z(&self) -> Vector {
        Vector::new(self.m[8], self.m[9], self.m[10])
    }

    pub fn is_identity(&self) -> bool {
        let identity = Xform::identity();
        for i in 0..16 {
            if (self.m[i] - identity.m[i]).abs() > 1e-10 {
                return false;
            }
        }
        true
    }

    #[allow(clippy::too_many_arguments)]
    pub fn change_basis(
        origin_1: &Point,
        x_axis_1: &Vector,
        y_axis_1: &Vector,
        z_axis_1: &Vector,
        origin_0: &Point,
        x_axis_0: &Vector,
        y_axis_0: &Vector,
        z_axis_0: &Vector,
    ) -> Self {
        let a = x_axis_1.dot(y_axis_1);
        let b = x_axis_1.dot(z_axis_1);
        let c = y_axis_1.dot(z_axis_1);

        let mut r = [
            [
                x_axis_1.dot(x_axis_1),
                a,
                b,
                x_axis_1.dot(x_axis_0),
                x_axis_1.dot(y_axis_0),
                x_axis_1.dot(z_axis_0),
            ],
            [
                a,
                y_axis_1.dot(y_axis_1),
                c,
                y_axis_1.dot(x_axis_0),
                y_axis_1.dot(y_axis_0),
                y_axis_1.dot(z_axis_0),
            ],
            [
                b,
                c,
                z_axis_1.dot(z_axis_1),
                z_axis_1.dot(x_axis_0),
                z_axis_1.dot(y_axis_0),
                z_axis_1.dot(z_axis_0),
            ],
        ];

        let mut i0 = if r[0][0] >= r[1][1] { 0 } else { 1 };
        if r[2][2] > r[i0][i0] {
            i0 = 2;
        }
        let i1 = (i0 + 1) % 3;
        let i2 = (i1 + 1) % 3;

        if r[i0][i0] == 0.0 {
            return Self::identity();
        }

        let d = 1.0 / r[i0][i0];
        for j in 0..6 {
            r[i0][j] *= d;
        }
        r[i0][i0] = 1.0;

        if r[i1][i0] != 0.0 {
            let d = -r[i1][i0];
            for j in 0..6 {
                r[i1][j] += d * r[i0][j];
            }
            r[i1][i0] = 0.0;
        }
        if r[i2][i0] != 0.0 {
            let d = -r[i2][i0];
            for j in 0..6 {
                r[i2][j] += d * r[i0][j];
            }
            r[i2][i0] = 0.0;
        }

        let (i1, i2) = if r[i1][i1].abs() < r[i2][i2].abs() {
            (i2, i1)
        } else {
            (i1, i2)
        };
        if r[i1][i1] == 0.0 {
            return Self::identity();
        }

        let d = 1.0 / r[i1][i1];
        for j in 0..6 {
            r[i1][j] *= d;
        }
        r[i1][i1] = 1.0;

        if r[i0][i1] != 0.0 {
            let d = -r[i0][i1];
            for j in 0..6 {
                r[i0][j] += d * r[i1][j];
            }
            r[i0][i1] = 0.0;
        }
        if r[i2][i1] != 0.0 {
            let d = -r[i2][i1];
            for j in 0..6 {
                r[i2][j] += d * r[i1][j];
            }
            r[i2][i1] = 0.0;
        }

        if r[i2][i2] == 0.0 {
            return Self::identity();
        }

        let d = 1.0 / r[i2][i2];
        for j in 0..6 {
            r[i2][j] *= d;
        }
        r[i2][i2] = 1.0;

        if r[i0][i2] != 0.0 {
            let d = -r[i0][i2];
            for j in 0..6 {
                r[i0][j] += d * r[i2][j];
            }
            r[i0][i2] = 0.0;
        }
        if r[i1][i2] != 0.0 {
            let d = -r[i1][i2];
            for j in 0..6 {
                r[i1][j] += d * r[i2][j];
            }
            r[i1][i2] = 0.0;
        }

        let mut m_xform = Self::identity();
        m_xform.m[0] = r[0][3];
        m_xform.m[4] = r[0][4];
        m_xform.m[8] = r[0][5];
        m_xform.m[1] = r[1][3];
        m_xform.m[5] = r[1][4];
        m_xform.m[9] = r[1][5];
        m_xform.m[2] = r[2][3];
        m_xform.m[6] = r[2][4];
        m_xform.m[10] = r[2][5];

        let t0 = Self::translation(-origin_1[0], -origin_1[1], -origin_1[2]);
        let t2 = Self::translation(origin_0[0], origin_0[1], origin_0[2]);
        &t2 * &(&m_xform * &t0)
    }

    /// Build the change-of-basis xform from two 4-point joint volume rectangles.
    ///
    /// Maps the unit cube `[-0.5, +0.5]^3` to the world frame defined by the
    /// two rectangles. Verbatim port of the inline `change_basis` helper from
    /// main_5.cpp:782 (which mirrored wood `wood_joint.cpp:103`).
    ///
    /// Returns `Xform::identity()` if the rectangle is degenerate.
    pub fn from_change_of_basis(rect0: &crate::polyline::Polyline, rect1: &crate::polyline::Polyline) -> Self {
        if rect0.point_count() < 4 || rect1.point_count() < 1 {
            return Xform::identity();
        }

        let o1x = -0.5_f64;
        let o1y = -0.5_f64;
        let o1z = -0.5_f64;

        let o0 = rect0.get_point(0).unwrap();
        let r01 = rect0.get_point(1).unwrap();
        let r03 = rect0.get_point(3).unwrap();
        let r10 = rect1.get_point(0).unwrap();
        let x0 = [r01[0] - o0[0], r01[1] - o0[1], r01[2] - o0[2]];
        let y0 = [r03[0] - o0[0], r03[1] - o0[1], r03[2] - o0[2]];
        let z0 = [r10[0] - o0[0], r10[1] - o0[1], r10[2] - o0[2]];

        // Augmented matrix [I | dot(Xi, X0j)] (X1, Y1, Z1 are orthonormal so
        // their Gram is the identity).
        let mut r: [[f64; 6]; 3] = [
            [1.0, 0.0, 0.0, x0[0], y0[0], z0[0]],
            [0.0, 1.0, 0.0, x0[1], y0[1], z0[1]],
            [0.0, 0.0, 1.0, x0[2], y0[2], z0[2]],
        ];

        let mut i0 = if r[0][0] >= r[1][1] { 0 } else { 1 };
        if r[2][2] > r[i0][i0] {
            i0 = 2;
        }
        let mut i1 = (i0 + 1) % 3;
        let mut i2 = (i1 + 1) % 3;
        if r[i0][i0] == 0.0 {
            return Xform::identity();
        }

        // Inline elimination/normalization (closures would borrow r mutably).
        macro_rules! elim {
            ($pivot:expr, $target:expr) => {{
                let pv: usize = $pivot;
                let tg: usize = $target;
                if r[tg][pv] != 0.0 {
                    let dd = -r[tg][pv];
                    for k in 0..6 {
                        r[tg][k] += dd * r[pv][k];
                    }
                    r[tg][pv] = 0.0;
                }
            }};
        }
        macro_rules! norm_row {
            ($row:expr) => {{
                let rw: usize = $row;
                let dd = 1.0 / r[rw][rw];
                for k in 0..6 {
                    r[rw][k] *= dd;
                }
                r[rw][rw] = 1.0;
            }};
        }

        norm_row!(i0); elim!(i0, i1); elim!(i0, i2);
        if r[i1][i1].abs() < r[i2][i2].abs() {
            std::mem::swap(&mut i1, &mut i2);
        }
        if r[i1][i1] == 0.0 {
            return Xform::identity();
        }
        norm_row!(i1); elim!(i1, i0); elim!(i1, i2);
        if r[i2][i2] == 0.0 {
            return Xform::identity();
        }
        norm_row!(i2); elim!(i2, i0); elim!(i2, i1);

        let tx = o0[0] - (r[0][3]*o1x + r[0][4]*o1y + r[0][5]*o1z);
        let ty = o0[1] - (r[1][3]*o1x + r[1][4]*o1y + r[1][5]*o1z);
        let tz = o0[2] - (r[2][3]*o1x + r[2][4]*o1y + r[2][5]*o1z);
        Xform {
            typ: "Xform".to_string(),
            guid: std::sync::OnceLock::new(),
            name: "my_xform".to_string(),
            m: [
                r[0][3], r[1][3], r[2][3], 0.0,
                r[0][4], r[1][4], r[2][4], 0.0,
                r[0][5], r[1][5], r[2][5], 0.0,
                tx,      ty,      tz,      1.0,
            ],
        }
    }

    /// Transform mapping one plane to another.
    pub fn plane_to_plane(plane_from: &Plane, plane_to: &Plane) -> Self {
        let mut x0 = plane_from.x_axis();
        let mut y0 = plane_from.y_axis();
        let mut z0 = plane_from.z_axis();
        let mut x1 = plane_to.x_axis();
        let mut y1 = plane_to.y_axis();
        let mut z1 = plane_to.z_axis();
        x0.normalize_self();
        y0.normalize_self();
        z0.normalize_self();
        x1.normalize_self();
        y1.normalize_self();
        z1.normalize_self();

        let origin_0 = plane_from.origin();
        let origin_1 = plane_to.origin();

        let t0 = Self::translation(-origin_0[0], -origin_0[1], -origin_0[2]);

        let mut f0 = Self::identity();
        f0.m[0] = x0[0];
        f0.m[1] = x0[1];
        f0.m[2] = x0[2];
        f0.m[4] = y0[0];
        f0.m[5] = y0[1];
        f0.m[6] = y0[2];
        f0.m[8] = z0[0];
        f0.m[9] = z0[1];
        f0.m[10] = z0[2];

        let mut f1 = Self::identity();
        f1.m[0] = x1[0];
        f1.m[4] = x1[1];
        f1.m[8] = x1[2];
        f1.m[1] = y1[0];
        f1.m[5] = y1[1];
        f1.m[9] = y1[2];
        f1.m[2] = z1[0];
        f1.m[6] = z1[1];
        f1.m[10] = z1[2];

        let r = &f1 * &f0;
        let t1 = Self::translation(origin_1[0], origin_1[1], origin_1[2]);
        &t1 * &(&r * &t0)
    }

    pub fn plane_to_xy(origin: &Point, x_axis: &Vector, y_axis: &Vector, z_axis: &Vector) -> Self {
        let mut x = x_axis.clone();
        let mut y = y_axis.clone();
        let mut z = z_axis.clone();
        x.normalize_self();
        y.normalize_self();
        z.normalize_self();

        let t = Self::translation(-origin[0], -origin[1], -origin[2]);
        let mut f = Self::identity();
        f.m[0] = x[0];
        f.m[1] = x[1];
        f.m[2] = x[2];
        f.m[4] = y[0];
        f.m[5] = y[1];
        f.m[6] = y[2];
        f.m[8] = z[0];
        f.m[9] = z[1];
        f.m[10] = z[2];
        &f * &t
    }

    pub fn xy_to_plane(origin: &Point, x_axis: &Vector, y_axis: &Vector, z_axis: &Vector) -> Self {
        let mut x = x_axis.clone();
        let mut y = y_axis.clone();
        let mut z = z_axis.clone();
        x.normalize_self();
        y.normalize_self();
        z.normalize_self();

        let mut f = Self::identity();
        f.m[0] = x[0];
        f.m[4] = y[0];
        f.m[8] = z[0];
        f.m[1] = x[1];
        f.m[5] = y[1];
        f.m[9] = z[1];
        f.m[2] = x[2];
        f.m[6] = y[2];
        f.m[10] = z[2];

        let t = Self::translation(origin[0], origin[1], origin[2]);
        &t * &f
    }

    /// Transform world points INTO a local frame defined by (origin, x, y, z).
    /// Given a world point p, returns (u, v, w) such that
    /// `p = origin + u*x_hat + v*y_hat + w*z_hat`. Stores the basis as matrix
    /// ROWS (world-to-local), unlike `plane_to_xy` which stores them as
    /// COLUMNS and therefore actually does local-to-world despite its name.
    /// Use this when you need a faithful 3D projection — especially when the
    /// input geometry's normal can align with one of the basis axes (where
    /// `plane_to_xy` collapses a dimension). See wood_main.cpp type-13 branch.
    pub fn world_to_frame(origin: &Point, x_axis: &Vector, y_axis: &Vector, z_axis: &Vector) -> Self {
        let mut x = x_axis.clone();
        let mut y = y_axis.clone();
        let mut z = z_axis.clone();
        x.normalize_self();
        y.normalize_self();
        z.normalize_self();

        let mut f = Self::identity();
        f.m[0] = x[0]; f.m[4] = x[1]; f.m[8]  = x[2];
        f.m[1] = y[0]; f.m[5] = y[1]; f.m[9]  = y[2];
        f.m[2] = z[0]; f.m[6] = z[1]; f.m[10] = z[2];

        let t = Self::translation(-origin[0], -origin[1], -origin[2]);
        &f * &t
    }

    /// Inverse of `world_to_frame`: local (u,v,w) -> world point at
    /// `origin + u*x_hat + v*y_hat + w*z_hat`.
    pub fn frame_to_world(origin: &Point, x_axis: &Vector, y_axis: &Vector, z_axis: &Vector) -> Self {
        let mut x = x_axis.clone();
        let mut y = y_axis.clone();
        let mut z = z_axis.clone();
        x.normalize_self();
        y.normalize_self();
        z.normalize_self();

        let mut f = Self::identity();
        f.m[0] = x[0]; f.m[1] = x[1]; f.m[2]  = x[2];
        f.m[4] = y[0]; f.m[5] = y[1]; f.m[6]  = y[2];
        f.m[8] = z[0]; f.m[9] = z[1]; f.m[10] = z[2];

        let t = Self::translation(origin[0], origin[1], origin[2]);
        &t * &f
    }

    /// Transform from world XY to target frame/plane (same as COMPAS from_frame)
    pub fn to_frame(frame: &Plane) -> Self {
        let x = frame.x_axis().normalized();
        let y = frame.y_axis().normalized();
        let z = frame.z_axis().normalized();
        let o = frame.origin();

        let mut xf = Self::identity();
        xf.m[0] = x[0]; xf.m[4] = y[0]; xf.m[8]  = z[0]; xf.m[12] = o[0];
        xf.m[1] = x[1]; xf.m[5] = y[1]; xf.m[9]  = z[1]; xf.m[13] = o[1];
        xf.m[2] = x[2]; xf.m[6] = y[2]; xf.m[10] = z[2]; xf.m[14] = o[2];
        xf.m[3] = 0.0;  xf.m[7] = 0.0;  xf.m[11] = 0.0;  xf.m[15] = 1.0;
        xf
    }

    pub fn scale_xyz(scale_x: f64, scale_y: f64, scale_z: f64) -> Self {
        let mut xform = Self::identity();
        xform.m[0] = scale_x;
        xform.m[5] = scale_y;
        xform.m[10] = scale_z;
        xform
    }

    pub fn scale_uniform(origin: &Point, scale_value: f64) -> Self {
        let t0 = Self::translation(-origin[0], -origin[1], -origin[2]);
        let t1 = Self::scale_xyz(scale_value, scale_value, scale_value);
        let t2 = Self::translation(origin[0], origin[1], origin[2]);
        &t2 * &(&t1 * &t0)
    }

    pub fn scale_non_uniform(origin: &Point, scale_x: f64, scale_y: f64, scale_z: f64) -> Self {
        let t0 = Self::translation(-origin[0], -origin[1], -origin[2]);
        let t1 = Self::scale_xyz(scale_x, scale_y, scale_z);
        let t2 = Self::translation(origin[0], origin[1], origin[2]);
        &t2 * &(&t1 * &t0)
    }

    pub fn axis_rotation(angle: f64, axis: &Vector, degrees: bool) -> Self {
        let angle = if degrees { angle * Tolerance::TO_RADIANS } else { angle };
        let c = angle.cos();
        let s = angle.sin();
        let ux = axis[0];
        let uy = axis[1];
        let uz = axis[2];
        let t = 1.0 - c;

        let mut xform = Self::identity();
        xform.m[0] = t * ux * ux + c;
        xform.m[4] = t * ux * uy - uz * s;
        xform.m[8] = t * ux * uz + uy * s;

        xform.m[1] = t * ux * uy + uz * s;
        xform.m[5] = t * uy * uy + c;
        xform.m[9] = t * uy * uz - ux * s;

        xform.m[2] = t * ux * uz - uy * s;
        xform.m[6] = t * uy * uz + ux * s;
        xform.m[10] = t * uz * uz + c;

        xform
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

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
        Self::jsonload(json_string).unwrap_or_else(|_| Self::default())
    }

    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to protobuf binary format.
    ///
    /// # Returns
    ///
    /// A Vec<u8> containing the serialized protobuf data.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        let proto = crate::proto::Xform {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            matrix: self.m.to_vec(),
        };
        proto.encode_to_vec()
    }

    /// Create Xform from protobuf binary data.
    ///
    /// # Arguments
    ///
    /// * `data` - Byte slice containing protobuf-encoded xform data.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized Xform or an error.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        let proto = crate::proto::Xform::decode(data)?;

        let mut xform = Self::identity();
        xform.set_guid(proto.guid);
        xform.name = proto.name;
        for (i, val) in proto.matrix.iter().enumerate() {
            if i < 16 {
                xform.m[i] = *val;
            }
        }
        Ok(xform)
    }

    /// Write protobuf to file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the output file.
    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    /// Read protobuf from file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the protobuf file.
    ///
    /// # Returns
    ///
    /// The deserialized Xform.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representations
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Minimal string representation (matrix rows)
    pub fn str(&self) -> String {
        let mut rows = Vec::new();
        for i in 0..4 {
            rows.push(format!(
                "[{:.6}, {:.6}, {:.6}, {:.6}]",
                self.m[i],
                self.m[4 + i],
                self.m[8 + i],
                self.m[12 + i]
            ));
        }
        rows.join("\n")
    }

    /// Full string representation (name and guid prefix)
    pub fn repr(&self) -> String {
        format!("Xform({}, {})", self.name, &self.guid()[..8])
    }

    /// Create a copy with a new GUID
    pub fn duplicate(&self) -> Self {
        let mut copy = Self::from_matrix(self.m);
        copy.name = self.name.clone();
        copy
    }
}

// Implement Display for Xform (compact 4x4 matrix)
impl fmt::Display for Xform {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "[{:7.3}, {:7.3}, {:7.3}, {:7.3}]",
            self.m[0], self.m[4], self.m[8], self.m[12]
        )?;
        writeln!(
            f,
            "[{:7.3}, {:7.3}, {:7.3}, {:7.3}]",
            self.m[1], self.m[5], self.m[9], self.m[13]
        )?;
        writeln!(
            f,
            "[{:7.3}, {:7.3}, {:7.3}, {:7.3}]",
            self.m[2], self.m[6], self.m[10], self.m[14]
        )?;
        write!(
            f,
            "[{:7.3}, {:7.3}, {:7.3}, {:7.3}]",
            self.m[3], self.m[7], self.m[11], self.m[15]
        )
    }
}

// Implement Debug for Xform (full representation with all 16 values)
impl fmt::Debug for Xform {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let vals: Vec<String> = self.m.iter().map(|v| format!("{:.3}", v)).collect();
        write!(
            f,
            "Xform(name='{}', matrix=[{}])",
            self.name,
            vals.join(", ")
        )
    }
}

/// Implement Default for Xform to return identity matrix
impl Default for Xform {
    fn default() -> Self {
        Self::identity()
    }
}

/// Custom PartialEq that compares only matrix values (with tolerance), ignoring guid and name
impl PartialEq for Xform {
    fn eq(&self, other: &Self) -> bool {
        for i in 0..16 {
            if (self.m[i] - other.m[i]).abs() > 1e-10 {
                return false;
            }
        }
        true
    }
}

impl Eq for Xform {}

// Implement Index trait for accessing matrix elements with [(row, col)] syntax
impl Index<(usize, usize)> for Xform {
    type Output = f64;

    fn index(&self, idx: (usize, usize)) -> &Self::Output {
        let (row, col) = idx;
        assert!(row < 4 && col < 4, "Index out of bounds: ({row}, {col})");
        // Column-major order: index = col * 4 + row
        &self.m[col * 4 + row]
    }
}

// Implement IndexMut trait for modifying matrix elements with [(row, col)] syntax
impl IndexMut<(usize, usize)> for Xform {
    fn index_mut(&mut self, idx: (usize, usize)) -> &mut Self::Output {
        let (row, col) = idx;
        assert!(row < 4 && col < 4, "Index out of bounds: ({row}, {col})");
        // Column-major order: index = col * 4 + row
        &mut self.m[col * 4 + row]
    }
}

// Implement Mul for matrix multiplication: Xform * Xform = Xform
impl Mul for &Xform {
    type Output = Xform;

    fn mul(self, rhs: &Xform) -> Self::Output {
        let mut result = Xform::identity();

        for i in 0..4 {
            for j in 0..4 {
                let mut sum = 0.0;
                for k in 0..4 {
                    // self[i,k] * rhs[k,j]
                    sum += self[(i, k)] * rhs[(k, j)];
                }
                result[(i, j)] = sum;
            }
        }

        result
    }
}

// Implement Mul for owned matrices
impl Mul for Xform {
    type Output = Xform;

    fn mul(self, rhs: Xform) -> Self::Output {
        &self * &rhs
    }
}

// Implement MulAssign for in-place matrix multiplication: xform *= other_xform
impl MulAssign for Xform {
    fn mul_assign(&mut self, rhs: Self) {
        *self = &*self * &rhs;
    }
}

#[cfg(test)]
#[path = "xform_test.rs"]
mod tests;
