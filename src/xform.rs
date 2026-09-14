use crate::tolerance::Tolerance;
use crate::{Line, Plane, Point, Polyline, Vector};
use serde::ser::SerializeMap;
use serde::{Deserialize, Deserializer, Serializer};
use std::fmt;
use std::ops::{Index, IndexMut, Mul, MulAssign};
use std::sync::OnceLock;

/// A 4x4 column-major transformation matrix
#[derive(Clone)]
pub struct Xform {
    guid: OnceLock<String>,
    pub name: String,
    pub m: [f64; 16],
}

impl Xform {
    /// Identity
    pub fn new() -> Self {
        Xform {
            guid: OnceLock::new(),
            name: "my_xform".to_string(),
            m: [
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
        }
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

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn identity() -> Self {
        Self::new()
    }

    pub fn from_matrix(matrix: [f64; 16]) -> Self {
        let mut xform = Self::new();
        xform.m = matrix;
        xform
    }

    /// Copy (new guid, same data)
    pub fn duplicate(&self) -> Self {
        let mut copy = Self::from_matrix(self.m);
        copy.name = self.name.clone();
        copy
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformations
    // ═══════════════════════════════════════════════════════════════════════════

    /// Pure rotation from three column axis vectors
    pub fn from_axes(col_x: &Vector, col_y: &Vector, col_z: &Vector) -> Self {
        let mut xform = Self::new();
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

    pub fn translation(x: f64, y: f64, z: f64) -> Self {
        let mut xform = Self::new();
        xform.m[12] = x;
        xform.m[13] = y;
        xform.m[14] = z;
        xform
    }

    pub fn rotation_x(angle: f64, degrees: bool) -> Self {
        let angle = if degrees {
            angle * Tolerance::TO_RADIANS
        } else {
            angle
        };
        let mut xform = Self::new();
        let cos_angle = angle.cos();
        let sin_angle = angle.sin();
        xform.m[5] = cos_angle;
        xform.m[6] = sin_angle;
        xform.m[9] = -sin_angle;
        xform.m[10] = cos_angle;
        xform
    }

    pub fn rotation_y(angle: f64, degrees: bool) -> Self {
        let angle = if degrees {
            angle * Tolerance::TO_RADIANS
        } else {
            angle
        };
        let mut xform = Self::new();
        let cos_angle = angle.cos();
        let sin_angle = angle.sin();
        xform.m[0] = cos_angle;
        xform.m[2] = -sin_angle;
        xform.m[8] = sin_angle;
        xform.m[10] = cos_angle;
        xform
    }

    pub fn rotation_z(angle: f64, degrees: bool) -> Self {
        let angle = if degrees {
            angle * Tolerance::TO_RADIANS
        } else {
            angle
        };
        let mut xform = Self::new();
        let cos_angle = angle.cos();
        let sin_angle = angle.sin();
        xform.m[0] = cos_angle;
        xform.m[1] = sin_angle;
        xform.m[4] = -sin_angle;
        xform.m[5] = cos_angle;
        xform
    }

    pub fn rotation(axis: &Vector, angle: f64, degrees: bool) -> Self {
        let angle = if degrees {
            angle * Tolerance::TO_RADIANS
        } else {
            angle
        };
        let mut xform = Self::new();
        let unit = axis.normalized();
        let cos_angle = angle.cos();
        let sin_angle = angle.sin();
        let one_minus_cos = 1.0 - cos_angle;
        let xx = unit[0] * unit[0];
        let xy = unit[0] * unit[1];
        let xz = unit[0] * unit[2];
        let yy = unit[1] * unit[1];
        let yz = unit[1] * unit[2];
        let zz = unit[2] * unit[2];
        xform.m[0] = cos_angle + xx * one_minus_cos;
        xform.m[1] = xy * one_minus_cos + unit[2] * sin_angle;
        xform.m[2] = xz * one_minus_cos - unit[1] * sin_angle;
        xform.m[4] = xy * one_minus_cos - unit[2] * sin_angle;
        xform.m[5] = cos_angle + yy * one_minus_cos;
        xform.m[6] = yz * one_minus_cos + unit[0] * sin_angle;
        xform.m[8] = xz * one_minus_cos + unit[1] * sin_angle;
        xform.m[9] = yz * one_minus_cos - unit[0] * sin_angle;
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

    /// Change of basis from frame 1 to frame 0
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
        let mut i1 = (i0 + 1) % 3;
        let mut i2 = (i1 + 1) % 3;
        if r[i0][i0] == 0.0 {
            return Self::identity();
        }

        let mut d = 1.0 / r[i0][i0];
        for j in 0..6 {
            r[i0][j] *= d;
        }
        r[i0][i0] = 1.0;
        if r[i1][i0] != 0.0 {
            d = -r[i1][i0];
            for j in 0..6 {
                r[i1][j] += d * r[i0][j];
            }
            r[i1][i0] = 0.0;
        }
        if r[i2][i0] != 0.0 {
            d = -r[i2][i0];
            for j in 0..6 {
                r[i2][j] += d * r[i0][j];
            }
            r[i2][i0] = 0.0;
        }

        if r[i1][i1].abs() < r[i2][i2].abs() {
            std::mem::swap(&mut i1, &mut i2);
        }
        if r[i1][i1] == 0.0 {
            return Self::identity();
        }

        d = 1.0 / r[i1][i1];
        for j in 0..6 {
            r[i1][j] *= d;
        }
        r[i1][i1] = 1.0;
        if r[i0][i1] != 0.0 {
            d = -r[i0][i1];
            for j in 0..6 {
                r[i0][j] += d * r[i1][j];
            }
            r[i0][i1] = 0.0;
        }
        if r[i2][i1] != 0.0 {
            d = -r[i2][i1];
            for j in 0..6 {
                r[i2][j] += d * r[i1][j];
            }
            r[i2][i1] = 0.0;
        }

        if r[i2][i2] == 0.0 {
            return Self::identity();
        }

        d = 1.0 / r[i2][i2];
        for j in 0..6 {
            r[i2][j] *= d;
        }
        r[i2][i2] = 1.0;
        if r[i0][i2] != 0.0 {
            d = -r[i0][i2];
            for j in 0..6 {
                r[i0][j] += d * r[i2][j];
            }
            r[i0][i2] = 0.0;
        }
        if r[i1][i2] != 0.0 {
            d = -r[i1][i2];
            for j in 0..6 {
                r[i1][j] += d * r[i2][j];
            }
            r[i1][i2] = 0.0;
        }

        let mut m_xform = Self::new();
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
        t2 * (m_xform * t0)
    }

    /// Unit cube [-0.5, 0.5]^3 to the joint volume frame spanned by rect0 (x, y) and rect1[0] (z)
    pub fn from_change_of_basis(rect0: &Polyline, rect1: &Polyline) -> Self {
        if rect0.point_count() < 4 || rect1.point_count() < 1 {
            return Self::identity();
        }
        let origin_1 = Point::new(-0.5, -0.5, -0.5);
        let x_axis_1 = Vector::new(1.0, 0.0, 0.0);
        let y_axis_1 = Vector::new(0.0, 1.0, 0.0);
        let z_axis_1 = Vector::new(0.0, 0.0, 1.0);
        let origin_0 = rect0.get_point(0).unwrap();
        let x_axis_0 = &rect0.get_point(1).unwrap() - &origin_0;
        let y_axis_0 = &rect0.get_point(3).unwrap() - &origin_0;
        let z_axis_0 = &rect1.get_point(0).unwrap() - &origin_0;
        Self::change_basis(
            &origin_1, &x_axis_1, &y_axis_1, &z_axis_1, &origin_0, &x_axis_0, &y_axis_0, &z_axis_0,
        )
    }

    pub fn plane_to_plane(plane_from: &Plane, plane_to: &Plane) -> Self {
        let x0 = plane_from.x_axis().normalized();
        let y0 = plane_from.y_axis().normalized();
        let z0 = plane_from.z_axis().normalized();
        let x1 = plane_to.x_axis().normalized();
        let y1 = plane_to.y_axis().normalized();
        let z1 = plane_to.z_axis().normalized();
        let origin_0 = plane_from.origin();
        let origin_1 = plane_to.origin();

        let t0 = Self::translation(-origin_0[0], -origin_0[1], -origin_0[2]);
        let mut f0 = Self::new();
        f0.m[0] = x0[0];
        f0.m[1] = x0[1];
        f0.m[2] = x0[2];
        f0.m[4] = y0[0];
        f0.m[5] = y0[1];
        f0.m[6] = y0[2];
        f0.m[8] = z0[0];
        f0.m[9] = z0[1];
        f0.m[10] = z0[2];
        let mut f1 = Self::new();
        f1.m[0] = x1[0];
        f1.m[4] = x1[1];
        f1.m[8] = x1[2];
        f1.m[1] = y1[0];
        f1.m[5] = y1[1];
        f1.m[9] = y1[2];
        f1.m[2] = z1[0];
        f1.m[6] = z1[1];
        f1.m[10] = z1[2];
        let r = f1 * f0;
        let t1 = Self::translation(origin_1[0], origin_1[1], origin_1[2]);
        t1 * (r * t0)
    }

    /// Frame axes as columns, minus origin (local-to-world despite the name)
    pub fn plane_to_xy(origin: &Point, x_axis: &Vector, y_axis: &Vector, z_axis: &Vector) -> Self {
        let x = x_axis.normalized();
        let y = y_axis.normalized();
        let z = z_axis.normalized();
        let t = Self::translation(-origin[0], -origin[1], -origin[2]);
        let mut f = Self::new();
        f.m[0] = x[0];
        f.m[1] = x[1];
        f.m[2] = x[2];
        f.m[4] = y[0];
        f.m[5] = y[1];
        f.m[6] = y[2];
        f.m[8] = z[0];
        f.m[9] = z[1];
        f.m[10] = z[2];
        f * t
    }

    pub fn xy_to_plane(origin: &Point, x_axis: &Vector, y_axis: &Vector, z_axis: &Vector) -> Self {
        let x = x_axis.normalized();
        let y = y_axis.normalized();
        let z = z_axis.normalized();
        let mut f = Self::new();
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
        t * f
    }

    /// World point to frame coordinates (axes as rows)
    pub fn world_to_frame(
        origin: &Point,
        x_axis: &Vector,
        y_axis: &Vector,
        z_axis: &Vector,
    ) -> Self {
        let x = x_axis.normalized();
        let y = y_axis.normalized();
        let z = z_axis.normalized();
        let mut f = Self::new();
        f.m[0] = x[0];
        f.m[4] = x[1];
        f.m[8] = x[2];
        f.m[1] = y[0];
        f.m[5] = y[1];
        f.m[9] = y[2];
        f.m[2] = z[0];
        f.m[6] = z[1];
        f.m[10] = z[2];
        let t = Self::translation(-origin[0], -origin[1], -origin[2]);
        f * t
    }

    /// Frame coordinates to world point (axes as columns)
    pub fn frame_to_world(
        origin: &Point,
        x_axis: &Vector,
        y_axis: &Vector,
        z_axis: &Vector,
    ) -> Self {
        let x = x_axis.normalized();
        let y = y_axis.normalized();
        let z = z_axis.normalized();
        let mut f = Self::new();
        f.m[0] = x[0];
        f.m[1] = x[1];
        f.m[2] = x[2];
        f.m[4] = y[0];
        f.m[5] = y[1];
        f.m[6] = y[2];
        f.m[8] = z[0];
        f.m[9] = z[1];
        f.m[10] = z[2];
        let t = Self::translation(origin[0], origin[1], origin[2]);
        t * f
    }

    /// World XY to the frame plane (COMPAS from_frame)
    pub fn to_frame(frame: &Plane) -> Self {
        let x = frame.x_axis().normalized();
        let y = frame.y_axis().normalized();
        let z = frame.z_axis().normalized();
        let o = frame.origin();
        let mut xf = Self::new();
        xf.m[0] = x[0];
        xf.m[4] = y[0];
        xf.m[8] = z[0];
        xf.m[12] = o[0];
        xf.m[1] = x[1];
        xf.m[5] = y[1];
        xf.m[9] = z[1];
        xf.m[13] = o[1];
        xf.m[2] = x[2];
        xf.m[6] = y[2];
        xf.m[10] = z[2];
        xf.m[14] = o[2];
        xf
    }

    pub fn scale_xyz(scale_x: f64, scale_y: f64, scale_z: f64) -> Self {
        let mut xform = Self::new();
        xform.m[0] = scale_x;
        xform.m[5] = scale_y;
        xform.m[10] = scale_z;
        xform
    }

    pub fn scale_uniform(origin: &Point, scale_value: f64) -> Self {
        let t0 = Self::translation(-origin[0], -origin[1], -origin[2]);
        let t1 = Self::scale_xyz(scale_value, scale_value, scale_value);
        let t2 = Self::translation(origin[0], origin[1], origin[2]);
        t2 * (t1 * t0)
    }

    pub fn scale_non_uniform(origin: &Point, scale_x: f64, scale_y: f64, scale_z: f64) -> Self {
        let t0 = Self::translation(-origin[0], -origin[1], -origin[2]);
        let t1 = Self::scale_xyz(scale_x, scale_y, scale_z);
        let t2 = Self::translation(origin[0], origin[1], origin[2]);
        t2 * (t1 * t0)
    }

    /// Rodrigues rotation about a unit axis
    pub fn axis_rotation(angle: f64, axis: &Vector, degrees: bool) -> Self {
        let angle = if degrees {
            angle * Tolerance::TO_RADIANS
        } else {
            angle
        };
        let c = angle.cos();
        let s = angle.sin();
        let ux = axis[0];
        let uy = axis[1];
        let uz = axis[2];
        let t = 1.0 - c;
        let mut xform = Self::new();
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

    /// Right-handed view matrix (camera looks down -Z, up must not be parallel to the view)
    pub fn look_at_right_handed(eye: &Point, target: &Point, up: &Vector) -> Self {
        let f = (target - eye).normalized();
        let s = f.cross(&up.normalized()).normalized();
        let u = s.cross(&f);
        let mut xform = Self::new();
        xform.m[0] = s[0];
        xform.m[4] = s[1];
        xform.m[8] = s[2];
        xform.m[1] = u[0];
        xform.m[5] = u[1];
        xform.m[9] = u[2];
        xform.m[2] = -f[0];
        xform.m[6] = -f[1];
        xform.m[10] = -f[2];
        let eye_vec = Vector::new(eye[0], eye[1], eye[2]);
        xform.m[12] = -s.dot(&eye_vec);
        xform.m[13] = -u.dot(&eye_vec);
        xform.m[14] = f.dot(&eye_vec);
        xform
    }

    pub fn look_to_right_handed(eye: &Point, direction: &Vector, up: &Vector) -> Self {
        let f = direction.normalized();
        let s = f.cross(&up.normalized()).normalized();
        let u = s.cross(&f);
        let mut xform = Self::new();
        xform.m[0] = s[0];
        xform.m[4] = s[1];
        xform.m[8] = s[2];
        xform.m[1] = u[0];
        xform.m[5] = u[1];
        xform.m[9] = u[2];
        xform.m[2] = -f[0];
        xform.m[6] = -f[1];
        xform.m[10] = -f[2];
        let eye_vec = Vector::new(eye[0], eye[1], eye[2]);
        xform.m[12] = -s.dot(&eye_vec);
        xform.m[13] = -u.dot(&eye_vec);
        xform.m[14] = f.dot(&eye_vec);
        xform
    }

    /// Right-handed projection, depth [0, 1]
    pub fn perspective(fov_y: f64, aspect: f64, near: f64, far: f64) -> Self {
        let f = 1.0 / (fov_y / 2.0).tan();
        let nf = near - far;
        let mut xform = Self::from_matrix([0.0; 16]);
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
        let mut xform = Self::from_matrix([0.0; 16]);
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
        let mut xform = Self::new();
        xform.m[0] = 1.0 - nx * nx;
        xform.m[4] = -nx * ny;
        xform.m[8] = -nx * nz;
        xform.m[12] = nx * d;
        xform.m[1] = -ny * nx;
        xform.m[5] = 1.0 - ny * ny;
        xform.m[9] = -ny * nz;
        xform.m[13] = ny * d;
        xform.m[2] = -nz * nx;
        xform.m[6] = -nz * ny;
        xform.m[10] = 1.0 - nz * nz;
        xform.m[14] = nz * d;
        xform
    }

    pub fn project_to_plane_by_axis(plane: &Plane, direction: &Vector) -> Self {
        let n = plane.z_axis();
        let o = plane.origin();
        let (nx, ny, nz) = (n[0], n[1], n[2]);
        let (dx, dy, dz) = (direction[0], direction[1], direction[2]);
        let s = 1.0 / (nx * dx + ny * dy + nz * dz);
        let d = o[0] * nx + o[1] * ny + o[2] * nz;
        let mut xform = Self::new();
        xform.m[0] = 1.0 - dx * s * nx;
        xform.m[4] = -dx * s * ny;
        xform.m[8] = -dx * s * nz;
        xform.m[12] = dx * s * d;
        xform.m[1] = -dy * s * nx;
        xform.m[5] = 1.0 - dy * s * ny;
        xform.m[9] = -dy * s * nz;
        xform.m[13] = dy * s * d;
        xform.m[2] = -dz * s * nx;
        xform.m[6] = -dz * s * ny;
        xform.m[10] = 1.0 - dz * s * nz;
        xform.m[14] = dz * s * d;
        xform
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Apply Transformations
    // ═══════════════════════════════════════════════════════════════════════════

    /// Homogeneous multiply, divides by w when projective
    pub fn transform_point(&self, p: &Point) -> Point {
        let x = self.m[0] * p[0] + self.m[4] * p[1] + self.m[8] * p[2] + self.m[12];
        let y = self.m[1] * p[0] + self.m[5] * p[1] + self.m[9] * p[2] + self.m[13];
        let z = self.m[2] * p[0] + self.m[6] * p[1] + self.m[10] * p[2] + self.m[14];
        let w = self.m[3] * p[0] + self.m[7] * p[1] + self.m[11] * p[2] + self.m[15];
        if w.abs() < 1e-12 {
            return Point::new(x, y, z);
        }
        Point::new(x / w, y / w, z / w)
    }

    /// Rotation and scale only
    pub fn transform_vector(&self, v: &Vector) -> Vector {
        let x = self.m[0] * v[0] + self.m[4] * v[1] + self.m[8] * v[2];
        let y = self.m[1] * v[0] + self.m[5] * v[1] + self.m[9] * v[2];
        let z = self.m[2] * v[0] + self.m[6] * v[1] + self.m[10] * v[2];
        Vector::new(x, y, z)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Details
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn inverse(&self) -> Option<Xform> {
        let s0 = self.m[0] * self.m[5] - self.m[1] * self.m[4];
        let s1 = self.m[0] * self.m[9] - self.m[1] * self.m[8];
        let s2 = self.m[0] * self.m[13] - self.m[1] * self.m[12];
        let s3 = self.m[4] * self.m[9] - self.m[5] * self.m[8];
        let s4 = self.m[4] * self.m[13] - self.m[5] * self.m[12];
        let s5 = self.m[8] * self.m[13] - self.m[9] * self.m[12];
        let c5 = self.m[10] * self.m[15] - self.m[11] * self.m[14];
        let c4 = self.m[6] * self.m[15] - self.m[7] * self.m[14];
        let c3 = self.m[6] * self.m[11] - self.m[7] * self.m[10];
        let c2 = self.m[2] * self.m[15] - self.m[3] * self.m[14];
        let c1 = self.m[2] * self.m[11] - self.m[3] * self.m[10];
        let c0 = self.m[2] * self.m[7] - self.m[3] * self.m[6];
        let det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
        if det.abs() < 1e-12 {
            return None;
        }
        let inv_det = 1.0 / det;
        let mut res = Self::new();
        res.m[0] = (self.m[5] * c5 - self.m[9] * c4 + self.m[13] * c3) * inv_det;
        res.m[4] = (-self.m[4] * c5 + self.m[8] * c4 - self.m[12] * c3) * inv_det;
        res.m[8] = (self.m[7] * s5 - self.m[11] * s4 + self.m[15] * s3) * inv_det;
        res.m[12] = (-self.m[6] * s5 + self.m[10] * s4 - self.m[14] * s3) * inv_det;
        res.m[1] = (-self.m[1] * c5 + self.m[9] * c2 - self.m[13] * c1) * inv_det;
        res.m[5] = (self.m[0] * c5 - self.m[8] * c2 + self.m[12] * c1) * inv_det;
        res.m[9] = (-self.m[3] * s5 + self.m[11] * s2 - self.m[15] * s1) * inv_det;
        res.m[13] = (self.m[2] * s5 - self.m[10] * s2 + self.m[14] * s1) * inv_det;
        res.m[2] = (self.m[1] * c4 - self.m[5] * c2 + self.m[13] * c0) * inv_det;
        res.m[6] = (-self.m[0] * c4 + self.m[4] * c2 - self.m[12] * c0) * inv_det;
        res.m[10] = (self.m[3] * s4 - self.m[7] * s2 + self.m[15] * s0) * inv_det;
        res.m[14] = (-self.m[2] * s4 + self.m[6] * s2 - self.m[14] * s0) * inv_det;
        res.m[3] = (-self.m[1] * c3 + self.m[5] * c1 - self.m[9] * c0) * inv_det;
        res.m[7] = (self.m[0] * c3 - self.m[4] * c1 + self.m[8] * c0) * inv_det;
        res.m[11] = (-self.m[3] * s3 + self.m[7] * s1 - self.m[11] * s0) * inv_det;
        res.m[15] = (self.m[2] * s3 - self.m[6] * s1 + self.m[10] * s0) * inv_det;
        Some(res)
    }

    pub fn is_identity(&self) -> bool {
        let identity = Self::new();
        for i in 0..16 {
            if (self.m[i] - identity.m[i]).abs() > 1e-10 {
                return false;
            }
        }
        true
    }

    /// Four columns of four rows
    pub fn to_cols(&self) -> [[f64; 4]; 4] {
        [
            [self.m[0], self.m[1], self.m[2], self.m[3]],
            [self.m[4], self.m[5], self.m[6], self.m[7]],
            [self.m[8], self.m[9], self.m[10], self.m[11]],
            [self.m[12], self.m[13], self.m[14], self.m[15]],
        ]
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

    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json)
    }

    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_else(|_| Self::new())
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        let proto = crate::proto::Xform {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            matrix: self.m.to_vec(),
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Xform::decode(data)?;
        let mut xform = Self::new();
        if !proto.guid.is_empty() {
            xform.set_guid(proto.guid);
        }
        xform.name = proto.name;
        for i in 0..proto.matrix.len().min(16) {
            xform.m[i] = proto.matrix[i];
        }
        Ok(xform)
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
    // String Representations
    // ═══════════════════════════════════════════════════════════════════════════

    /// Four matrix rows
    pub fn str(&self) -> String {
        let mut rows: Vec<String> = Vec::new();
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

    /// Name and guid prefix
    pub fn repr(&self) -> String {
        format!("Xform({}, {})", self.name, &self.guid()[..8])
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SESSION_VIEWER
    // ═══════════════════════════════════════════════════════════════════════════

    /// Column-major f32 copy for a GPU uniform
    pub fn to_f32(&self) -> [f32; 16] {
        std::array::from_fn(|i| self.m[i] as f32)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

impl Default for Xform {
    fn default() -> Self {
        Self::new()
    }
}

impl Mul for &Xform {
    type Output = Xform;
    fn mul(self, other: &Xform) -> Xform {
        let mut result = Xform::new();
        for i in 0..4 {
            for j in 0..4 {
                let mut sum = 0.0;
                for k in 0..4 {
                    sum += self.m[k * 4 + i] * other.m[j * 4 + k];
                }
                result.m[j * 4 + i] = sum;
            }
        }
        result
    }
}

impl Mul for Xform {
    type Output = Xform;
    fn mul(self, other: Xform) -> Xform {
        &self * &other
    }
}

impl MulAssign for Xform {
    fn mul_assign(&mut self, other: Xform) {
        *self = &*self * &other;
    }
}

impl Index<(usize, usize)> for Xform {
    type Output = f64;
    /// Element at (row, col)
    fn index(&self, (row, col): (usize, usize)) -> &f64 {
        assert!(row < 4 && col < 4, "Index out of bounds: ({row}, {col})");
        &self.m[col * 4 + row]
    }
}

impl IndexMut<(usize, usize)> for Xform {
    fn index_mut(&mut self, (row, col): (usize, usize)) -> &mut f64 {
        assert!(row < 4 && col < 4, "Index out of bounds: ({row}, {col})");
        &mut self.m[col * 4 + row]
    }
}

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

impl fmt::Display for Xform {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

impl fmt::Debug for Xform {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.repr())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Serde
// ═══════════════════════════════════════════════════════════════════════════

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
            #[serde(default)]
            m: Option<[f64; 16]>,
        }
        let d = XformData::deserialize(deserializer)?;
        let mut xform = Xform::new();
        if let Some(guid) = d.guid {
            xform.set_guid(guid);
        }
        if let Some(name) = d.name {
            xform.name = name;
        }
        if let Some(m) = d.m {
            xform.m = m;
        }
        Ok(xform)
    }
}
