use crate::tolerance::Tolerance;
use crate::tolerance::TOLERANCE;
use crate::{Color, Point, Polyline, Vector, Xform};
use serde::ser::SerializeMap;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::fmt;
use std::ops::{Add, AddAssign, Index, IndexMut, Sub, SubAssign};
use std::sync::OnceLock;

/// A plane defined by an origin and an orthonormal x, y, z frame
#[derive(Debug, Clone)]
pub struct Plane {
    guid: OnceLock<String>,
    pub name: String,
    pub width: f64,
    pub linecolor: Color,
    _origin: Point,
    _x_axis: Vector,
    _y_axis: Vector,
    _z_axis: Vector,
    _a: f64,
    _b: f64,
    _c: f64,
    _d: f64,
}

impl Default for Plane {
    fn default() -> Self {
        Self::from_frame(
            Point::default(),
            Vector::x_axis(),
            Vector::y_axis(),
            Vector::z_axis(),
        )
    }
}

impl Plane {
    /// Origin and two axes; x is normalized, y is made orthogonal to x, z = x × y
    pub fn new(point: Point, x_axis: Vector, y_axis: Vector) -> Self {
        Self::with_name(point, x_axis, y_axis, "my_plane")
    }

    /// Construct from origin, two axes and a name
    pub fn with_name(point: Point, mut x_axis: Vector, mut y_axis: Vector, name: &str) -> Self {
        x_axis.normalize_self();
        y_axis -= x_axis.clone() * y_axis.dot(&x_axis);
        y_axis.normalize_self();
        let mut z_axis = x_axis.cross(&y_axis);
        z_axis.normalize_self();
        let mut plane = Self::from_frame(point, x_axis, y_axis, z_axis);
        plane.name = name.to_string();
        plane
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

    /// Clear the guid so a fresh one mints lazily on next read
    pub fn refresh_guid(&mut self) {
        self.guid = OnceLock::new();
    }

    pub fn origin(&self) -> Point {
        self._origin.clone()
    }

    pub fn x_axis(&self) -> Vector {
        self._x_axis.clone()
    }

    pub fn y_axis(&self) -> Vector {
        self._y_axis.clone()
    }

    pub fn z_axis(&self) -> Vector {
        self._z_axis.clone()
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

    /// Recompute a, b, c, d from z_axis and origin
    fn update_equation(&mut self) {
        self._a = self._z_axis[0];
        self._b = self._z_axis[1];
        self._c = self._z_axis[2];
        self._d =
            -(self._a * self._origin[0] + self._b * self._origin[1] + self._c * self._origin[2]);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Frame taken as given, no normalization
    pub fn from_frame(origin: Point, x_axis: Vector, y_axis: Vector, z_axis: Vector) -> Self {
        let mut plane = Self {
            guid: OnceLock::new(),
            name: "my_plane".to_string(),
            width: 1.0,
            linecolor: Color::blue(),
            _origin: origin,
            _x_axis: x_axis,
            _y_axis: y_axis,
            _z_axis: z_axis,
            _a: 0.0,
            _b: 0.0,
            _c: 0.0,
            _d: 0.0,
        };
        plane.update_equation();
        plane
    }

    /// Plane through point with normal as z axis (normalize: default true)
    pub fn from_point_normal(point: Point, normal: Vector, normalize: Option<bool>) -> Self {
        let normalize = normalize.unwrap_or(true);
        let mut z_axis = normal;
        if normalize {
            z_axis.normalize_self();
        }
        let mut x_axis = Vector::default();
        x_axis.perpendicular_to(&z_axis);
        if normalize {
            x_axis.normalize_self();
        }
        let mut y_axis = z_axis.cross(&x_axis);
        if normalize {
            y_axis.normalize_self();
        }
        Self::from_frame(point, x_axis, y_axis, z_axis)
    }

    /// Plane through the first three points, x axis along the first edge
    pub fn from_points(points: Vec<Point>) -> Self {
        if points.len() < 3 {
            return Self::default();
        }
        let v1 = points[1].clone() - points[0].clone();
        let v2 = points[2].clone() - points[0].clone();
        let mut z_axis = v1.cross(&v2);
        z_axis.normalize_self();
        let mut x_axis = v1;
        x_axis.normalize_self();
        let mut y_axis = z_axis.cross(&x_axis);
        y_axis.normalize_self();
        Self::from_frame(points[0].clone(), x_axis, y_axis, z_axis)
    }

    /// Least-squares plane through points by power-iteration PCA
    pub fn from_points_pca(points: Vec<Point>) -> Self {
        if points.len() < 3 {
            return Self::default();
        }
        let n = points.len() as f64;
        let mut cx = 0.0;
        let mut cy = 0.0;
        let mut cz = 0.0;
        for p in &points {
            cx += p[0];
            cy += p[1];
            cz += p[2];
        }
        cx /= n;
        cy /= n;
        cz /= n;
        let mut cxx = 0.0;
        let mut cyy = 0.0;
        let mut czz = 0.0;
        let mut cxy = 0.0;
        let mut cxz = 0.0;
        let mut cyz = 0.0;
        for p in &points {
            let dx = p[0] - cx;
            let dy = p[1] - cy;
            let dz = p[2] - cz;
            cxx += dx * dx;
            cyy += dy * dy;
            czz += dz * dz;
            cxy += dx * dy;
            cxz += dx * dz;
            cyz += dy * dz;
        }
        let mut eigvec = [[0.0; 3]; 3];
        let mut eigval = [0.0; 3];
        let mut cov = [[cxx, cxy, cxz], [cxy, cyy, cyz], [cxz, cyz, czz]];
        for e in 0..3 {
            let mut vx = if e == 0 { 1.0 } else { 0.0 };
            let mut vy = if e == 1 { 1.0 } else { 0.0 };
            let mut vz = if e == 2 { 1.0 } else { 0.0 };
            for _iter in 0..100 {
                let nx = cov[0][0] * vx + cov[0][1] * vy + cov[0][2] * vz;
                let ny = cov[1][0] * vx + cov[1][1] * vy + cov[1][2] * vz;
                let nz = cov[2][0] * vx + cov[2][1] * vy + cov[2][2] * vz;
                let mag = (nx * nx + ny * ny + nz * nz).sqrt();
                if mag < 1e-15 {
                    break;
                }
                vx = nx / mag;
                vy = ny / mag;
                vz = nz / mag;
            }
            eigvec[e][0] = vx;
            eigvec[e][1] = vy;
            eigvec[e][2] = vz;
            eigval[e] = cov[0][0] * vx * vx
                + cov[1][1] * vy * vy
                + cov[2][2] * vz * vz
                + 2.0 * cov[0][1] * vx * vy
                + 2.0 * cov[0][2] * vx * vz
                + 2.0 * cov[1][2] * vy * vz;
            for i in 0..3 {
                for j in 0..3 {
                    cov[i][j] -= eigval[e] * eigvec[e][i] * eigvec[e][j];
                }
            }
        }
        let mut x_axis = Vector::new(eigvec[0][0], eigvec[0][1], eigvec[0][2]);
        let mut y_axis = Vector::new(eigvec[1][0], eigvec[1][1], eigvec[1][2]);
        let mut z_axis = x_axis.cross(&y_axis);
        z_axis.normalize_self();
        y_axis = z_axis.cross(&x_axis);
        y_axis.normalize_self();
        x_axis.normalize_self();
        Self::from_frame(Point::new(cx, cy, cz), x_axis, y_axis, z_axis)
    }

    /// Plane with x axis from point1 to point2
    pub fn from_two_points(point1: Point, point2: Point) -> Self {
        let mut x_axis = point2 - point1.clone();
        x_axis.normalize_self();
        let mut z_axis = Vector::default();
        z_axis.perpendicular_to(&x_axis);
        z_axis.normalize_self();
        let mut y_axis = z_axis.cross(&x_axis);
        y_axis.normalize_self();
        Self::from_frame(point1, x_axis, y_axis, z_axis)
    }

    /// All-zero frame; fails is_valid()
    pub fn invalid() -> Self {
        Self::from_frame(
            Point::new(0.0, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.0),
        )
    }

    pub fn xy_plane() -> Self {
        let mut plane = Self::from_frame(
            Point::new(0.0, 0.0, 0.0),
            Vector::x_axis(),
            Vector::y_axis(),
            Vector::z_axis(),
        );
        plane.name = "xy_plane".to_string();
        plane
    }

    pub fn yz_plane() -> Self {
        let mut plane = Self::from_frame(
            Point::new(0.0, 0.0, 0.0),
            Vector::y_axis(),
            Vector::z_axis(),
            Vector::x_axis(),
        );
        plane.name = "yz_plane".to_string();
        plane
    }

    pub fn xz_plane() -> Self {
        let mut plane = Self::from_frame(
            Point::new(0.0, 0.0, 0.0),
            Vector::x_axis(),
            Vector::new(0.0, 0.0, -1.0),
            Vector::new(0.0, 1.0, 0.0),
        );
        plane.name = "xz_plane".to_string();
        plane
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Transform in place
    pub fn transform(&mut self, xform: &Xform) {
        self._origin.transform(xform);
        self._x_axis.transform(xform);
        self._y_axis.transform(xform);
        self._z_axis.transform(xform);
        self.update_equation();
    }

    /// Transformed copy
    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut result = self.clone();
        result.transform(xform);
        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn is_valid(&self) -> bool {
        self._x_axis.magnitude() > 1e-14
            && self._y_axis.magnitude() > 1e-14
            && self._z_axis.magnitude() > 1e-14
    }

    /// Swap x and y and flip z
    pub fn reverse(&mut self) {
        std::mem::swap(&mut self._x_axis, &mut self._y_axis);
        self._z_axis.reverse();
        self.update_equation();
    }

    /// Rotate x and y around z
    pub fn rotate(&mut self, angles_in_radians: f64) {
        let cos_angle = angles_in_radians.cos();
        let sin_angle = angles_in_radians.sin();
        let new_x = self._x_axis.clone() * cos_angle + self._y_axis.clone() * sin_angle;
        let new_y = self._y_axis.clone() * cos_angle - self._x_axis.clone() * sin_angle;
        self._x_axis = new_x;
        self._y_axis = new_y;
    }

    /// True when x × y points along z
    pub fn is_right_hand(&self) -> bool {
        self._x_axis.cross(&self._y_axis).dot(&self._z_axis) > 0.999
    }

    /// Normals parallel (can_be_flipped) or exactly opposite (!can_be_flipped)
    pub fn is_same_direction(plane0: &Plane, plane1: &Plane, can_be_flipped: bool) -> bool {
        let parallel = plane0._z_axis.is_parallel_to(&plane1._z_axis);
        if can_be_flipped {
            return parallel != 0;
        }
        parallel == -1
    }

    /// Each origin lies on the other plane
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
        let tolerance = Tolerance::APPROXIMATION;
        dist0 < tolerance && dist1 < tolerance
    }

    /// Same direction and same position
    pub fn is_coplanar(plane0: &Plane, plane1: &Plane, can_be_flipped: bool) -> bool {
        Self::is_same_direction(plane0, plane1, can_be_flipped)
            && Self::is_same_position(plane0, plane1)
    }

    /// is_coplanar from origin and normal pairs without building planes; tolerance < 0 uses APPROXIMATION
    pub fn is_coplanar_from_normals(
        origin0: &Point,
        normal0: &Vector,
        origin1: &Point,
        normal1: &Vector,
        can_be_flipped: bool,
        tolerance: f64,
    ) -> bool {
        let parallel = normal0.is_parallel_to(normal1);
        if if can_be_flipped {
            parallel == 0
        } else {
            parallel != -1
        } {
            return false;
        }
        let d0 = -(normal0[0] * origin0[0] + normal0[1] * origin0[1] + normal0[2] * origin0[2]);
        let d1 = -(normal1[0] * origin1[0] + normal1[1] * origin1[1] + normal1[2] * origin1[2]);
        let dist0 =
            (normal0[0] * origin1[0] + normal0[1] * origin1[1] + normal0[2] * origin1[2] + d0)
                .abs();
        let dist1 =
            (normal1[0] * origin0[0] + normal1[1] * origin0[1] + normal1[2] * origin0[2] + d1)
                .abs();
        let tol = if tolerance < 0.0 {
            Tolerance::APPROXIMATION
        } else {
            tolerance
        };
        dist0 < tol && dist1 < tol
    }

    /// Copy moved along z by distance
    pub fn translate_by_normal(&self, distance: f64) -> Plane {
        let mut normal = self._z_axis.clone();
        normal.normalize_self();
        Plane::with_name(
            self._origin.clone() + normal * distance,
            self._x_axis.clone(),
            self._y_axis.clone(),
            &self.name,
        )
    }

    /// Orthogonal projection of p onto the plane
    pub fn project(&self, p: &Point) -> Point {
        let dist = self._a * p[0] + self._b * p[1] + self._c * p[2] + self._d;
        Point::new(
            p[0] - dist * self._a,
            p[1] - dist * self._b,
            p[2] - dist * self._c,
        )
    }

    /// True when a*p[0] + b*p[1] + c*p[2] + d < 0
    pub fn has_on_negative_side(&self, p: &Point) -> bool {
        self._a * p[0] + self._b * p[1] + self._c * p[2] + self._d < 0.0
    }

    /// Canonical in-plane axis from the normal alone: zero the smallest normal coordinate, negate-swap the other two
    pub fn base1(&self) -> Vector {
        let nx = self._z_axis[0];
        let ny = self._z_axis[1];
        let nz = self._z_axis[2];
        let ax = nx.abs();
        let ay = ny.abs();
        let az = nz.abs();
        let mut b = if ax <= ay && ax <= az {
            Vector::new(0.0, -nz, ny)
        } else if ay <= ax && ay <= az {
            Vector::new(-nz, 0.0, nx)
        } else {
            Vector::new(-ny, nx, 0.0)
        };
        b.normalize_self();
        b
    }

    /// z × base1, unit length
    pub fn base2(&self) -> Vector {
        let mut b2 = self._z_axis.cross(&self.base1());
        b2.normalize_self();
        b2
    }

    /// Square outline of side scale plus the three axes as polylines
    pub fn to_polylines(&self, scale: f64) -> Vec<Polyline> {
        let s = scale * 0.5;
        let o = &self._origin;
        let x = &self._x_axis;
        let y = &self._y_axis;
        let z = &self._z_axis;
        let c0 = Point::new(
            o[0] - x[0] * s - y[0] * s,
            o[1] - x[1] * s - y[1] * s,
            o[2] - x[2] * s - y[2] * s,
        );
        let c1 = Point::new(
            o[0] + x[0] * s - y[0] * s,
            o[1] + x[1] * s - y[1] * s,
            o[2] + x[2] * s - y[2] * s,
        );
        let c2 = Point::new(
            o[0] + x[0] * s + y[0] * s,
            o[1] + x[1] * s + y[1] * s,
            o[2] + x[2] * s + y[2] * s,
        );
        let c3 = Point::new(
            o[0] - x[0] * s + y[0] * s,
            o[1] - x[1] * s + y[1] * s,
            o[2] - x[2] * s + y[2] * s,
        );
        let mut rect = Polyline::new(vec![c0.clone(), c1, c2, c3, c0]);
        rect.linecolor = self.linecolor.clone();
        let origin_pt = Point::new(o[0], o[1], o[2]);
        let mut x_line = Polyline::new(vec![
            origin_pt.clone(),
            Point::new(o[0] + x[0] * s, o[1] + x[1] * s, o[2] + x[2] * s),
        ]);
        x_line.linecolor = Color::red();
        let mut y_line = Polyline::new(vec![
            origin_pt.clone(),
            Point::new(o[0] + y[0] * s, o[1] + y[1] * s, o[2] + y[2] * s),
        ]);
        y_line.linecolor = Color::green();
        let mut z_line = Polyline::new(vec![
            origin_pt,
            Point::new(o[0] + z[0] * s, o[1] + z[1] * s, o[2] + z[2] * s),
        ]);
        z_line.linecolor = Color::blue();
        vec![rect, x_line, y_line, z_line]
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
        self.to_proto().encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        Ok(Self::from_proto(crate::proto::Plane::decode(data)?))
    }

    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    /// The proto message; pb_dumps encodes it and Session embeds it
    pub fn to_proto(&self) -> crate::proto::Plane {
        let mut frame = Vec::with_capacity(12);
        for i in 0..3 {
            frame.push(self._origin[i]);
        }
        for i in 0..3 {
            frame.push(self._x_axis[i]);
        }
        for i in 0..3 {
            frame.push(self._y_axis[i]);
        }
        for i in 0..3 {
            frame.push(self._z_axis[i]);
        }
        crate::proto::Plane {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            frame,
            width: self.width,
            linecolor: Some(crate::proto::Color {
                guid: String::new(),
                name: self.linecolor.name.clone(),
                r: self.linecolor.r,
                g: self.linecolor.g,
                b: self.linecolor.b,
                a: self.linecolor.a,
            }),
        }
    }

    /// Plane from a decoded proto message
    pub fn from_proto(proto: crate::proto::Plane) -> Self {
        let mut plane = Self::default();
        if proto.frame.len() >= 12 {
            plane = Self::from_frame(
                Point::new(proto.frame[0], proto.frame[1], proto.frame[2]),
                Vector::new(proto.frame[3], proto.frame[4], proto.frame[5]),
                Vector::new(proto.frame[6], proto.frame[7], proto.frame[8]),
                Vector::new(proto.frame[9], proto.frame[10], proto.frame[11]),
            );
        }
        if !proto.guid.is_empty() {
            plane.set_guid(proto.guid);
        }
        plane.name = proto.name;
        if proto.width > 0.0 {
            plane.width = proto.width;
        }
        if let Some(color) = proto.linecolor {
            plane.linecolor.name = color.name;
            plane.linecolor.r = color.r;
            plane.linecolor.g = color.g;
            plane.linecolor.b = color.b;
            plane.linecolor.a = color.a;
        }
        plane
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════

    /// "origin\nx_axis\ny_axis\nz_axis"
    pub fn str(&self) -> String {
        format!(
            "{}\n{}\n{}\n{}",
            self._origin.str(),
            self._x_axis.str(),
            self._y_axis.str(),
            self._z_axis.str()
        )
    }

    /// "Plane(name, ox, oy, oz, zx, zy, zz, Color(...))"
    pub fn repr(&self) -> String {
        let prec = Tolerance::ROUNDING;
        format!(
            "Plane({}, {}, {}, {}, {}, {}, {}, {})",
            self.name,
            TOLERANCE.format_number(self._origin[0], prec),
            TOLERANCE.format_number(self._origin[1], prec),
            TOLERANCE.format_number(self._origin[2], prec),
            TOLERANCE.format_number(self._z_axis[0], prec),
            TOLERANCE.format_number(self._z_axis[1], prec),
            TOLERANCE.format_number(self._z_axis[2], prec),
            self.linecolor.repr()
        )
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SESSION_VIEWER
    // ═══════════════════════════════════════════════════════════════════════════

    /// GPU-ready column-major [f32; 16] frame matrix (x, y, z, origin columns), the f64 to f32 boundary for wgpu upload
    pub fn to_f32(&self) -> [f32; 16] {
        let x = &self._x_axis;
        let y = &self._y_axis;
        let z = &self._z_axis;
        let o = &self._origin;
        [
            x[0] as f32,
            x[1] as f32,
            x[2] as f32,
            0.0,
            y[0] as f32,
            y[1] as f32,
            y[2] as f32,
            0.0,
            z[0] as f32,
            z[1] as f32,
            z[2] as f32,
            0.0,
            o[0] as f32,
            o[1] as f32,
            o[2] as f32,
            1.0,
        ]
    }
}

impl fmt::Display for Plane {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════

/// Frame as one flat array of 12 numbers: origin, x_axis, y_axis, z_axis
impl Serialize for Plane {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut map = serializer.serialize_map(Some(6))?;
        map.serialize_entry(
            "frame",
            &[
                self._origin[0],
                self._origin[1],
                self._origin[2],
                self._x_axis[0],
                self._x_axis[1],
                self._x_axis[2],
                self._y_axis[0],
                self._y_axis[1],
                self._y_axis[2],
                self._z_axis[0],
                self._z_axis[1],
                self._z_axis[2],
            ],
        )?;
        map.serialize_entry("guid", self.guid())?;
        map.serialize_entry("linecolor", &self.linecolor)?;
        map.serialize_entry("name", &self.name)?;
        map.serialize_entry("type", "Plane")?;
        map.serialize_entry("width", &self.width)?;
        map.end()
    }
}

/// a, b, c, d are recomputed from the frame on load
impl<'de> Deserialize<'de> for Plane {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct PlaneData {
            frame: [f64; 12],
            guid: String,
            #[serde(default)]
            linecolor: Option<Color>,
            name: String,
            #[serde(default = "default_width")]
            width: f64,
        }

        fn default_width() -> f64 {
            1.0
        }

        let data = PlaneData::deserialize(deserializer)?;
        let mut plane = Plane::from_frame(
            Point::new(data.frame[0], data.frame[1], data.frame[2]),
            Vector::new(data.frame[3], data.frame[4], data.frame[5]),
            Vector::new(data.frame[6], data.frame[7], data.frame[8]),
            Vector::new(data.frame[9], data.frame[10], data.frame[11]),
        );
        plane.set_guid(data.guid);
        plane.name = data.name;
        plane.width = data.width;
        plane.linecolor = data.linecolor.unwrap_or_else(Color::blue);
        Ok(plane)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

impl Index<usize> for Plane {
    type Output = Vector;

    /// Axis by index (0=x, 1=y, 2=z)
    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self._x_axis,
            1 => &self._y_axis,
            _ => &self._z_axis,
        }
    }
}

impl IndexMut<usize> for Plane {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self._x_axis,
            1 => &mut self._y_axis,
            _ => &mut self._z_axis,
        }
    }
}

impl PartialEq for Plane {
    /// Same name, frame and linecolor; guid ignored
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && self._origin == other._origin
            && self._x_axis == other._x_axis
            && self._y_axis == other._y_axis
            && self._z_axis == other._z_axis
            && self.linecolor == other.linecolor
    }
}

impl AddAssign<Vector> for Plane {
    fn add_assign(&mut self, other: Vector) {
        self._origin += other;
        self.update_equation();
    }
}

impl SubAssign<Vector> for Plane {
    fn sub_assign(&mut self, other: Vector) {
        self._origin -= other;
        self.update_equation();
    }
}

impl Add<Vector> for Plane {
    type Output = Plane;

    fn add(self, other: Vector) -> Plane {
        let mut result = self.clone();
        result += other;
        result
    }
}

impl Sub<Vector> for Plane {
    type Output = Plane;

    fn sub(self, other: Vector) -> Plane {
        let mut result = self.clone();
        result -= other;
        result
    }
}
