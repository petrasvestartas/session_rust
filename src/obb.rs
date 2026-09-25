use crate::Line;
use crate::Mesh;
use crate::NurbsCurve;
use crate::NurbsSurface;
use crate::Plane;
use crate::Point;
use crate::PointCloud;
use crate::Polyline;
use crate::Vector;
use crate::Xform;
use crate::AABB;
use serde::Deserialize;
use serde::Serialize;
use std::fmt;
use std::sync::OnceLock;

const NUM_SAMPLES: usize = 20; // Samples per span when searching curve extrema.
const MAX_ITER: usize = 20; // Newton iterations per extremum.

/// Oriented bounding box as center, three axes and half-size.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "OBB")]
pub struct OBB {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: OnceLock<String>, // Lazy guid.
    pub center: Point,     // Box center.
    pub x_axis: Vector,    // Unit x axis.
    pub y_axis: Vector,    // Unit y axis.
    pub z_axis: Vector,    // Unit z axis.
    pub half_size: Vector, // Half extent along each axis.
    pub name: String,      // Box name.
}

impl OBB {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from center, axes and half-size.
    pub fn new(
        center: Point,
        x_axis: Vector,
        y_axis: Vector,
        z_axis: Vector,
        half_size: Vector,
    ) -> Self {
        Self {
            guid: OnceLock::new(),
            center,
            x_axis,
            y_axis,
            z_axis,
            half_size,
            name: "my_obb".to_string(),
        }
    }

    /// Copy with a new guid and the same data.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = OnceLock::new();

        copy
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the guid, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid.
    pub fn set_guid(&mut self, guid: String) {
        self.guid = OnceLock::from(guid);
    }

    /// Clear the guid so a fresh one mints lazily on the next read.
    pub fn refresh_guid(&mut self) {
        self.guid = OnceLock::new();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct on the plane frame with full sizes dx, dy, dz.
    pub fn from_plane(plane: &Plane, dx: f64, dy: f64, dz: f64) -> Self {
        Self::new(
            plane.origin(),
            plane.x_axis(),
            plane.y_axis(),
            plane.z_axis(),
            Vector::new(dx * 0.5, dy * 0.5, dz * 0.5),
        )
    }

    /// Construct the world-aligned box with the center and half-size of aabb.
    pub fn from_aabb(aabb: &AABB) -> Self {
        Self::new(
            Point::new(aabb.cx, aabb.cy, aabb.cz),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(aabb.hx, aabb.hy, aabb.hz),
        )
    }

    /// Construct the world-aligned box of half-size inflate around point.
    pub fn from_point(point: &Point, inflate: f64) -> Self {
        Self::from_aabb(&AABB::from_point(point, inflate))
    }

    /// Construct the world-aligned tight box of points, or tight in the plane frame, grown by inflate.
    pub fn from_points(points: &[Point], inflate: f64, plane: Option<&Plane>) -> Self {
        let Some(plane) = plane else {
            return Self::from_aabb(&AABB::from_points(points, inflate));
        };

        if points.is_empty() {
            return OBB::default();
        }

        let origin = plane.origin();
        let x_axis = plane.x_axis();
        let y_axis = plane.y_axis();
        let z_axis = plane.z_axis();
        let world_to_local = Xform::world_to_frame(&origin, &x_axis, &y_axis, &z_axis);
        let local_to_world = Xform::frame_to_world(&origin, &x_axis, &y_axis, &z_axis);

        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut min_z = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;
        let mut max_z = f64::MIN;

        for pt in points {
            let local = pt.transformed(&world_to_local);

            min_x = min_x.min(local[0]);
            min_y = min_y.min(local[1]);
            min_z = min_z.min(local[2]);
            max_x = max_x.max(local[0]);
            max_y = max_y.max(local[1]);
            max_z = max_z.max(local[2]);
        }

        let local_center = Point::new(
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
        );
        let half_size = Vector::new(
            (max_x - min_x) * 0.5 + inflate,
            (max_y - min_y) * 0.5 + inflate,
            (max_z - min_z) * 0.5 + inflate,
        );

        Self::new(
            local_center.transformed(&local_to_world),
            x_axis,
            y_axis,
            z_axis,
            half_size,
        )
    }

    /// Construct the tight box of the two ends, world-aligned or in the plane frame, grown by inflate.
    pub fn from_line(line: &Line, inflate: f64, plane: Option<&Plane>) -> Self {
        let Some(plane) = plane else {
            return Self::from_aabb(&AABB::from_line(line, inflate));
        };

        Self::from_points(&[line.start(), line.end()], inflate, Some(plane))
    }

    /// Construct the tight box of the vertices, world-aligned or in the plane frame, grown by inflate.
    pub fn from_polyline(polyline: &Polyline, inflate: f64, plane: Option<&Plane>) -> Self {
        let Some(plane) = plane else {
            return Self::from_aabb(&AABB::from_polyline(polyline, inflate));
        };

        Self::from_points(&polyline.get_points(), inflate, Some(plane))
    }

    /// Construct the tight box of the vertices, world-aligned or in the plane frame, grown by inflate.
    pub fn from_mesh(mesh: &Mesh, inflate: f64, plane: Option<&Plane>) -> Self {
        let Some(plane) = plane else {
            return Self::from_aabb(&AABB::from_mesh(mesh, inflate));
        };

        Self::from_points(&mesh.to_vertices_and_faces().0, inflate, Some(plane))
    }

    /// Construct the tight box of the points, world-aligned or in the plane frame, grown by inflate.
    pub fn from_pointcloud(pointcloud: &PointCloud, inflate: f64, plane: Option<&Plane>) -> Self {
        let Some(plane) = plane else {
            return Self::from_aabb(&AABB::from_pointcloud(pointcloud, inflate));
        };

        Self::from_points(&pointcloud.get_points(), inflate, Some(plane))
    }

    /// Construct the box of the control points, or of the curve extrema when tight, world-aligned or in the plane frame.
    pub fn from_nurbscurve(
        curve: &NurbsCurve,
        inflate: f64,
        tight: bool,
        plane: Option<&Plane>,
    ) -> Self {
        let Some(plane) = plane else {
            return Self::from_aabb(&AABB::from_nurbscurve(curve, inflate, tight));
        };

        if !curve.is_valid() || curve.cv_count() == 0 {
            return OBB::default();
        }

        let mut points = Vec::new();

        if !tight {
            for i in 0..curve.cv_count() {
                if let Some(pt) = curve.get_cv(i) {
                    points.push(pt);
                }
            }

            return Self::from_points(&points, inflate, Some(plane));
        }

        let t0 = curve.domain_start();
        let t1 = curve.domain_end();

        points.push(curve.point_at(t0));
        points.push(curve.point_at(t1));

        for t in curve.get_span_vector() {
            if t > t0 && t < t1 {
                points.push(curve.point_at(t));
            }
        }

        let axes = [plane.x_axis(), plane.y_axis(), plane.z_axis()];
        let dt = (t1 - t0) / NUM_SAMPLES as f64;

        for axis in &axes {
            for i in 0..NUM_SAMPLES {
                let t_start = t0 + i as f64 * dt;
                let t_end = t_start + dt;
                let deriv_start: Vec<Vector> = curve.evaluate(t_start, 1);
                let deriv_end: Vec<Vector> = curve.evaluate(t_end, 1);

                if deriv_start.len() < 2 || deriv_end.len() < 2 {
                    continue;
                }

                let d_start = deriv_start[1].dot(axis);
                let d_end = deriv_end[1].dot(axis);

                if d_start * d_end < 0.0 {
                    let t_root = Self::compute_extremum(curve, axis, t_start, t_end, d_start);
                    points.push(curve.point_at(t_root));
                }
            }
        }

        Self::from_points(&points, inflate, Some(plane))
    }

    /// Construct the box of the control points, world-aligned or in the plane frame, grown by inflate.
    pub fn from_nurbssurface(surface: &NurbsSurface, inflate: f64, plane: Option<&Plane>) -> Self {
        let Some(plane) = plane else {
            return Self::from_aabb(&AABB::from_nurbssurface(surface, inflate));
        };

        if !surface.is_valid() || surface.cv_count(0) == 0 || surface.cv_count(1) == 0 {
            return OBB::default();
        }

        let mut points = Vec::new();

        for i in 0..surface.cv_count(0) {
            for j in 0..surface.cv_count(1) {
                if let Some(pt) = surface.get_cv(i, j) {
                    points.push(pt);
                }
            }
        }

        Self::from_points(&points, inflate, Some(plane))
    }

    /// Compute the parameter in [t_lo, t_hi] where the derivative along axis crosses zero, by Newton steps bracketed by bisection.
    fn compute_extremum(
        curve: &NurbsCurve,
        axis: &Vector,
        t_lo: f64,
        t_hi: f64,
        d_start: f64,
    ) -> f64 {
        let mut t_lo = t_lo;
        let mut t_hi = t_hi;
        let mut d_start = d_start;
        let mut t_root = (t_lo + t_hi) * 0.5;

        for _ in 0..MAX_ITER {
            let deriv: Vec<Vector> = curve.evaluate(t_root, 2);

            if deriv.len() < 3 {
                break;
            }

            let d1 = deriv[1].dot(axis);
            let d2 = deriv[2].dot(axis);

            if d1.abs() < 1e-12 {
                break;
            }

            if d2.abs() > 1e-14 {
                let t_new = t_root - d1 / d2;

                if t_new >= t_lo && t_new <= t_hi {
                    t_root = t_new;
                } else {
                    if d1 * d_start < 0.0 {
                        t_hi = t_root;
                    } else {
                        t_lo = t_root;
                    }

                    t_root = (t_lo + t_hi) * 0.5;
                }
            } else {
                t_root = (t_lo + t_hi) * 0.5;
            }

            let deriv_check: Vec<Vector> = curve.evaluate(t_root, 1);

            if deriv_check.len() < 2 {
                continue;
            }

            let d_check = deriv_check[1].dot(axis);

            if d_check * d_start < 0.0 {
                t_hi = t_root;
            } else {
                t_lo = t_root;
                d_start = d_check;
            }
        }

        t_root
    }
}

impl Default for OBB {
    /// Construct the unit box at the origin.
    fn default() -> Self {
        Self::new(
            Point::new(0.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(0.5, 0.5, 0.5),
        )
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
impl PartialEq for OBB {
    /// Compare name, center, axes and half-size to 1e-6; guid ignored.
    fn eq(&self, other: &Self) -> bool {
        if self.name != other.name {
            return false;
        }

        for i in 0..3 {
            if (self.center[i] * 1000000.0).round() != (other.center[i] * 1000000.0).round() {
                return false;
            }

            if (self.x_axis[i] * 1000000.0).round() != (other.x_axis[i] * 1000000.0).round() {
                return false;
            }

            if (self.y_axis[i] * 1000000.0).round() != (other.y_axis[i] * 1000000.0).round() {
                return false;
            }

            if (self.z_axis[i] * 1000000.0).round() != (other.z_axis[i] * 1000000.0).round() {
                return false;
            }

            if (self.half_size[i] * 1000000.0).round() != (other.half_size[i] * 1000000.0).round() {
                return false;
            }
        }

        true
    }
}

impl OBB {
    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform center and axes in place.
    pub fn transform(&mut self, xform: &Xform) {
        self.center.transform(xform);
        self.x_axis.transform(xform);
        self.y_axis.transform(xform);
        self.z_axis.transform(xform);
    }

    /// Return a transformed copy.
    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut result = self.duplicate();
        result.transform(xform);

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the world-aligned box enclosing the corners.
    pub fn aabb(&self) -> AABB {
        let ex = self.half_size[0];
        let ey = self.half_size[1];
        let ez = self.half_size[2];
        let hx = self.x_axis[0].abs() * ex + self.y_axis[0].abs() * ey + self.z_axis[0].abs() * ez;
        let hy = self.x_axis[1].abs() * ex + self.y_axis[1].abs() * ey + self.z_axis[1].abs() * ez;
        let hz = self.x_axis[2].abs() * ex + self.y_axis[2].abs() * ey + self.z_axis[2].abs() * ez;

        AABB::new(self.center[0], self.center[1], self.center[2], hx, hy, hz)
    }

    /// Return the min corner of the world-aligned box.
    pub fn min_point(&self) -> Point {
        self.aabb().min_point()
    }

    /// Return the max corner of the world-aligned box.
    pub fn max_point(&self) -> Point {
        self.aabb().max_point()
    }

    /// Return the surface area.
    pub fn area(&self) -> f64 {
        let hx = self.half_size[0];
        let hy = self.half_size[1];
        let hz = self.half_size[2];

        8.0 * (hx * hy + hy * hz + hz * hx)
    }

    /// Return the length of the space diagonal.
    pub fn diagonal(&self) -> f64 {
        let hx = self.half_size[0];
        let hy = self.half_size[1];
        let hz = self.half_size[2];

        2.0 * (hx * hx + hy * hy + hz * hz).sqrt()
    }

    /// Return the volume.
    pub fn volume(&self) -> f64 {
        8.0 * self.half_size[0] * self.half_size[1] * self.half_size[2]
    }

    /// Return whether no half-size is negative.
    pub fn is_valid(&self) -> bool {
        self.half_size[0] >= 0.0 && self.half_size[1] >= 0.0 && self.half_size[2] >= 0.0
    }

    /// Return pt clamped to the box in its own frame.
    pub fn closest_point(&self, pt: &Point) -> Point {
        let offset = pt - &self.center;
        let lx = (-self.half_size[0]).max(self.half_size[0].min(offset.dot(&self.x_axis)));
        let ly = (-self.half_size[1]).max(self.half_size[1].min(offset.dot(&self.y_axis)));
        let lz = (-self.half_size[2]).max(self.half_size[2].min(offset.dot(&self.z_axis)));

        self.point_at(lx, ly, lz)
    }

    /// Return whether pt lies inside or on the box.
    pub fn contains(&self, pt: &Point) -> bool {
        let offset = pt - &self.center;
        let lx = offset.dot(&self.x_axis).abs();
        let ly = offset.dot(&self.y_axis).abs();
        let lz = offset.dot(&self.z_axis).abs();

        lx <= self.half_size[0] && ly <= self.half_size[1] && lz <= self.half_size[2]
    }

    /// Return the corner picked by the sign of each half-size.
    pub fn corner(&self, x_max: bool, y_max: bool, z_max: bool) -> Point {
        let ox = if x_max {
            self.half_size[0]
        } else {
            -self.half_size[0]
        };
        let oy = if y_max {
            self.half_size[1]
        } else {
            -self.half_size[1]
        };
        let oz = if z_max {
            self.half_size[2]
        } else {
            -self.half_size[2]
        };

        self.point_at(ox, oy, oz)
    }

    /// Return the bottom loop then the top loop, counter-clockwise from +x+y.
    pub fn corners(&self) -> [Point; 8] {
        [
            self.point_at(self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(-self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(-self.half_size[0], -self.half_size[1], -self.half_size[2]),
            self.point_at(self.half_size[0], -self.half_size[1], -self.half_size[2]),
            self.point_at(self.half_size[0], self.half_size[1], self.half_size[2]),
            self.point_at(-self.half_size[0], self.half_size[1], self.half_size[2]),
            self.point_at(-self.half_size[0], -self.half_size[1], self.half_size[2]),
            self.point_at(self.half_size[0], -self.half_size[1], self.half_size[2]),
        ]
    }

    /// Return the bottom loop then the top loop, counter-clockwise from +x+y.
    pub fn get_corners(&self) -> [Point; 8] {
        self.corners()
    }

    /// Return the bottom loop, the top loop, then the four verticals.
    pub fn get_edges(&self) -> Vec<Line> {
        let points = self.corners();

        vec![
            Line::from_points(&points[0], &points[1]),
            Line::from_points(&points[1], &points[2]),
            Line::from_points(&points[2], &points[3]),
            Line::from_points(&points[3], &points[0]),
            Line::from_points(&points[4], &points[5]),
            Line::from_points(&points[5], &points[6]),
            Line::from_points(&points[6], &points[7]),
            Line::from_points(&points[7], &points[4]),
            Line::from_points(&points[0], &points[4]),
            Line::from_points(&points[1], &points[5]),
            Line::from_points(&points[2], &points[6]),
            Line::from_points(&points[3], &points[7]),
        ]
    }

    /// Return the bottom loop and the top loop, each closed by repeating its first corner.
    pub fn two_rectangles(&self) -> [Point; 10] {
        [
            self.point_at(self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(-self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(-self.half_size[0], -self.half_size[1], -self.half_size[2]),
            self.point_at(self.half_size[0], -self.half_size[1], -self.half_size[2]),
            self.point_at(self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(self.half_size[0], self.half_size[1], self.half_size[2]),
            self.point_at(-self.half_size[0], self.half_size[1], self.half_size[2]),
            self.point_at(-self.half_size[0], -self.half_size[1], self.half_size[2]),
            self.point_at(self.half_size[0], -self.half_size[1], self.half_size[2]),
            self.point_at(self.half_size[0], self.half_size[1], self.half_size[2]),
        ]
    }

    /// Return the center offset by x, y, z along the axes.
    pub fn point_at(&self, x: f64, y: f64, z: f64) -> Point {
        &self.center + &self.x_axis * x + &self.y_axis * y + &self.z_axis * z
    }

    /// Grow every half-size by amount.
    pub fn inflate(&mut self, amount: f64) {
        self.half_size += Vector::new(amount, amount, amount);
    }

    /// Grow in place to enclose the corners of other.
    pub fn union_with(&mut self, other: &OBB) {
        let mut min_x = -self.half_size[0];
        let mut min_y = -self.half_size[1];
        let mut min_z = -self.half_size[2];
        let mut max_x = self.half_size[0];
        let mut max_y = self.half_size[1];
        let mut max_z = self.half_size[2];

        for point in &other.corners() {
            let offset = point - &self.center;
            let lx = offset.dot(&self.x_axis);
            let ly = offset.dot(&self.y_axis);
            let lz = offset.dot(&self.z_axis);

            min_x = min_x.min(lx);
            min_y = min_y.min(ly);
            min_z = min_z.min(lz);
            max_x = max_x.max(lx);
            max_y = max_y.max(ly);
            max_z = max_z.max(lz);
        }

        self.center = self.point_at(
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
        );
        self.half_size = Vector::new(
            (max_x - min_x) * 0.5,
            (max_y - min_y) * 0.5,
            (max_z - min_z) * 0.5,
        );
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Collision
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the boxes overlap by the separating axis test (collides_with_rtcd).
    pub fn collides_with(&self, other: &OBB) -> bool {
        self.collides_with_rtcd(other)
    }

    /// Return whether the boxes overlap, rejecting by AABB before collides_with.
    pub fn collides_with_broad(&self, other: &OBB) -> bool {
        if !self.aabb().intersects(&other.aabb()) {
            return false;
        }

        self.collides_with(other)
    }

    /// Return whether the boxes overlap by the fifteen-axis test in the frame of this box (Real-Time Collision Detection).
    pub fn collides_with_rtcd(&self, other: &OBB) -> bool {
        let eps = 1e-9;
        let a0 = self.half_size[0];
        let a1 = self.half_size[1];
        let a2 = self.half_size[2];
        let b0 = other.half_size[0];
        let b1 = other.half_size[1];
        let b2 = other.half_size[2];
        let r00 = self.x_axis.dot(&other.x_axis);
        let r01 = self.x_axis.dot(&other.y_axis);
        let r02 = self.x_axis.dot(&other.z_axis);
        let r10 = self.y_axis.dot(&other.x_axis);
        let r11 = self.y_axis.dot(&other.y_axis);
        let r12 = self.y_axis.dot(&other.z_axis);
        let r20 = self.z_axis.dot(&other.x_axis);
        let r21 = self.z_axis.dot(&other.y_axis);
        let r22 = self.z_axis.dot(&other.z_axis);
        let offset = &other.center - &self.center;
        let t0 = offset.dot(&self.x_axis);
        let t1 = offset.dot(&self.y_axis);
        let t2 = offset.dot(&self.z_axis);
        let ar00 = r00.abs() + eps;
        let ar01 = r01.abs() + eps;
        let ar02 = r02.abs() + eps;
        let ar10 = r10.abs() + eps;
        let ar11 = r11.abs() + eps;
        let ar12 = r12.abs() + eps;
        let ar20 = r20.abs() + eps;
        let ar21 = r21.abs() + eps;
        let ar22 = r22.abs() + eps;

        if t0.abs() > a0 + b0 * ar00 + b1 * ar01 + b2 * ar02 {
            return false;
        }

        if t1.abs() > a1 + b0 * ar10 + b1 * ar11 + b2 * ar12 {
            return false;
        }

        if t2.abs() > a2 + b0 * ar20 + b1 * ar21 + b2 * ar22 {
            return false;
        }

        if (t0 * r00 + t1 * r10 + t2 * r20).abs() > a0 * ar00 + a1 * ar10 + a2 * ar20 + b0 {
            return false;
        }

        if (t0 * r01 + t1 * r11 + t2 * r21).abs() > a0 * ar01 + a1 * ar11 + a2 * ar21 + b1 {
            return false;
        }

        if (t0 * r02 + t1 * r12 + t2 * r22).abs() > a0 * ar02 + a1 * ar12 + a2 * ar22 + b2 {
            return false;
        }

        if (t2 * r10 - t1 * r20).abs() > a1 * ar20 + a2 * ar10 + b1 * ar02 + b2 * ar01 {
            return false;
        }

        if (t2 * r11 - t1 * r21).abs() > a1 * ar21 + a2 * ar11 + b0 * ar02 + b2 * ar00 {
            return false;
        }

        if (t2 * r12 - t1 * r22).abs() > a1 * ar22 + a2 * ar12 + b0 * ar01 + b1 * ar00 {
            return false;
        }

        if (t0 * r20 - t2 * r00).abs() > a0 * ar20 + a2 * ar00 + b1 * ar12 + b2 * ar11 {
            return false;
        }

        if (t0 * r21 - t2 * r01).abs() > a0 * ar21 + a2 * ar01 + b0 * ar12 + b2 * ar10 {
            return false;
        }

        if (t0 * r22 - t2 * r02).abs() > a0 * ar22 + a2 * ar02 + b0 * ar11 + b1 * ar10 {
            return false;
        }

        if (t1 * r00 - t0 * r10).abs() > a0 * ar10 + a1 * ar00 + b1 * ar22 + b2 * ar21 {
            return false;
        }

        if (t1 * r01 - t0 * r11).abs() > a0 * ar11 + a1 * ar01 + b0 * ar22 + b2 * ar20 {
            return false;
        }

        if (t1 * r02 - t0 * r12).abs() > a0 * ar12 + a1 * ar02 + b0 * ar21 + b1 * ar20 {
            return false;
        }

        true
    }

    /// Return whether the boxes overlap by the fifteen-axis test on projected extents.
    pub fn collides_with_naive(&self, other: &OBB) -> bool {
        let offset = &other.center - &self.center;
        let axes = [
            self.x_axis.clone(),
            self.y_axis.clone(),
            self.z_axis.clone(),
            other.x_axis.clone(),
            other.y_axis.clone(),
            other.z_axis.clone(),
            self.x_axis.cross(&other.x_axis),
            self.x_axis.cross(&other.y_axis),
            self.x_axis.cross(&other.z_axis),
            self.y_axis.cross(&other.x_axis),
            self.y_axis.cross(&other.y_axis),
            self.y_axis.cross(&other.z_axis),
            self.z_axis.cross(&other.x_axis),
            self.z_axis.cross(&other.y_axis),
            self.z_axis.cross(&other.z_axis),
        ];

        for axis in &axes {
            if Self::separating_plane_exists(&offset, axis, self, other) {
                return false;
            }
        }

        true
    }

    /// Return whether the extents of both boxes projected on axis do not reach their center distance.
    fn separating_plane_exists(
        relative_position: &Vector,
        axis: &Vector,
        box1: &OBB,
        box2: &OBB,
    ) -> bool {
        let proj1 = (&box1.x_axis * box1.half_size[0]).dot(axis).abs()
            + (&box1.y_axis * box1.half_size[1]).dot(axis).abs()
            + (&box1.z_axis * box1.half_size[2]).dot(axis).abs();

        let proj2 = (&box2.x_axis * box2.half_size[0]).dot(axis).abs()
            + (&box2.y_axis * box2.half_size[1]).dot(axis).abs()
            + (&box2.z_axis * box2.half_size[2]).dot(axis).abs();

        relative_position.dot(axis).abs() > proj1 + proj2
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_default()
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filename, self.jsondump()?)?;

        Ok(())
    }

    /// Read from a JSON file.
    pub fn file_json_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filename)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::BoundingBox {
        crate::proto::BoundingBox {
            center: Some(self.center.to_proto()),
            x_axis: Some(self.x_axis.to_proto()),
            y_axis: Some(self.y_axis.to_proto()),
            z_axis: Some(self.z_axis.to_proto()),
            half_size: Some(self.half_size.to_proto()),
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
        }
    }

    /// Construct from the protobuf message.
    pub fn from_proto(
        proto: crate::proto::BoundingBox,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let mut obb = Self::new(
            Point::from_proto(proto.center.unwrap_or_default()),
            Vector::from_proto(proto.x_axis.unwrap_or_default()),
            Vector::from_proto(proto.y_axis.unwrap_or_default()),
            Vector::from_proto(proto.z_axis.unwrap_or_default()),
            Vector::from_proto(proto.half_size.unwrap_or_default()),
        );

        if !proto.guid.is_empty() {
            obb.set_guid(proto.guid);
        }

        obb.name = proto.name;

        Ok(obb)
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Self::from_proto(crate::proto::BoundingBox::decode(data)?)
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filename, self.pb_dumps())?;

        Ok(())
    }

    /// Read from a protobuf file.
    pub fn pb_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::pb_loads(&std::fs::read(filename)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "center\nx_axis\ny_axis\nz_axis\nhalf_size".
    pub fn str(&self) -> String {
        format!(
            "{}\n{}\n{}\n{}\n{}",
            self.center.str(),
            self.x_axis.str(),
            self.y_axis.str(),
            self.z_axis.str(),
            self.half_size.str()
        )
    }

    /// Return "OBB(name, center, x_axis, y_axis, z_axis, half_size)".
    pub fn repr(&self) -> String {
        format!(
            "OBB({}, {}, {}, {}, {}, {})",
            self.name,
            self.center.str(),
            self.x_axis.str(),
            self.y_axis.str(),
            self.z_axis.str(),
            self.half_size.str()
        )
    }
}

impl fmt::Display for OBB {
    /// Write the str() form to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SESSION_VIEWER
// ═══════════════════════════════════════════════════════════════════════════
impl OBB {
    /// Return the 8 box corners as f32 [x, y, z] rows for a wireframe-box buffer, same winding as corners.
    pub fn corners_f32(&self) -> [[f32; 3]; 8] {
        self.corners().map(|p| p.to_f32())
    }
}
