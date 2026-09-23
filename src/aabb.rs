use crate::tolerance::Tolerance;
use crate::tolerance::TOLERANCE;
use crate::Line;
use crate::Mesh;
use crate::NurbsCurve;
use crate::NurbsSurface;
use crate::Point;
use crate::PointCloud;
use crate::Polyline;
use crate::Vector;
use crate::Xform;
use std::fmt;

const NUM_SAMPLES: usize = 20; // Samples per span when searching curve extrema.
const MAX_ITER: usize = 20; // Newton iterations per extremum.

/// Axis-aligned bounding box as center and half-size.
#[derive(Clone, Copy, Default, Debug)]
pub struct AABB {
    pub cx: f64, // Center x.
    pub cy: f64, // Center y.
    pub cz: f64, // Center z.
    pub hx: f64, // Half-size along x.
    pub hy: f64, // Half-size along y.
    pub hz: f64, // Half-size along z.
}

impl AABB {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from center and half-size.
    pub fn new(cx: f64, cy: f64, cz: f64, hx: f64, hy: f64, hz: f64) -> Self {
        AABB {
            cx,
            cy,
            cz,
            hx,
            hy,
            hz,
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct the box of half-size inflate around point.
    pub fn from_point(point: &Point, inflate: f64) -> Self {
        AABB::new(point[0], point[1], point[2], inflate, inflate, inflate)
    }

    /// Construct the tight box of points grown by inflate.
    pub fn from_points(points: &[Point], inflate: f64) -> Self {
        if points.is_empty() {
            return AABB::default();
        }

        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut min_z = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;
        let mut max_z = f64::MIN;

        for pt in points {
            min_x = min_x.min(pt[0]);
            min_y = min_y.min(pt[1]);
            min_z = min_z.min(pt[2]);
            max_x = max_x.max(pt[0]);
            max_y = max_y.max(pt[1]);
            max_z = max_z.max(pt[2]);
        }

        AABB::new(
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
            (max_x - min_x) * 0.5 + inflate,
            (max_y - min_y) * 0.5 + inflate,
            (max_z - min_z) * 0.5 + inflate,
        )
    }

    /// Construct the tight box of the two ends grown by inflate.
    pub fn from_line(line: &Line, inflate: f64) -> Self {
        Self::from_points(&[line.start(), line.end()], inflate)
    }

    /// Construct the tight box of the vertices grown by inflate.
    pub fn from_polyline(polyline: &Polyline, inflate: f64) -> Self {
        Self::from_points(&polyline.get_points(), inflate)
    }

    /// Construct the tight box of the vertices grown by inflate.
    pub fn from_mesh(mesh: &Mesh, inflate: f64) -> Self {
        Self::from_points(&mesh.to_vertices_and_faces().0, inflate)
    }

    /// Construct the tight box of the points grown by inflate.
    pub fn from_pointcloud(pointcloud: &PointCloud, inflate: f64) -> Self {
        Self::from_points(&pointcloud.get_points(), inflate)
    }

    /// Construct the box of the control points, or of the curve extrema when tight.
    pub fn from_nurbscurve(curve: &NurbsCurve, inflate: f64, tight: bool) -> Self {
        if !curve.is_valid() || curve.cv_count() == 0 {
            return AABB::default();
        }

        let mut points = Vec::new();

        if !tight {
            for i in 0..curve.cv_count() {
                if let Some(pt) = curve.get_cv(i) {
                    points.push(pt);
                }
            }

            return Self::from_points(&points, inflate);
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

        let dt = (t1 - t0) / NUM_SAMPLES as f64;

        for axis in 0..3 {
            for i in 0..NUM_SAMPLES {
                let t_start = t0 + i as f64 * dt;
                let t_end = t_start + dt;
                let deriv_start: Vec<Vector> = curve.evaluate(t_start, 1);
                let deriv_end: Vec<Vector> = curve.evaluate(t_end, 1);

                if deriv_start.len() < 2 || deriv_end.len() < 2 {
                    continue;
                }

                let d_start = deriv_start[1][axis];
                let d_end = deriv_end[1][axis];

                if d_start * d_end < 0.0 {
                    let t_root = Self::compute_extremum(curve, axis, t_start, t_end, d_start);
                    points.push(curve.point_at(t_root));
                }
            }
        }

        Self::from_points(&points, inflate)
    }

    /// Construct the box of the control points grown by inflate.
    pub fn from_nurbssurface(surface: &NurbsSurface, inflate: f64) -> Self {
        if !surface.is_valid() || surface.cv_count(0) == 0 || surface.cv_count(1) == 0 {
            return AABB::default();
        }

        let mut points = Vec::new();

        for i in 0..surface.cv_count(0) {
            for j in 0..surface.cv_count(1) {
                if let Some(pt) = surface.get_cv(i, j) {
                    points.push(pt);
                }
            }
        }

        Self::from_points(&points, inflate)
    }

    /// Construct the box enclosing both a and b; an invalid box contributes nothing.
    #[inline(always)]
    pub fn merge(a: &AABB, b: &AABB) -> AABB {
        if !a.is_valid() {
            return *b;
        }

        if !b.is_valid() {
            return *a;
        }

        let min_x = (a.cx - a.hx).min(b.cx - b.hx);
        let min_y = (a.cy - a.hy).min(b.cy - b.hy);
        let min_z = (a.cz - a.hz).min(b.cz - b.hz);
        let max_x = (a.cx + a.hx).max(b.cx + b.hx);
        let max_y = (a.cy + a.hy).max(b.cy + b.hy);
        let max_z = (a.cz + a.hz).max(b.cz + b.hz);

        AABB::new(
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
            (max_x - min_x) * 0.5,
            (max_y - min_y) * 0.5,
            (max_z - min_z) * 0.5,
        )
    }

    /// Construct the box nothing has grown yet: negative half-sizes, so is_valid is false.
    pub fn empty() -> Self {
        AABB::new(0.0, 0.0, 0.0, -1.0, -1.0, -1.0)
    }

    /// Return the parameter in [t_lo, t_hi] where the axis derivative crosses zero, by Newton steps bracketed by bisection.
    fn compute_extremum(
        curve: &NurbsCurve,
        axis: usize,
        mut t_lo: f64,
        mut t_hi: f64,
        mut d_start: f64,
    ) -> f64 {
        let mut t_root = (t_lo + t_hi) * 0.5;

        for _ in 0..MAX_ITER {
            let deriv: Vec<Vector> = curve.evaluate(t_root, 2);

            if deriv.len() < 3 {
                break;
            }

            let d1 = deriv[1][axis];
            let d2 = deriv[2][axis];

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

            let d_check = deriv_check[1][axis];

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

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
impl PartialEq for AABB {
    /// Compare center and half-size to 1e-6.
    fn eq(&self, other: &Self) -> bool {
        (self.cx * 1000000.0).round() == (other.cx * 1000000.0).round()
            && (self.cy * 1000000.0).round() == (other.cy * 1000000.0).round()
            && (self.cz * 1000000.0).round() == (other.cz * 1000000.0).round()
            && (self.hx * 1000000.0).round() == (other.hx * 1000000.0).round()
            && (self.hy * 1000000.0).round() == (other.hy * 1000000.0).round()
            && (self.hz * 1000000.0).round() == (other.hz * 1000000.0).round()
    }
}

impl AABB {
    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the min corner.
    pub fn min_point(&self) -> Point {
        Point::new(self.cx - self.hx, self.cy - self.hy, self.cz - self.hz)
    }

    /// Return the max corner.
    pub fn max_point(&self) -> Point {
        Point::new(self.cx + self.hx, self.cy + self.hy, self.cz + self.hz)
    }

    /// Return the center.
    pub fn center(&self) -> Point {
        Point::new(self.cx, self.cy, self.cz)
    }

    /// Return the surface area.
    pub fn area(&self) -> f64 {
        8.0 * (self.hx * self.hy + self.hy * self.hz + self.hz * self.hx)
    }

    /// Return the length of the space diagonal, 0 when invalid.
    pub fn diagonal(&self) -> f64 {
        if !self.is_valid() {
            return 0.0;
        }

        2.0 * (self.hx * self.hx + self.hy * self.hy + self.hz * self.hz).sqrt()
    }

    /// Return the volume.
    pub fn volume(&self) -> f64 {
        8.0 * self.hx * self.hy * self.hz
    }

    /// Return whether no half-size is negative.
    pub fn is_valid(&self) -> bool {
        self.hx >= 0.0 && self.hy >= 0.0 && self.hz >= 0.0
    }

    /// Return pt clamped to the box.
    pub fn closest_point(&self, pt: &Point) -> Point {
        Point::new(
            (self.cx - self.hx).max((self.cx + self.hx).min(pt[0])),
            (self.cy - self.hy).max((self.cy + self.hy).min(pt[1])),
            (self.cz - self.hz).max((self.cz + self.hz).min(pt[2])),
        )
    }

    /// Return whether pt is inside or on the box.
    pub fn contains(&self, pt: &Point) -> bool {
        pt[0] >= self.cx - self.hx
            && pt[0] <= self.cx + self.hx
            && pt[1] >= self.cy - self.hy
            && pt[1] <= self.cy + self.hy
            && pt[2] >= self.cz - self.hz
            && pt[2] <= self.cz + self.hz
    }

    /// Return whether the boxes overlap or touch.
    #[inline(always)]
    pub fn intersects(&self, other: &AABB) -> bool {
        self.cx - self.hx <= other.cx + other.hx
            && self.cx + self.hx >= other.cx - other.hx
            && self.cy - self.hy <= other.cy + other.hy
            && self.cy + self.hy >= other.cy - other.hy
            && self.cz - self.hz <= other.cz + other.hz
            && self.cz + self.hz >= other.cz - other.hz
    }

    /// Return the corner picked by the sign of each half-size.
    pub fn corner(&self, x_max: bool, y_max: bool, z_max: bool) -> Point {
        Point::new(
            self.cx + if x_max { self.hx } else { -self.hx },
            self.cy + if y_max { self.hy } else { -self.hy },
            self.cz + if z_max { self.hz } else { -self.hz },
        )
    }

    /// Return the bottom loop then top loop, counter-clockwise from +x+y.
    pub fn corners(&self) -> [Point; 8] {
        [
            Point::new(self.cx + self.hx, self.cy + self.hy, self.cz - self.hz),
            Point::new(self.cx - self.hx, self.cy + self.hy, self.cz - self.hz),
            Point::new(self.cx - self.hx, self.cy - self.hy, self.cz - self.hz),
            Point::new(self.cx + self.hx, self.cy - self.hy, self.cz - self.hz),
            Point::new(self.cx + self.hx, self.cy + self.hy, self.cz + self.hz),
            Point::new(self.cx - self.hx, self.cy + self.hy, self.cz + self.hz),
            Point::new(self.cx - self.hx, self.cy - self.hy, self.cz + self.hz),
            Point::new(self.cx + self.hx, self.cy - self.hy, self.cz + self.hz),
        ]
    }

    /// Return the bottom loop then top loop, counter-clockwise from +x+y.
    pub fn get_corners(&self) -> [Point; 8] {
        self.corners()
    }

    /// Return the bottom loop, top loop, then the four verticals.
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

    /// Return the center offset by x, y, z.
    pub fn point_at(&self, x: f64, y: f64, z: f64) -> Point {
        Point::new(self.cx + x, self.cy + y, self.cz + z)
    }

    /// Grow every half-size by amount.
    pub fn inflate(&mut self, amount: f64) {
        self.hx += amount;
        self.hy += amount;
        self.hz += amount;
    }

    /// Grow to enclose other; an invalid box contributes nothing.
    pub fn union_with(&mut self, other: &AABB) {
        *self = AABB::merge(self, other);
    }

    /// Grow to enclose (x, y, z); coordinates, not a Point, so a vertex loop allocates nothing.
    pub fn union_with_point(&mut self, x: f64, y: f64, z: f64) {
        *self = AABB::merge(self, &AABB::new(x, y, z, 0.0, 0.0, 0.0));
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Replace the box by the box of its eight transformed corners.
    pub fn transform(&mut self, xform: &Xform) {
        *self = self.transformed(xform);
    }

    /// Return the box of the eight transformed corners; an invalid box stays invalid.
    pub fn transformed(&self, xform: &Xform) -> AABB {
        if !self.is_valid() {
            return *self;
        }

        let mut out = AABB::empty();

        for point in self.corners() {
            let moved = xform.transform_point(&point);
            out.union_with_point(moved[0], moved[1], moved[2]);
        }

        out
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "cx, cy, cz, hx, hy, hz".
    pub fn str(&self) -> String {
        let prec = Tolerance::ROUNDING;

        format!(
            "{}, {}, {}, {}, {}, {}",
            TOLERANCE.format_number(self.cx, prec),
            TOLERANCE.format_number(self.cy, prec),
            TOLERANCE.format_number(self.cz, prec),
            TOLERANCE.format_number(self.hx, prec),
            TOLERANCE.format_number(self.hy, prec),
            TOLERANCE.format_number(self.hz, prec)
        )
    }

    /// Return "AABB(cx, cy, cz, hx, hy, hz)".
    pub fn repr(&self) -> String {
        format!("AABB({})", self.str())
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SESSION_VIEWER
    // ═══════════════════════════════════════════════════════════════════════════
    /// Returns the 8 box corners as f32 [x, y, z] rows for a wireframe-box buffer, same winding as corners.
    pub fn corners_f32(&self) -> [[f32; 3]; 8] {
        self.corners().map(|p| p.to_f32())
    }
}

impl fmt::Display for AABB {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}
