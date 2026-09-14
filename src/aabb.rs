use crate::tolerance::Tolerance;
use crate::tolerance::TOLERANCE;
use crate::{Line, Mesh, NurbsCurve, NurbsSurface, Point, PointCloud, Polyline, Vector};
use std::fmt;

const NUM_SAMPLES: usize = 20;
const MAX_ITER: usize = 20;

/// Axis-aligned bounding box as center and half-size
#[derive(Clone, Copy, Default, Debug)]
pub struct AABB {
    pub cx: f64,
    pub cy: f64,
    pub cz: f64,
    pub hx: f64,
    pub hy: f64,
    pub hz: f64,
}

impl AABB {
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

    /// Box of half-size inflate around point
    pub fn from_point(point: &Point, inflate: f64) -> Self {
        AABB::new(point[0], point[1], point[2], inflate, inflate, inflate)
    }

    /// Tight box of points grown by inflate
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

    /// Tight box of the two ends grown by inflate
    pub fn from_line(line: &Line, inflate: f64) -> Self {
        Self::from_points(&[line.start(), line.end()], inflate)
    }

    /// Tight box of the vertices grown by inflate
    pub fn from_polyline(polyline: &Polyline, inflate: f64) -> Self {
        Self::from_points(&polyline.get_points(), inflate)
    }

    /// Tight box of the vertices grown by inflate
    pub fn from_mesh(mesh: &Mesh, inflate: f64) -> Self {
        let (vertices, _faces) = mesh.to_vertices_and_faces();
        Self::from_points(&vertices, inflate)
    }

    /// Tight box of the points grown by inflate
    pub fn from_pointcloud(pointcloud: &PointCloud, inflate: f64) -> Self {
        Self::from_points(&pointcloud.get_points(), inflate)
    }

    /// Box of the control points, or of the curve extrema when tight
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
        let (t0, t1) = curve.domain();
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

    /// Box of the control points grown by inflate
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

    /// Box enclosing both a and b
    #[inline(always)]
    pub fn merge(a: &AABB, b: &AABB) -> AABB {
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

    /// Parameter in [t_lo, t_hi] where the axis derivative crosses zero, by Newton steps bracketed by bisection
    fn compute_extremum(
        curve: &NurbsCurve,
        axis: usize,
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
            let f = deriv[1][axis];
            let fp = deriv[2][axis];
            if f.abs() < 1e-12 {
                break;
            }
            if fp.abs() > 1e-14 {
                let t_new = t_root - f / fp;
                if t_new >= t_lo && t_new <= t_hi {
                    t_root = t_new;
                } else {
                    if f * d_start < 0.0 {
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
            let f_check = deriv_check[1][axis];
            if f_check * d_start < 0.0 {
                t_hi = t_root;
            } else {
                t_lo = t_root;
                d_start = f_check;
            }
        }
        t_root
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn min_point(&self) -> Point {
        Point::new(self.cx - self.hx, self.cy - self.hy, self.cz - self.hz)
    }

    pub fn max_point(&self) -> Point {
        Point::new(self.cx + self.hx, self.cy + self.hy, self.cz + self.hz)
    }

    pub fn center(&self) -> Point {
        Point::new(self.cx, self.cy, self.cz)
    }

    /// Surface area
    pub fn area(&self) -> f64 {
        8.0 * (self.hx * self.hy + self.hy * self.hz + self.hz * self.hx)
    }

    /// Length of the space diagonal
    pub fn diagonal(&self) -> f64 {
        2.0 * (self.hx * self.hx + self.hy * self.hy + self.hz * self.hz).sqrt()
    }

    pub fn volume(&self) -> f64 {
        8.0 * self.hx * self.hy * self.hz
    }

    /// No negative half-size
    pub fn is_valid(&self) -> bool {
        self.hx >= 0.0 && self.hy >= 0.0 && self.hz >= 0.0
    }

    /// pt clamped to the box
    pub fn closest_point(&self, pt: &Point) -> Point {
        let x = (self.cx - self.hx).max((self.cx + self.hx).min(pt[0]));
        let y = (self.cy - self.hy).max((self.cy + self.hy).min(pt[1]));
        let z = (self.cz - self.hz).max((self.cz + self.hz).min(pt[2]));
        Point::new(x, y, z)
    }

    pub fn contains(&self, pt: &Point) -> bool {
        pt[0] >= self.cx - self.hx
            && pt[0] <= self.cx + self.hx
            && pt[1] >= self.cy - self.hy
            && pt[1] <= self.cy + self.hy
            && pt[2] >= self.cz - self.hz
            && pt[2] <= self.cz + self.hz
    }

    #[inline(always)]
    pub fn intersects(&self, other: &AABB) -> bool {
        self.cx - self.hx <= other.cx + other.hx
            && self.cx + self.hx >= other.cx - other.hx
            && self.cy - self.hy <= other.cy + other.hy
            && self.cy + self.hy >= other.cy - other.hy
            && self.cz - self.hz <= other.cz + other.hz
            && self.cz + self.hz >= other.cz - other.hz
    }

    /// Corner picked by the sign of each half-size
    pub fn corner(&self, x_max: bool, y_max: bool, z_max: bool) -> Point {
        Point::new(
            self.cx + if x_max { self.hx } else { -self.hx },
            self.cy + if y_max { self.hy } else { -self.hy },
            self.cz + if z_max { self.hz } else { -self.hz },
        )
    }

    /// Bottom loop then top loop, counter-clockwise from +x+y
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

    pub fn get_corners(&self) -> [Point; 8] {
        self.corners()
    }

    /// Bottom loop, top loop, then the four verticals
    pub fn get_edges(&self) -> Vec<Line> {
        let c = self.corners();
        vec![
            Line::from_points(&c[0], &c[1]),
            Line::from_points(&c[1], &c[2]),
            Line::from_points(&c[2], &c[3]),
            Line::from_points(&c[3], &c[0]),
            Line::from_points(&c[4], &c[5]),
            Line::from_points(&c[5], &c[6]),
            Line::from_points(&c[6], &c[7]),
            Line::from_points(&c[7], &c[4]),
            Line::from_points(&c[0], &c[4]),
            Line::from_points(&c[1], &c[5]),
            Line::from_points(&c[2], &c[6]),
            Line::from_points(&c[3], &c[7]),
        ]
    }

    /// Center offset by x, y, z
    pub fn point_at(&self, x: f64, y: f64, z: f64) -> Point {
        Point::new(self.cx + x, self.cy + y, self.cz + z)
    }

    /// Grow every half-size by amount
    pub fn inflate(&mut self, amount: f64) {
        self.hx += amount;
        self.hy += amount;
        self.hz += amount;
    }

    /// Grow to enclose other
    pub fn union_with(&mut self, other: &AABB) {
        *self = AABB::merge(self, other);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════

    /// "cx, cy, cz, hx, hy, hz"
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

    /// "AABB(cx, cy, cz, hx, hy, hz)"
    pub fn repr(&self) -> String {
        format!("AABB({})", self.str())
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SESSION_VIEWER
    // ═══════════════════════════════════════════════════════════════════════════

    /// The 8 box corners as f32 `[x, y, z]` rows — ready for a wireframe-box vertex/segment
    /// buffer. Same winding as [`corners`](Self::corners); the kernel keeps f64.
    pub fn corners_f32(&self) -> [[f32; 3]; 8] {
        self.corners().map(|p| p.to_f32())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

/// Center and half-size to 1e-6
impl PartialEq for AABB {
    fn eq(&self, other: &Self) -> bool {
        (self.cx * 1000000.0).round() == (other.cx * 1000000.0).round()
            && (self.cy * 1000000.0).round() == (other.cy * 1000000.0).round()
            && (self.cz * 1000000.0).round() == (other.cz * 1000000.0).round()
            && (self.hx * 1000000.0).round() == (other.hx * 1000000.0).round()
            && (self.hy * 1000000.0).round() == (other.hy * 1000000.0).round()
            && (self.hz * 1000000.0).round() == (other.hz * 1000000.0).round()
    }
}

impl fmt::Display for AABB {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}
