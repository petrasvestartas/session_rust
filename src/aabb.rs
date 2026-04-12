// AABBTree — flat contiguous BVH over axis-aligned boxes (SAH median split).
// Use for: closest-point on static mesh faces, ray-mesh intersection.
//   Build once, query many times. Cache-friendly 56-byte nodes.
// Prefer over BVH  when geometry is static and all volumes are world-aligned.
// Prefer over RTree when no dynamic insert/delete is needed.
// Prefer over KDTree when querying faces/volumes, not bare point clouds.
use crate::Point;

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
        AABB { cx, cy, cz, hx, hy, hz }
    }

    pub fn from_point(point: &Point, inflate: f64) -> Self {
        AABB { cx: point[0], cy: point[1], cz: point[2], hx: inflate, hy: inflate, hz: inflate }
    }

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
        AABB {
            cx: (min_x + max_x) * 0.5,
            cy: (min_y + max_y) * 0.5,
            cz: (min_z + max_z) * 0.5,
            hx: (max_x - min_x) * 0.5 + inflate,
            hy: (max_y - min_y) * 0.5 + inflate,
            hz: (max_z - min_z) * 0.5 + inflate,
        }
    }

    /// Build an AABB directly from a stride-3 coord buffer (e.g. `Polyline::coords`)
    /// without constructing an intermediate `Vec<Point>`. Used on hot paths like
    /// `Session::add_polyline` where the caller already has raw coords.
    pub fn from_coords_stride3(coords: &[f64], inflate: f64) -> Self {
        if coords.len() < 3 {
            return AABB::default();
        }
        let n = coords.len() / 3;
        let mut min_x = coords[0];
        let mut min_y = coords[1];
        let mut min_z = coords[2];
        let mut max_x = min_x;
        let mut max_y = min_y;
        let mut max_z = min_z;
        for i in 1..n {
            let x = coords[i * 3];
            let y = coords[i * 3 + 1];
            let z = coords[i * 3 + 2];
            if x < min_x { min_x = x; } else if x > max_x { max_x = x; }
            if y < min_y { min_y = y; } else if y > max_y { max_y = y; }
            if z < min_z { min_z = z; } else if z > max_z { max_z = z; }
        }
        AABB {
            cx: (min_x + max_x) * 0.5,
            cy: (min_y + max_y) * 0.5,
            cz: (min_z + max_z) * 0.5,
            hx: (max_x - min_x) * 0.5 + inflate,
            hy: (max_y - min_y) * 0.5 + inflate,
            hz: (max_z - min_z) * 0.5 + inflate,
        }
    }

    pub fn from_line(line: &crate::line::Line, inflate: f64) -> Self {
        let points = vec![line.start(), line.end()];
        Self::from_points(&points, inflate)
    }

    pub fn from_polyline(polyline: &crate::polyline::Polyline, inflate: f64) -> Self {
        Self::from_points(&polyline.get_points(), inflate)
    }

    pub fn from_mesh(mesh: &crate::mesh::Mesh, inflate: f64) -> Self {
        let (vertices, _) = mesh.to_vertices_and_faces();
        Self::from_points(&vertices, inflate)
    }

    pub fn from_pointcloud(pointcloud: &crate::pointcloud::PointCloud, inflate: f64) -> Self {
        Self::from_points(&pointcloud.get_points(), inflate)
    }

    pub fn from_nurbscurve(curve: &crate::nurbscurve::NurbsCurve, inflate: f64, tight: bool) -> Self {
        if !curve.is_valid() || curve.cv_count() == 0 {
            return AABB::default();
        }
        if !tight {
            let points: Vec<Point> = (0..curve.cv_count())
                .filter_map(|i| curve.get_cv(i))
                .collect();
            return Self::from_points(&points, inflate);
        }
        let (t0, t1) = curve.domain();
        let mut extrema_points = vec![curve.point_at(t0), curve.point_at(t1)];
        for t in curve.get_span_vector() {
            if t > t0 && t < t1 {
                extrema_points.push(curve.point_at(t));
            }
        }
        const NUM_SAMPLES: usize = 20;
        let dt = (t1 - t0) / NUM_SAMPLES as f64;
        for axis in 0..3 {
            for i in 0..NUM_SAMPLES {
                let t_start = t0 + i as f64 * dt;
                let t_end = t_start + dt;
                let deriv_start = curve.evaluate(t_start, 1);
                let deriv_end = curve.evaluate(t_end, 1);
                if deriv_start.len() < 2 || deriv_end.len() < 2 {
                    continue;
                }
                let mut d_start = deriv_start[1][axis];
                let d_end = deriv_end[1][axis];
                if d_start * d_end < 0.0 {
                    let mut t_lo = t_start;
                    let mut t_hi = t_end;
                    let mut t_root = (t_lo + t_hi) * 0.5;
                    for _ in 0..20 {
                        let deriv = curve.evaluate(t_root, 2);
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
                        let deriv_check = curve.evaluate(t_root, 1);
                        if deriv_check.len() >= 2 {
                            let f_check = deriv_check[1][axis];
                            if f_check * d_start < 0.0 {
                                t_hi = t_root;
                            } else {
                                t_lo = t_root;
                                d_start = f_check;
                            }
                        }
                    }
                    extrema_points.push(curve.point_at(t_root));
                }
            }
        }
        Self::from_points(&extrema_points, inflate)
    }

    pub fn from_nurbssurface(surface: &crate::nurbssurface::NurbsSurface, inflate: f64) -> Self {
        if !surface.is_valid() || surface.cv_count_dir(Some(0)) == 0 || surface.cv_count_dir(Some(1)) == 0 {
            return AABB::default();
        }
        let mut points = Vec::new();
        for i in 0..surface.cv_count_dir(Some(0)) {
            for j in 0..surface.cv_count_dir(Some(1)) {
                if let Some(pt) = surface.get_cv(i, j) {
                    points.push(pt);
                }
            }
        }
        Self::from_points(&points, inflate)
    }

    pub fn min_point(&self) -> Point {
        Point::new(self.cx - self.hx, self.cy - self.hy, self.cz - self.hz)
    }

    pub fn max_point(&self) -> Point {
        Point::new(self.cx + self.hx, self.cy + self.hy, self.cz + self.hz)
    }

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

    pub fn center(&self) -> Point {
        Point::new(self.cx, self.cy, self.cz)
    }

    pub fn area(&self) -> f64 {
        8.0 * (self.hx * self.hy + self.hy * self.hz + self.hz * self.hx)
    }

    pub fn diagonal(&self) -> f64 {
        2.0 * (self.hx * self.hx + self.hy * self.hy + self.hz * self.hz).sqrt()
    }

    pub fn is_valid(&self) -> bool {
        self.hx >= 0.0 && self.hy >= 0.0 && self.hz >= 0.0
    }

    pub fn volume(&self) -> f64 {
        8.0 * self.hx * self.hy * self.hz
    }

    pub fn closest_point(&self, pt: &Point) -> Point {
        let x = pt[0].max(self.cx - self.hx).min(self.cx + self.hx);
        let y = pt[1].max(self.cy - self.hy).min(self.cy + self.hy);
        let z = pt[2].max(self.cz - self.hz).min(self.cz + self.hz);
        Point::new(x, y, z)
    }

    pub fn contains(&self, pt: &Point) -> bool {
        pt[0] >= self.cx - self.hx && pt[0] <= self.cx + self.hx
            && pt[1] >= self.cy - self.hy && pt[1] <= self.cy + self.hy
            && pt[2] >= self.cz - self.hz && pt[2] <= self.cz + self.hz
    }

    pub fn corner(&self, x_max: bool, y_max: bool, z_max: bool) -> Point {
        Point::new(
            self.cx + if x_max { self.hx } else { -self.hx },
            self.cy + if y_max { self.hy } else { -self.hy },
            self.cz + if z_max { self.hz } else { -self.hz },
        )
    }

    pub fn get_corners(&self) -> [Point; 8] {
        self.corners()
    }

    pub fn get_edges(&self) -> Vec<crate::line::Line> {
        let c = self.corners();
        vec![
            crate::line::Line::new(c[0][0], c[0][1], c[0][2], c[1][0], c[1][1], c[1][2]),
            crate::line::Line::new(c[1][0], c[1][1], c[1][2], c[2][0], c[2][1], c[2][2]),
            crate::line::Line::new(c[2][0], c[2][1], c[2][2], c[3][0], c[3][1], c[3][2]),
            crate::line::Line::new(c[3][0], c[3][1], c[3][2], c[0][0], c[0][1], c[0][2]),
            crate::line::Line::new(c[4][0], c[4][1], c[4][2], c[5][0], c[5][1], c[5][2]),
            crate::line::Line::new(c[5][0], c[5][1], c[5][2], c[6][0], c[6][1], c[6][2]),
            crate::line::Line::new(c[6][0], c[6][1], c[6][2], c[7][0], c[7][1], c[7][2]),
            crate::line::Line::new(c[7][0], c[7][1], c[7][2], c[4][0], c[4][1], c[4][2]),
            crate::line::Line::new(c[0][0], c[0][1], c[0][2], c[4][0], c[4][1], c[4][2]),
            crate::line::Line::new(c[1][0], c[1][1], c[1][2], c[5][0], c[5][1], c[5][2]),
            crate::line::Line::new(c[2][0], c[2][1], c[2][2], c[6][0], c[6][1], c[6][2]),
            crate::line::Line::new(c[3][0], c[3][1], c[3][2], c[7][0], c[7][1], c[7][2]),
        ]
    }

    pub fn point_at(&self, x: f64, y: f64, z: f64) -> Point {
        Point::new(self.cx + x, self.cy + y, self.cz + z)
    }

    pub fn union_with(&mut self, other: &AABB) {
        let min_x = (self.cx - self.hx).min(other.cx - other.hx);
        let min_y = (self.cy - self.hy).min(other.cy - other.hy);
        let min_z = (self.cz - self.hz).min(other.cz - other.hz);
        let max_x = (self.cx + self.hx).max(other.cx + other.hx);
        let max_y = (self.cy + self.hy).max(other.cy + other.hy);
        let max_z = (self.cz + self.hz).max(other.cz + other.hz);
        self.cx = (min_x + max_x) * 0.5; self.hx = (max_x - min_x) * 0.5;
        self.cy = (min_y + max_y) * 0.5; self.hy = (max_y - min_y) * 0.5;
        self.cz = (min_z + max_z) * 0.5; self.hz = (max_z - min_z) * 0.5;
    }

    pub fn inflate(&mut self, amount: f64) {
        self.hx += amount;
        self.hy += amount;
        self.hz += amount;
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

    #[inline(always)]
    pub fn merge(a: AABB, b: AABB) -> AABB {
        let min_x = (a.cx - a.hx).min(b.cx - b.hx);
        let min_y = (a.cy - a.hy).min(b.cy - b.hy);
        let min_z = (a.cz - a.hz).min(b.cz - b.hz);
        let max_x = (a.cx + a.hx).max(b.cx + b.hx);
        let max_y = (a.cy + a.hy).max(b.cy + b.hy);
        let max_z = (a.cz + a.hz).max(b.cz + b.hz);
        AABB {
            cx: (min_x + max_x) * 0.5,
            cy: (min_y + max_y) * 0.5,
            cz: (min_z + max_z) * 0.5,
            hx: (max_x - min_x) * 0.5,
            hy: (max_y - min_y) * 0.5,
            hz: (max_z - min_z) * 0.5,
        }
    }
}
