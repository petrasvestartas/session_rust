use crate::{AABB, Plane, Point, Vector, Xform};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "OBB")]
pub struct OBB {
    pub center: Point,
    pub x_axis: Vector,
    pub y_axis: Vector,
    pub z_axis: Vector,
    pub half_size: Vector,
    #[serde(serialize_with = "crate::guid_serde::serialize", deserialize_with = "crate::guid_serde::deserialize")]
    guid: std::sync::OnceLock<String>,
    pub name: String,
}

impl OBB {
    pub fn new(
        center: Point,
        x_axis: Vector,
        y_axis: Vector,
        z_axis: Vector,
        half_size: Vector,
    ) -> Self {
        OBB {
            center,
            x_axis,
            y_axis,
            z_axis,
            half_size,
            guid: std::sync::OnceLock::new(),
            name: "my_obb".to_string(),
        }
    }

    pub fn from_plane(plane: &Plane, dx: f64, dy: f64, dz: f64) -> Self {
        OBB {
            center: plane.origin(),
            x_axis: plane.x_axis(),
            y_axis: plane.y_axis(),
            z_axis: plane.z_axis(),
            half_size: Vector::new(dx * 0.5, dy * 0.5, dz * 0.5),
            guid: std::sync::OnceLock::new(),
            name: String::new(),
        }
    }

    pub fn from_point(point: Point, inflate: f64) -> Self {
        Self::from_aabb(AABB::from_point(&point, inflate))
    }

    pub fn from_points(points: &[Point], inflate: f64) -> Self {
        Self::from_aabb(AABB::from_points(points, inflate))
    }

    pub fn from_line(line: &crate::line::Line, inflate: f64) -> Self {
        Self::from_aabb(AABB::from_line(line, inflate))
    }

    pub fn from_polyline(polyline: &crate::polyline::Polyline, inflate: f64) -> Self {
        Self::from_aabb(AABB::from_polyline(polyline, inflate))
    }

    pub fn from_aabb(a: AABB) -> Self {
        OBB {
            center: Point::new(a.cx, a.cy, a.cz),
            x_axis: Vector::new(1.0, 0.0, 0.0),
            y_axis: Vector::new(0.0, 1.0, 0.0),
            z_axis: Vector::new(0.0, 0.0, 1.0),
            half_size: Vector::new(a.hx, a.hy, a.hz),
            guid: std::sync::OnceLock::new(),
            name: String::new(),
        }
    }

    pub fn from_nurbscurve(curve: &crate::nurbscurve::NurbsCurve, inflate: f64, tight: bool) -> Self {
        Self::from_aabb(AABB::from_nurbscurve(curve, inflate, tight))
    }

    pub fn from_nurbscurve_with_plane(
        curve: &crate::nurbscurve::NurbsCurve,
        plane: &Plane,
        inflate: f64,
        tight: bool,
    ) -> Self {
        if !curve.is_valid() || curve.cv_count() == 0 {
            return OBB::default();
        }

        if !tight {
            let points: Vec<Point> = (0..curve.cv_count())
                .filter_map(|i| curve.get_cv(i))
                .collect();
            return Self::from_points_with_plane(&points, plane, inflate);
        }

        let (t0, t1) = curve.domain();
        let mut extrema_points = vec![curve.point_at(t0), curve.point_at(t1)];

        let spans = curve.get_span_vector();
        for t in spans {
            if t > t0 && t < t1 {
                extrema_points.push(curve.point_at(t));
            }
        }

        let axes = [plane.x_axis(), plane.y_axis(), plane.z_axis()];
        const NUM_SAMPLES: usize = 20;
        let dt = (t1 - t0) / NUM_SAMPLES as f64;

        for axis in &axes {
            for i in 0..NUM_SAMPLES {
                let t_start = t0 + i as f64 * dt;
                let t_end = t_start + dt;

                let deriv_start = curve.evaluate(t_start, 1);
                let deriv_end = curve.evaluate(t_end, 1);
                if deriv_start.len() < 2 || deriv_end.len() < 2 {
                    continue;
                }

                let mut d_start = deriv_start[1].dot(axis);
                let d_end = deriv_end[1].dot(axis);

                if d_start * d_end < 0.0 {
                    let mut t_lo = t_start;
                    let mut t_hi = t_end;
                    let mut t_root = (t_lo + t_hi) * 0.5;

                    for _ in 0..20 {
                        let deriv = curve.evaluate(t_root, 2);
                        if deriv.len() < 3 {
                            break;
                        }

                        let f = deriv[1].dot(axis);
                        let fp = deriv[2].dot(axis);

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
                            let f_check = deriv_check[1].dot(axis);
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

        Self::from_points_with_plane(&extrema_points, plane, inflate)
    }

    pub fn from_points_with_plane(points: &[Point], plane: &Plane, inflate: f64) -> Self {
        if points.is_empty() {
            return OBB::default();
        }

        let origin = plane.origin();
        let x_axis = plane.x_axis();
        let y_axis = plane.y_axis();
        let z_axis = plane.z_axis();
        // plane_to_xy stores the basis as matrix COLUMNS, so it rotates local->world in
        // both directions and its forward call yields the inverse frame for every plane
        // that is not axis-aligned. world_to_frame / frame_to_world are the real pair.
        let world_to_local = Xform::world_to_frame(&origin, &x_axis, &y_axis, &z_axis);
        let local_to_world = Xform::frame_to_world(&origin, &x_axis, &y_axis, &z_axis);

        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut min_z = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;
        let mut max_z = f64::MIN;

        for pt in points {
            let local_pt = pt.transformed(&world_to_local);
            min_x = min_x.min(local_pt[0]);
            min_y = min_y.min(local_pt[1]);
            min_z = min_z.min(local_pt[2]);
            max_x = max_x.max(local_pt[0]);
            max_y = max_y.max(local_pt[1]);
            max_z = max_z.max(local_pt[2]);
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

        let world_center = local_center.transformed(&local_to_world);

        OBB {
            center: world_center,
            x_axis,
            y_axis,
            z_axis,
            half_size,
            guid: std::sync::OnceLock::new(),
            name: String::new(),
        }
    }

    pub fn from_mesh(mesh: &crate::mesh::Mesh, inflate: f64) -> Self {
        Self::from_aabb(AABB::from_mesh(mesh, inflate))
    }

    pub fn from_mesh_with_plane(mesh: &crate::mesh::Mesh, plane: &Plane, inflate: f64) -> Self {
        let (vertices, _) = mesh.to_vertices_and_faces();
        Self::from_points_with_plane(&vertices, plane, inflate)
    }

    pub fn from_pointcloud(pointcloud: &crate::pointcloud::PointCloud, inflate: f64) -> Self {
        Self::from_aabb(AABB::from_pointcloud(pointcloud, inflate))
    }

    pub fn from_pointcloud_with_plane(pointcloud: &crate::pointcloud::PointCloud, plane: &Plane, inflate: f64) -> Self {
        Self::from_points_with_plane(&pointcloud.get_points(), plane, inflate)
    }

    pub fn from_nurbssurface(surface: &crate::nurbssurface::NurbsSurface, inflate: f64) -> Self {
        Self::from_aabb(AABB::from_nurbssurface(surface, inflate))
    }

    pub fn from_nurbssurface_with_plane(surface: &crate::nurbssurface::NurbsSurface, plane: &Plane, inflate: f64) -> Self {
        if !surface.is_valid() || surface.cv_count_dir(Some(0)) == 0 || surface.cv_count_dir(Some(1)) == 0 {
            return OBB::default();
        }
        let mut points = Vec::new();
        for i in 0..surface.cv_count_dir(Some(0)) {
            for j in 0..surface.cv_count_dir(Some(1)) {
                if let Some(pt) = surface.get_cv(i, j) {
                    points.push(pt);
                }
            }
        }
        Self::from_points_with_plane(&points, plane, inflate)
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    pub fn aabb(&self) -> AABB {
        let ex = self.half_size[0];
        let ey = self.half_size[1];
        let ez = self.half_size[2];
        let hx = self.x_axis[0].abs() * ex + self.y_axis[0].abs() * ey + self.z_axis[0].abs() * ez;
        let hy = self.x_axis[1].abs() * ex + self.y_axis[1].abs() * ey + self.z_axis[1].abs() * ez;
        let hz = self.x_axis[2].abs() * ex + self.y_axis[2].abs() * ey + self.z_axis[2].abs() * ez;
        AABB::new(self.center[0], self.center[1], self.center[2], hx, hy, hz)
    }

    pub fn point_at(&self, x: f64, y: f64, z: f64) -> Point {
        Point::new(
            self.center[0] + x * self.x_axis[0] + y * self.y_axis[0] + z * self.z_axis[0],
            self.center[1] + x * self.x_axis[1] + y * self.y_axis[1] + z * self.z_axis[1],
            self.center[2] + x * self.x_axis[2] + y * self.y_axis[2] + z * self.z_axis[2],
        )
    }

    pub fn min_point(&self) -> Point {
        self.aabb().min_point()
    }

    pub fn max_point(&self) -> Point {
        self.aabb().max_point()
    }

    pub fn corners(&self) -> [Point; 8] {
        [
            self.point_at(self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(-self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(
                -self.half_size[0],
                -self.half_size[1],
                -self.half_size[2],
            ),
            self.point_at(self.half_size[0], -self.half_size[1], -self.half_size[2]),
            self.point_at(self.half_size[0], self.half_size[1], self.half_size[2]),
            self.point_at(-self.half_size[0], self.half_size[1], self.half_size[2]),
            self.point_at(-self.half_size[0], -self.half_size[1], self.half_size[2]),
            self.point_at(self.half_size[0], -self.half_size[1], self.half_size[2]),
        ]
    }

    pub fn two_rectangles(&self) -> [Point; 10] {
        [
            self.point_at(self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(-self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(
                -self.half_size[0],
                -self.half_size[1],
                -self.half_size[2],
            ),
            self.point_at(self.half_size[0], -self.half_size[1], -self.half_size[2]),
            self.point_at(self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(self.half_size[0], self.half_size[1], self.half_size[2]),
            self.point_at(-self.half_size[0], self.half_size[1], self.half_size[2]),
            self.point_at(-self.half_size[0], -self.half_size[1], self.half_size[2]),
            self.point_at(self.half_size[0], -self.half_size[1], self.half_size[2]),
            self.point_at(self.half_size[0], self.half_size[1], self.half_size[2]),
        ]
    }

    pub fn inflate(&mut self, amount: f64) {
        self.half_size = Vector::new(
            self.half_size[0] + amount,
            self.half_size[1] + amount,
            self.half_size[2] + amount,
        );
    }

    pub fn area(&self) -> f64 {
        let (hx, hy, hz) = (self.half_size[0], self.half_size[1], self.half_size[2]);
        8.0 * (hx * hy + hy * hz + hz * hx)
    }

    pub fn diagonal(&self) -> f64 {
        let (hx, hy, hz) = (self.half_size[0], self.half_size[1], self.half_size[2]);
        2.0 * (hx * hx + hy * hy + hz * hz).sqrt()
    }

    pub fn is_valid(&self) -> bool {
        self.half_size[0] >= 0.0 && self.half_size[1] >= 0.0 && self.half_size[2] >= 0.0
    }

    pub fn volume(&self) -> f64 {
        8.0 * self.half_size[0] * self.half_size[1] * self.half_size[2]
    }

    pub fn closest_point(&self, pt: &Point) -> Point {
        let dx = pt[0] - self.center[0];
        let dy = pt[1] - self.center[1];
        let dz = pt[2] - self.center[2];
        let mut lx = dx * self.x_axis[0] + dy * self.x_axis[1] + dz * self.x_axis[2];
        let mut ly = dx * self.y_axis[0] + dy * self.y_axis[1] + dz * self.y_axis[2];
        let mut lz = dx * self.z_axis[0] + dy * self.z_axis[1] + dz * self.z_axis[2];
        lx = lx.max(-self.half_size[0]).min(self.half_size[0]);
        ly = ly.max(-self.half_size[1]).min(self.half_size[1]);
        lz = lz.max(-self.half_size[2]).min(self.half_size[2]);
        Point::new(
            self.center[0] + lx * self.x_axis[0] + ly * self.y_axis[0] + lz * self.z_axis[0],
            self.center[1] + lx * self.x_axis[1] + ly * self.y_axis[1] + lz * self.z_axis[1],
            self.center[2] + lx * self.x_axis[2] + ly * self.y_axis[2] + lz * self.z_axis[2],
        )
    }

    pub fn contains(&self, pt: &Point) -> bool {
        let dx = pt[0] - self.center[0];
        let dy = pt[1] - self.center[1];
        let dz = pt[2] - self.center[2];
        let lx = (dx * self.x_axis[0] + dy * self.x_axis[1] + dz * self.x_axis[2]).abs();
        let ly = (dx * self.y_axis[0] + dy * self.y_axis[1] + dz * self.y_axis[2]).abs();
        let lz = (dx * self.z_axis[0] + dy * self.z_axis[1] + dz * self.z_axis[2]).abs();
        lx <= self.half_size[0] && ly <= self.half_size[1] && lz <= self.half_size[2]
    }

    pub fn corner(&self, x_max: bool, y_max: bool, z_max: bool) -> Point {
        let ox = if x_max { self.half_size[0] } else { -self.half_size[0] };
        let oy = if y_max { self.half_size[1] } else { -self.half_size[1] };
        let oz = if z_max { self.half_size[2] } else { -self.half_size[2] };
        self.point_at(ox, oy, oz)
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

    pub fn union_with(&mut self, other: &OBB) {
        let (mut min_x, mut max_x) = (-self.half_size[0], self.half_size[0]);
        let (mut min_y, mut max_y) = (-self.half_size[1], self.half_size[1]);
        let (mut min_z, mut max_z) = (-self.half_size[2], self.half_size[2]);
        for c in &other.corners() {
            let dx = c[0] - self.center[0];
            let dy = c[1] - self.center[1];
            let dz = c[2] - self.center[2];
            let lx = dx * self.x_axis[0] + dy * self.x_axis[1] + dz * self.x_axis[2];
            let ly = dx * self.y_axis[0] + dy * self.y_axis[1] + dz * self.y_axis[2];
            let lz = dx * self.z_axis[0] + dy * self.z_axis[1] + dz * self.z_axis[2];
            min_x = min_x.min(lx); max_x = max_x.max(lx);
            min_y = min_y.min(ly); max_y = max_y.max(ly);
            min_z = min_z.min(lz); max_z = max_z.max(lz);
        }
        let ox = (min_x + max_x) * 0.5;
        let oy = (min_y + max_y) * 0.5;
        let oz = (min_z + max_z) * 0.5;
        self.center = Point::new(
            self.center[0] + ox * self.x_axis[0] + oy * self.y_axis[0] + oz * self.z_axis[0],
            self.center[1] + ox * self.x_axis[1] + oy * self.y_axis[1] + oz * self.z_axis[1],
            self.center[2] + ox * self.x_axis[2] + oy * self.y_axis[2] + oz * self.z_axis[2],
        );
        self.half_size = Vector::new(
            (max_x - min_x) * 0.5,
            (max_y - min_y) * 0.5,
            (max_z - min_z) * 0.5,
        );
    }

    fn separating_plane_exists(
        relative_position: &Vector,
        axis: &Vector,
        box1: &OBB,
        box2: &OBB,
    ) -> bool {
        let dot_rp = relative_position.dot(axis).abs();

        let v1 = box1.x_axis.clone() * box1.half_size[0];
        let v2 = box1.y_axis.clone() * box1.half_size[1];
        let v3 = box1.z_axis.clone() * box1.half_size[2];
        let proj1 = v1.dot(axis).abs() + v2.dot(axis).abs() + v3.dot(axis).abs();

        let v4 = box2.x_axis.clone() * box2.half_size[0];
        let v5 = box2.y_axis.clone() * box2.half_size[1];
        let v6 = box2.z_axis.clone() * box2.half_size[2];
        let proj2 = v4.dot(axis).abs() + v5.dot(axis).abs() + v6.dot(axis).abs();

        dot_rp > (proj1 + proj2)
    }

    pub fn collides_with_broad(&self, other: &OBB) -> bool {
        if !self.aabb().intersects(&other.aabb()) {
            return false;
        }
        self.collides_with(other)
    }

    pub fn collides_with(&self, other: &OBB) -> bool {
        let center_pt = Point::new(self.center[0], self.center[1], self.center[2]);
        let other_center_pt = Point::new(other.center[0], other.center[1], other.center[2]);
        let relative_position = Vector::from_points(&center_pt, &other_center_pt);

        !(Self::separating_plane_exists(&relative_position, &self.x_axis, self, other)
            || Self::separating_plane_exists(&relative_position, &self.y_axis, self, other)
            || Self::separating_plane_exists(&relative_position, &self.z_axis, self, other)
            || Self::separating_plane_exists(&relative_position, &other.x_axis, self, other)
            || Self::separating_plane_exists(&relative_position, &other.y_axis, self, other)
            || Self::separating_plane_exists(&relative_position, &other.z_axis, self, other)
            || Self::separating_plane_exists(
                &relative_position,
                &self.x_axis.cross(&other.x_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.x_axis.cross(&other.y_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.x_axis.cross(&other.z_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.y_axis.cross(&other.x_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.y_axis.cross(&other.y_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.y_axis.cross(&other.z_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.z_axis.cross(&other.x_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.z_axis.cross(&other.y_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.z_axis.cross(&other.z_axis),
                self,
                other,
            ))
    }

    pub fn collides_with_rtcd(&self, other: &OBB) -> bool {
        const EPS: f64 = 1e-9;
        let (a0, a1, a2) = (self.half_size[0], self.half_size[1], self.half_size[2]);
        let (b0, b1, b2) = (other.half_size[0], other.half_size[1], other.half_size[2]);
        let (r00, r01, r02) = (self.x_axis.dot(&other.x_axis), self.x_axis.dot(&other.y_axis), self.x_axis.dot(&other.z_axis));
        let (r10, r11, r12) = (self.y_axis.dot(&other.x_axis), self.y_axis.dot(&other.y_axis), self.y_axis.dot(&other.z_axis));
        let (r20, r21, r22) = (self.z_axis.dot(&other.x_axis), self.z_axis.dot(&other.y_axis), self.z_axis.dot(&other.z_axis));
        let d = Vector::new(other.center[0] - self.center[0], other.center[1] - self.center[1], other.center[2] - self.center[2]);
        let (t0, t1, t2) = (d.dot(&self.x_axis), d.dot(&self.y_axis), d.dot(&self.z_axis));
        let (ar00, ar01, ar02) = (r00.abs() + EPS, r01.abs() + EPS, r02.abs() + EPS);
        let (ar10, ar11, ar12) = (r10.abs() + EPS, r11.abs() + EPS, r12.abs() + EPS);
        let (ar20, ar21, ar22) = (r20.abs() + EPS, r21.abs() + EPS, r22.abs() + EPS);
        if t0.abs() > a0 + b0 * ar00 + b1 * ar01 + b2 * ar02 { return false; }
        if t1.abs() > a1 + b0 * ar10 + b1 * ar11 + b2 * ar12 { return false; }
        if t2.abs() > a2 + b0 * ar20 + b1 * ar21 + b2 * ar22 { return false; }
        if (t0 * r00 + t1 * r10 + t2 * r20).abs() > a0 * ar00 + a1 * ar10 + a2 * ar20 + b0 { return false; }
        if (t0 * r01 + t1 * r11 + t2 * r21).abs() > a0 * ar01 + a1 * ar11 + a2 * ar21 + b1 { return false; }
        if (t0 * r02 + t1 * r12 + t2 * r22).abs() > a0 * ar02 + a1 * ar12 + a2 * ar22 + b2 { return false; }
        if (t2 * r10 - t1 * r20).abs() > a1 * ar20 + a2 * ar10 + b1 * ar02 + b2 * ar01 { return false; }
        if (t2 * r11 - t1 * r21).abs() > a1 * ar21 + a2 * ar11 + b0 * ar02 + b2 * ar00 { return false; }
        if (t2 * r12 - t1 * r22).abs() > a1 * ar22 + a2 * ar12 + b0 * ar01 + b1 * ar00 { return false; }
        if (t0 * r20 - t2 * r00).abs() > a0 * ar20 + a2 * ar00 + b1 * ar12 + b2 * ar11 { return false; }
        if (t0 * r21 - t2 * r01).abs() > a0 * ar21 + a2 * ar01 + b0 * ar12 + b2 * ar10 { return false; }
        if (t0 * r22 - t2 * r02).abs() > a0 * ar22 + a2 * ar02 + b0 * ar11 + b1 * ar10 { return false; }
        if (t1 * r00 - t0 * r10).abs() > a0 * ar10 + a1 * ar00 + b1 * ar22 + b2 * ar21 { return false; }
        if (t1 * r01 - t0 * r11).abs() > a0 * ar11 + a1 * ar01 + b0 * ar22 + b2 * ar20 { return false; }
        if (t1 * r02 - t0 * r12).abs() > a0 * ar12 + a1 * ar02 + b0 * ar21 + b1 * ar20 { return false; }
        true
    }

    pub fn collides_with_naive(&self, other: &OBB) -> bool {
        let relative_position = Vector::new(
            other.center[0] - self.center[0],
            other.center[1] - self.center[1],
            other.center[2] - self.center[2],
        );
        let (x1, y1, z1) = (&self.x_axis, &self.y_axis, &self.z_axis);
        let (x2, y2, z2) = (&other.x_axis, &other.y_axis, &other.z_axis);
        if Self::separating_plane_exists(&relative_position, x1, self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, y1, self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, z1, self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, x2, self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, y2, self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, z2, self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, &x1.cross(x2), self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, &x1.cross(y2), self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, &x1.cross(z2), self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, &y1.cross(x2), self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, &y1.cross(y2), self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, &y1.cross(z2), self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, &z1.cross(x2), self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, &z1.cross(y2), self, other) { return false; }
        if Self::separating_plane_exists(&relative_position, &z1.cross(z2), self, other) { return false; }
        true
    }

    pub fn transform(&mut self, xform: &Xform) {
        self.center.transform(xform);
        self.x_axis.transform(xform);
        self.y_axis.transform(xform);
        self.z_axis.transform(xform);
    }

    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut result = self.clone();
        result.transform(xform);
        result
    }

    pub fn jsondump(&self) -> Result<String, std::boxed::Box<dyn std::error::Error>> {
        let data = serde_json::json!({
            "type": "OBB",
            "center": serde_json::from_str::<serde_json::Value>(&self.center.jsondump()?)?,
            "x_axis": serde_json::from_str::<serde_json::Value>(&self.x_axis.jsondump()?)?,
            "y_axis": serde_json::from_str::<serde_json::Value>(&self.y_axis.jsondump()?)?,
            "z_axis": serde_json::from_str::<serde_json::Value>(&self.z_axis.jsondump()?)?,
            "half_size": serde_json::from_str::<serde_json::Value>(&self.half_size.jsondump()?)?,
            "guid": self.guid(),
            "name": self.name,
        });
        Ok(serde_json::to_string(&data)?)
    }

    pub fn jsonload(json_data: &str) -> Result<Self, std::boxed::Box<dyn std::error::Error>> {
        let data: serde_json::Value = serde_json::from_str(json_data)?;
        let mut bbox = OBB::new(
            Point::jsonload(&data["center"].to_string())?,
            Vector::jsonload(&data["x_axis"].to_string())?,
            Vector::jsonload(&data["y_axis"].to_string())?,
            Vector::jsonload(&data["z_axis"].to_string())?,
            Vector::jsonload(&data["half_size"].to_string())?,
        );
        bbox.set_guid(data["guid"].as_str().unwrap().to_string());
        bbox.name = data["name"].as_str().unwrap().to_string();
        Ok(bbox)
    }

    pub fn to_json(&self, filepath: &str) -> Result<(), std::boxed::Box<dyn std::error::Error>> {
        let json_string = self.jsondump()?;
        let value: serde_json::Value = serde_json::from_str(&json_string)?;
        let pretty = serde_json::to_string_pretty(&value)?;
        std::fs::write(filepath, pretty)?;
        Ok(())
    }

    pub fn from_json(filepath: &str) -> Result<Self, std::boxed::Box<dyn std::error::Error>> {
        let json_string = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json_string)
    }

    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn file_json_loads(s: &str) -> Self {
        Self::jsonload(s).unwrap_or_default()
    }

    pub fn file_json_dump(&self, filepath: &str) {
        let _ = self.to_json(filepath);
    }

    pub fn file_json_load(filepath: &str) -> Self {
        Self::from_json(filepath).unwrap_or_default()
    }

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        self.to_proto().encode_to_vec()
    }

    /// The proto struct itself — pb_dumps encodes it; Session embeds it directly.
    pub fn to_proto(&self) -> crate::proto::BoundingBox {
        use prost::Message;
        crate::proto::BoundingBox {
            center: Some(crate::proto::Point::decode(self.center.pb_dumps().as_slice()).unwrap()),
            x_axis: Some(crate::proto::Vector::decode(self.x_axis.pb_dumps().as_slice()).unwrap()),
            y_axis: Some(crate::proto::Vector::decode(self.y_axis.pb_dumps().as_slice()).unwrap()),
            z_axis: Some(crate::proto::Vector::decode(self.z_axis.pb_dumps().as_slice()).unwrap()),
            half_size: Some(crate::proto::Vector::decode(self.half_size.pb_dumps().as_slice()).unwrap()),
            guid: self.guid().to_string(),
            name: self.name.clone(),
        }
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        Self::from_proto(crate::proto::BoundingBox::decode(data)?)
    }

    /// Build from an already-decoded proto — pb_loads decodes then calls this.
    pub fn from_proto(proto: crate::proto::BoundingBox) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let center = if let Some(p) = &proto.center {
            crate::point::Point::pb_loads(&p.encode_to_vec())?
        } else {
            crate::point::Point::new(0.0, 0.0, 0.0)
        };
        let x_axis = if let Some(v) = &proto.x_axis {
            crate::vector::Vector::pb_loads(&v.encode_to_vec())?
        } else {
            crate::vector::Vector::new(1.0, 0.0, 0.0)
        };
        let y_axis = if let Some(v) = &proto.y_axis {
            crate::vector::Vector::pb_loads(&v.encode_to_vec())?
        } else {
            crate::vector::Vector::new(0.0, 1.0, 0.0)
        };
        let z_axis = if let Some(v) = &proto.z_axis {
            crate::vector::Vector::pb_loads(&v.encode_to_vec())?
        } else {
            crate::vector::Vector::new(0.0, 0.0, 1.0)
        };
        let half_size = if let Some(v) = &proto.half_size {
            crate::vector::Vector::pb_loads(&v.encode_to_vec())?
        } else {
            crate::vector::Vector::new(0.5, 0.5, 0.5)
        };
        let mut bbox = OBB::new(center, x_axis, y_axis, z_axis, half_size);
        bbox.set_guid(proto.guid);
        bbox.name = proto.name;
        Ok(bbox)
    }

    pub fn pb_dump(&self, filepath: &str) {
        std::fs::write(filepath, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // WGPU
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// The 8 box corners as f32 `[x, y, z]` rows — ready for a wireframe-box vertex/segment
    /// buffer. Same winding as [`corners`](Self::corners); the kernel keeps f64.
    pub fn corners_f32(&self) -> [[f32; 3]; 8] {
        self.corners().map(|p| p.to_f32())
    }
}

impl Default for OBB {
    fn default() -> Self {
        OBB {
            center: Point::new(0.0, 0.0, 0.0),
            x_axis: Vector::new(1.0, 0.0, 0.0),
            y_axis: Vector::new(0.0, 1.0, 0.0),
            z_axis: Vector::new(0.0, 0.0, 1.0),
            half_size: Vector::new(0.5, 0.5, 0.5),
            guid: std::sync::OnceLock::new(),
            name: String::new(),
        }
    }
}

#[cfg(test)]
#[path = "obb_test.rs"]
mod obb_test;

