use crate::tolerance::Tolerance;
use crate::tolerance::PI;
use crate::Color;
use crate::Line;
use crate::Plane;
use crate::Point;
use crate::Vector;
use crate::Xform;
use serde::Deserialize;
use serde::Deserializer;
use serde::Serialize;
use std::collections::BinaryHeap;
use std::fmt;
use std::ops::Add;
use std::ops::AddAssign;
use std::ops::Div;
use std::ops::DivAssign;
use std::ops::Index;
use std::ops::IndexMut;
use std::ops::Mul;
use std::ops::MulAssign;
use std::ops::Neg;
use std::ops::Sub;
use std::ops::SubAssign;
use std::sync::OnceLock;

// ═══════════════════════════════════════════════════════════════════════════
// 2D helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Return the cross product sign of (b - a) x (p - a).
fn ccw_2d(ax: f64, ay: f64, bx: f64, by: f64, px: f64, py: f64) -> f64 {
    (bx - ax) * (py - ay) - (by - ay) * (px - ax)
}

/// Return the squared distance from (px, py) to the segment (a, b).
fn seg_dist_sq(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let mut x = ax;
    let mut y = ay;
    let mut dx = bx - x;
    let mut dy = by - y;

    if dx != 0.0 || dy != 0.0 {
        let t = ((px - x) * dx + (py - y) * dy) / (dx * dx + dy * dy);

        if t > 1.0 {
            x = bx;
            y = by;
        } else if t > 0.0 {
            x += dx * t;
            y += dy * t;
        }
    }

    dx = px - x;
    dy = py - y;

    dx * dx + dy * dy
}

/// Return the signed distance to the polygon rings, positive inside.
fn point_to_polygon_dist(px: f64, py: f64, polygon: &[Vec<[f64; 2]>]) -> f64 {
    let mut inside = false;
    let mut min_dist_sq = f64::INFINITY;

    for ring in polygon {
        let len = ring.len();

        if len == 0 {
            continue;
        }

        let mut j = len - 1;

        for i in 0..len {
            let ax = ring[i][0];
            let ay = ring[i][1];
            let bx = ring[j][0];
            let by = ring[j][1];

            if (ay > py) != (by > py) && (px < (bx - ax) * (py - ay) / (by - ay) + ax) {
                inside = !inside;
            }

            min_dist_sq = min_dist_sq.min(seg_dist_sq(px, py, ax, ay, bx, by));
            j = i;
        }
    }

    (if inside { 1.0 } else { -1.0 }) * min_dist_sq.sqrt()
}

/// Quadtree cell of the polylabel search: center, half size, distance and its upper bound.
#[derive(Clone, Copy)]
struct PCell {
    cx: f64, // Center x.
    cy: f64, // Center y.
    h: f64,  // Half size.
    d: f64,  // Signed distance from the center to the polygon.
    mx: f64, // Upper bound of the distance inside the cell.
}

impl PCell {
    /// Construct the cell at (cx, cy) with half size h against polygon.
    fn new(cx: f64, cy: f64, h: f64, polygon: &[Vec<[f64; 2]>]) -> Self {
        let d = point_to_polygon_dist(cx, cy, polygon);

        PCell {
            cx,
            cy,
            h,
            d,
            mx: d + h * std::f64::consts::SQRT_2,
        }
    }
}

impl PartialEq for PCell {
    /// Compare cells by upper bound.
    fn eq(&self, o: &Self) -> bool {
        self.mx == o.mx
    }
}

impl Eq for PCell {}

impl PartialOrd for PCell {
    /// Order cells by upper bound.
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}

impl Ord for PCell {
    /// Order by the upper bound so the priority queue pops the most promising cell.
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        self.mx
            .partial_cmp(&o.mx)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

/// Return the cell at the centroid of the outer ring.
fn centroid_cell(polygon: &[Vec<[f64; 2]>]) -> PCell {
    let mut area = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    let ring = &polygon[0];
    let len = ring.len();
    let mut j = len - 1;

    for i in 0..len {
        let ax = ring[i][0];
        let ay = ring[i][1];
        let bx = ring[j][0];
        let by = ring[j][1];
        let f = ax * by - bx * ay;
        cx += (ax + bx) * f;
        cy += (ay + by) * f;
        area += f * 3.0;
        j = i;
    }

    if area == 0.0 {
        return PCell::new(ring[0][0], ring[0][1], 0.0, polygon);
    }

    PCell::new(cx / area, cy / area, 0.0, polygon)
}

/// Return the center and radius of the largest inscribed circle in 2D (Mapbox polylabel).
fn mapbox_polylabel(polygon: &[Vec<[f64; 2]>], precision: f64) -> [f64; 3] {
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;

    for p in &polygon[0] {
        min_x = min_x.min(p[0]);
        max_x = max_x.max(p[0]);
        min_y = min_y.min(p[1]);
        max_y = max_y.max(p[1]);
    }

    let size_x = max_x - min_x;
    let size_y = max_y - min_y;
    let cell_size = size_x.min(size_y);
    let h = cell_size / 2.0;

    if cell_size == 0.0 {
        return [min_x, min_y, 0.0];
    }

    let mut queue: BinaryHeap<PCell> = BinaryHeap::new();
    let mut x = min_x;

    while x < max_x {
        let mut y = min_y;

        while y < max_y {
            queue.push(PCell::new(x + h, y + h, h, polygon));
            y += cell_size;
        }

        x += cell_size;
    }

    let mut best = centroid_cell(polygon);
    let bbox_cell = PCell::new(min_x + size_x / 2.0, min_y + size_y / 2.0, 0.0, polygon);

    if bbox_cell.d > best.d {
        best = bbox_cell;
    }

    let max_iter = 1000000;

    for _ in 0..max_iter {
        let Some(cell) = queue.pop() else { break };

        if cell.d > best.d {
            best = cell;
        }

        if cell.mx - best.d <= precision {
            continue;
        }

        let nh = cell.h / 2.0;
        queue.push(PCell::new(cell.cx - nh, cell.cy - nh, nh, polygon));
        queue.push(PCell::new(cell.cx + nh, cell.cy - nh, nh, polygon));
        queue.push(PCell::new(cell.cx - nh, cell.cy + nh, nh, polygon));
        queue.push(PCell::new(cell.cx + nh, cell.cy + nh, nh, polygon));
    }

    [best.cx, best.cy, best.d]
}

/// A polyline stored as flat coordinates [x0, y0, z0, x1, y1, z1, ...] with a lazily computed plane.
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "type", rename = "Polyline")]
pub struct Polyline {
    #[serde(serialize_with = "crate::guid_serde::serialize")]
    guid: OnceLock<String>, // Lazily minted GUID.
    #[serde(skip)]
    plane_dirty: bool, // True until get_plane recomputes.
    pub name: String,     // Polyline name.
    pub coords: Vec<f64>, // Flat [x, y, z, ...].
    #[serde(skip)]
    pub plane: Plane, // Lazily computed plane, see get_plane.
    pub width: f64,       // Display width.
    pub dash: Vec<f64>,   // Dash pattern lengths.
    pub linecolor: Color, // Display color.
}

impl Polyline {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from points.
    pub fn new(points: Vec<Point>) -> Self {
        let mut polyline = Self::default();
        polyline.coords.reserve(points.len() * 3);

        for p in &points {
            polyline.coords.push(p[0]);
            polyline.coords.push(p[1]);
            polyline.coords.push(p[2]);
        }

        polyline.recompute_plane_if_needed();

        polyline
    }

    /// Copy with a new guid and the same data.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = OnceLock::new();

        copy
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from flat [x, y, z, ...] coordinates.
    pub fn from_coords(coords: Vec<f64>) -> Self {
        let mut polyline = Self {
            coords,
            ..Default::default()
        };
        polyline.recompute_plane_if_needed();

        polyline
    }

    /// Construct a regular polygon of sides around the origin in the XY plane.
    pub fn from_sides(sides: usize, radius: f64, close: bool) -> Self {
        let mut points = Vec::with_capacity(if close { sides + 1 } else { sides });

        for i in 0..sides {
            let angle = 2.0 * PI * i as f64 / sides as f64;
            points.push(Point::new(radius * angle.cos(), radius * angle.sin(), 0.0));
        }

        if close {
            points.push(points[0].clone());
        }

        Self::new(points)
    }

    /// Construct a rectangle with its corner at origin, sides along x_axis and y_axis.
    pub fn rectangle(
        origin: &Point,
        x_axis: &Vector,
        y_axis: &Vector,
        width: f64,
        height: f64,
        close: bool,
    ) -> Self {
        let plane = Plane::new(origin.clone(), x_axis.clone(), y_axis.clone());
        let o = plane.origin();
        let x = plane.x_axis() * width;
        let y = plane.y_axis() * height;
        let mut points = vec![o.clone(), &o + &x, &(&o + &x) + &y, &o + &y];

        if close {
            points.push(points[0].clone());
        }

        Self::new(points)
    }

    /// Construct the quadratic Bezier through p0, p1, p2 sampled at divisions points.
    pub fn quadratic_points(p0: &Point, p1: &Point, p2: &Point, divisions: usize) -> Self {
        let n = divisions.max(2);
        let d = (n - 1) as f64;
        let mut points = Vec::with_capacity(n);

        for k in 0..n {
            let t = k as f64 / d;
            let s = 1.0 - t;
            let s2 = s * s;
            let ts = 2.0 * s * t;
            let t2 = t * t;
            points.push(Point::new(
                s2 * p0[0] + ts * p1[0] + t2 * p2[0],
                s2 * p0[1] + ts * p1[1] + t2 * p2[1],
                s2 * p0[2] + ts * p1[2] + t2 * p2[2],
            ));
        }

        Self::new(points)
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

    /// Set the guid if it has not already been created.
    pub fn set_guid(&self, guid: String) {
        let _ = self.guid.set(guid);
    }

    /// Clear the guid so a fresh one mints lazily on the next read.
    pub fn refresh_guid(&mut self) {
        self.guid = OnceLock::new();
    }

    /// Return the number of points.
    pub fn point_count(&self) -> usize {
        self.coords.len() / 3
    }

    /// Return the number of points.
    pub fn len(&self) -> usize {
        self.point_count()
    }

    /// Return whether the polyline has no points.
    pub fn is_empty(&self) -> bool {
        self.coords.is_empty()
    }

    /// Return the number of segments.
    pub fn segment_count(&self) -> usize {
        self.point_count().saturating_sub(1)
    }

    /// Return the point at index, or None when out of range.
    pub fn get_point(&self, index: usize) -> Option<Point> {
        if index >= self.point_count() {
            return None;
        }

        let idx = index * 3;

        Some(Point::new(
            self.coords[idx],
            self.coords[idx + 1],
            self.coords[idx + 2],
        ))
    }

    /// Return all points.
    pub fn get_points(&self) -> Vec<Point> {
        let mut points = Vec::with_capacity(self.point_count());

        for i in 0..self.point_count() {
            points.push(Point::new(
                self.coords[i * 3],
                self.coords[i * 3 + 1],
                self.coords[i * 3 + 2],
            ));
        }

        points
    }

    /// Return one line per segment.
    pub fn get_lines(&self) -> Vec<Line> {
        let mut lines = Vec::with_capacity(self.segment_count());

        for i in 0..self.segment_count() {
            let idx0 = i * 3;
            let idx1 = (i + 1) * 3;
            lines.push(Line::new(
                self.coords[idx0],
                self.coords[idx0 + 1],
                self.coords[idx0 + 2],
                self.coords[idx1],
                self.coords[idx1 + 1],
                self.coords[idx1 + 2],
            ));
        }

        lines
    }

    /// Return the plane from the first non-collinear triple, computed on first access.
    pub fn get_plane(&mut self) -> &Plane {
        if !self.plane_dirty || self.point_count() < 3 {
            return &self.plane;
        }

        let n = self.point_count();
        let p0 = self.point(0);
        let mut found = false;

        for i in 1..n {
            if found {
                break;
            }

            let mut v1 = &self.point(i) - &p0;

            if v1.magnitude_squared() < 1e-20 {
                continue;
            }

            for j in (i + 1)..n {
                if found {
                    break;
                }

                let mut normal = v1.cross(&(&self.point(j) - &p0));

                if normal.magnitude_squared() < 1e-20 {
                    continue;
                }

                normal.normalize_self();
                v1.normalize_self();
                let mut yax = normal.cross(&v1);
                yax.normalize_self();
                self.plane = Plane::from_frame(p0.clone(), v1.clone(), yax, normal);
                found = true;
            }
        }

        if !found {
            self.plane = Plane::default();
        }

        self.plane_dirty = false;

        &self.plane
    }

    /// Return the total length.
    pub fn length(&self) -> f64 {
        let mut total = 0.0;

        for i in 0..self.segment_count() {
            total += (&self.point(i + 1) - &self.point(i))
                .magnitude_squared()
                .sqrt();
        }

        total
    }

    /// Return the sum of squared segment lengths.
    pub fn length_squared(&self) -> f64 {
        let mut total = 0.0;

        for i in 0..self.segment_count() {
            total += (&self.point(i + 1) - &self.point(i)).magnitude_squared();
        }

        total
    }

    /// Return whether the first and last points coincide.
    pub fn is_closed(&self) -> bool {
        if self.point_count() < 2 {
            return false;
        }

        self.point(0)
            .distance(&self.point(self.point_count() - 1), None)
            < Tolerance::ZERO_TOLERANCE
    }

    /// Return a copy with the first point appended when open.
    pub fn closed(&self) -> Self {
        if self.is_closed() {
            return Self::from_coords(self.coords.clone());
        }

        let mut coords = self.coords.clone();
        coords.push(self.coords[0]);
        coords.push(self.coords[1]);
        coords.push(self.coords[2]);

        Self::from_coords(coords)
    }

    /// Return the average of the points, closing duplicate excluded.
    pub fn center(&self) -> Point {
        if self.coords.is_empty() {
            return Point::new(0.0, 0.0, 0.0);
        }

        let n = if self.is_closed() {
            self.point_count() - 1
        } else {
            self.point_count()
        };
        let mut x = 0.0;
        let mut y = 0.0;
        let mut z = 0.0;

        for i in 0..n {
            x += self.coords[i * 3];
            y += self.coords[i * 3 + 1];
            z += self.coords[i * 3 + 2];
        }

        Point::new(x / n as f64, y / n as f64, z / n as f64)
    }

    /// Compute the frame with origin at center, x along the first segment, z the average normal.
    pub fn get_average_plane(&self) -> (Point, Vector, Vector, Vector) {
        let origin = self.center();
        let mut x_axis = if self.point_count() >= 2 {
            &self.point(1) - &self.point(0)
        } else {
            Vector::new(1.0, 0.0, 0.0)
        };
        x_axis.normalize_self();
        let z_axis = self.average_normal();
        let mut y_axis = z_axis.cross(&x_axis);
        y_axis.normalize_self();

        (origin, x_axis, y_axis, z_axis)
    }

    /// Compute the plane with origin at the first point, normal the average normal.
    pub fn get_fast_plane(&self) -> (Point, Plane) {
        if self.coords.is_empty() {
            return (Point::new(0.0, 0.0, 0.0), Plane::default());
        }

        let origin = self.point(0);
        let normal = self.average_normal();
        let pln = Plane::from_point_normal(origin.clone(), normal, None);

        (origin, pln)
    }

    /// Compute one flag per corner, true when convex against the average normal.
    pub fn get_convex_corners(&self) -> Vec<bool> {
        if self.point_count() < 3 {
            return Vec::new();
        }

        let n = if self.is_closed() {
            self.point_count() - 1
        } else {
            self.point_count()
        };
        let normal = self.average_normal();
        let mut convex_or_concave = Vec::with_capacity(n);

        for current in 0..n {
            let prev = if current == 0 { n - 1 } else { current - 1 };
            let next = if current == n - 1 { 0 } else { current + 1 };
            let mut dir0 = &self.point(current) - &self.point(prev);
            dir0.normalize_self();
            let mut dir1 = &self.point(next) - &self.point(current);
            dir1.normalize_self();
            let mut cross = dir0.cross(&dir1);
            cross.normalize_self();
            convex_or_concave.push(cross.dot(&normal) >= 0.0);
        }

        convex_or_concave
    }

    /// Return the shoelace sign of the points projected onto pln.
    pub fn is_clockwise(&self, pln: &Plane) -> bool {
        let n = self.point_count();

        if n < 3 {
            return false;
        }

        let xv = pln.x_axis();
        let yv = pln.y_axis();
        let orig = pln.origin();
        let lim = if self.is_closed() { n - 1 } else { n };
        let mut area = 0.0;

        for i in 0..lim {
            let d0 = &self.point(i) - &orig;
            let d1 = &self.point((i + 1) % lim) - &orig;
            let u0 = d0.dot(&xv);
            let v0 = d0.dot(&yv);
            let u1 = d1.dot(&xv);
            let v1 = d1.dot(&yv);
            area += (u1 - u0) * (v1 + v0);
        }

        area > 0.0
    }

    /// Return the winding-number test on x and y.
    pub fn point_in_polygon_2d(&self, p: &Point) -> bool {
        let px = p[0];
        let py = p[1];
        let n = self.point_count();
        let mut winding: i32 = 0;

        for i in 0..n {
            let j = (i + 1) % n;
            let x0 = self.coords[i * 3];
            let y0 = self.coords[i * 3 + 1];
            let x1 = self.coords[j * 3];
            let y1 = self.coords[j * 3 + 1];
            let side = (x1 - x0) * (py - y0) - (px - x0) * (y1 - y0);

            if y0 <= py && y1 > py && side > 0.0 {
                winding += 1;
            } else if y0 > py && y1 <= py && side < 0.0 {
                winding -= 1;
            }
        }

        winding != 0
    }

    /// Return the distance to the nearest segment, with its index and the closest point.
    pub fn closest_distance_and_point(&self, point: &Point) -> (f64, usize, Point) {
        let mut edge_id = 0;
        let mut closest_distance = f64::MAX;
        let mut best_t = 0.0;

        for i in 0..self.segment_count() {
            let t = Self::closest_point_to_line(point, &self.point(i), &self.point(i + 1));
            let distance =
                point.distance(&Self::point_at(&self.point(i), &self.point(i + 1), t), None);

            if distance < closest_distance {
                closest_distance = distance;
                edge_id = i;
                best_t = t;
            }

            if closest_distance < Tolerance::ZERO_TOLERANCE {
                break;
            }
        }

        let closest_point = Self::point_at(&self.point(edge_id), &self.point(edge_id + 1), best_t);

        (closest_distance, edge_id, closest_point)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Set the point at index.
    pub fn set_point(&mut self, index: usize, point: &Point) {
        if index >= self.point_count() {
            return;
        }

        let idx = index * 3;
        self.coords[idx] = point[0];
        self.coords[idx + 1] = point[1];
        self.coords[idx + 2] = point[2];
    }

    /// Append a point.
    pub fn add_point(&mut self, point: Point) {
        self.coords.push(point[0]);
        self.coords.push(point[1]);
        self.coords.push(point[2]);

        if self.point_count() == 3 {
            self.recompute_plane_if_needed();
        }
    }

    /// Insert a point at index.
    pub fn insert_point(&mut self, index: usize, point: Point) {
        if index > self.point_count() {
            return;
        }

        let idx = index * 3;
        self.coords.splice(idx..idx, [point[0], point[1], point[2]]);

        if self.point_count() == 3 {
            self.recompute_plane_if_needed();
        }
    }

    /// Remove and return the point at index; None when out of range.
    pub fn remove_point(&mut self, index: usize) -> Option<Point> {
        if index >= self.point_count() {
            return None;
        }

        let idx = index * 3;
        let out_point = Point::new(self.coords[idx], self.coords[idx + 1], self.coords[idx + 2]);
        self.coords.drain(idx..idx + 3);

        if self.point_count() == 3 {
            self.recompute_plane_if_needed();
        }

        Some(out_point)
    }

    /// Reverse the point order in place.
    pub fn reverse(&mut self) {
        let n = self.point_count();
        let mut coords = Vec::with_capacity(self.coords.len());

        for i in (0..n).rev() {
            let idx = i * 3;
            coords.push(self.coords[idx]);
            coords.push(self.coords[idx + 1]);
            coords.push(self.coords[idx + 2]);
        }

        self.coords = coords;
        self.plane.reverse();
    }

    /// Return a reversed copy.
    pub fn reversed(&self) -> Self {
        let mut result = self.duplicate();
        result.reverse();

        result
    }

    /// Rotate the points by times positions, keeping the closing duplicate.
    pub fn shift(&mut self, times: i32) {
        if self.coords.is_empty() {
            return;
        }

        let was_closed = self.is_closed();

        if was_closed {
            self.coords.truncate(self.coords.len() - 3);
        }

        let n = self.point_count();

        if n > 0 && times != 0 {
            let mut offset = times % n as i32;

            if offset < 0 {
                offset += n as i32;
            }

            let mut coords = Vec::with_capacity(self.coords.len());

            for i in 0..n {
                let src = ((i + offset as usize) % n) * 3;
                coords.push(self.coords[src]);
                coords.push(self.coords[src + 1]);
                coords.push(self.coords[src + 2]);
            }

            self.coords = coords;
        }

        if was_closed && n > 0 {
            self.coords.push(self.coords[0]);
            self.coords.push(self.coords[1]);
            self.coords.push(self.coords[2]);
        }
    }

    /// Translate in place.
    pub fn translate(&mut self, v: &Vector) {
        *self += v;
    }

    /// Return a translated copy.
    pub fn translated(&self, v: &Vector) -> Self {
        let mut result = self.duplicate();
        result.translate(v);

        result
    }

    /// Move the segment ends by dist0 and dist1, or by proportions of its length when non-zero.
    pub fn extend_segment(
        &mut self,
        segment_id: usize,
        dist0: f64,
        dist1: f64,
        proportion0: f64,
        proportion1: f64,
    ) {
        if segment_id >= self.segment_count() {
            return;
        }

        if dist0 == 0.0 && dist1 == 0.0 && proportion0 == 0.0 && proportion1 == 0.0 {
            return;
        }

        let was_closed = self.is_closed();
        let mut p0 = self.point(segment_id);
        let mut p1 = self.point(segment_id + 1);
        let mut v = &p1 - &p0;

        if proportion0 != 0.0 || proportion1 != 0.0 {
            p0 = &p0 - &(&v * proportion0);
            p1 = &p1 + &(&v * proportion1);
        } else {
            v.normalize_self();
            p0 = &p0 - &(&v * dist0);
            p1 = &p1 + &(&v * dist1);
        }

        self.set_point(segment_id, &p0);
        self.set_point(segment_id + 1, &p1);

        if !was_closed {
            return;
        }

        if segment_id == 0 {
            self.set_point(self.point_count() - 1, &self.point(0));
        } else if segment_id + 1 == self.point_count() - 1 {
            self.set_point(0, &self.point(self.point_count() - 1));
        }
    }

    /// Move both segment ends by dist, or by proportion of its length when non-zero.
    pub fn extend_segment_equally(&mut self, segment_id: usize, dist: f64, proportion: f64) {
        if segment_id >= self.segment_count() {
            return;
        }

        let mut p0 = self.point(segment_id);
        let mut p1 = self.point(segment_id + 1);
        Self::extend_segment_equally_static(&mut p0, &mut p1, dist, proportion);
        self.set_point(segment_id, &p0);
        self.set_point(segment_id + 1, &p1);

        if self.point_count() <= 2 || !self.is_closed() {
            return;
        }

        if segment_id == 0 {
            self.set_point(self.point_count() - 1, &self.point(0));
        } else if segment_id + 1 == self.point_count() - 1 {
            self.set_point(0, &self.point(self.point_count() - 1));
        }
    }

    /// Slide both ends of edge edge_idx outward by distance, keeping the closing duplicate in sync.
    pub fn extend_edge_equally(&mut self, edge_idx: usize, distance: f64) {
        let n = self.point_count();

        if n < 2 || edge_idx + 1 >= n {
            return;
        }

        let i = edge_idx;
        let j = edge_idx + 1;
        let pi = self.point(i);
        let pj = self.point(j);
        let mut dir = &pj - &pi;
        let len = dir.magnitude_squared().sqrt();

        if len < 1e-12 {
            return;
        }

        dir = &dir * (distance / len);
        let new_pi = &pi - &dir;
        let new_pj = &pj + &dir;
        self.set_point(i, &new_pi);
        self.set_point(j, &new_pj);

        if i == 0 {
            self.set_point(n - 1, &new_pi);
        }

        if j == n - 1 {
            self.set_point(0, &new_pj);
        }
    }

    /// Drop points whose neighbours are collinear within tol; closed polylines wrap around.
    pub fn merge_collinear(&mut self, tol: f64) {
        let closed = self.is_closed();
        let mut points = self.get_points();

        if closed && points.len() > 1 {
            points.pop();
        }

        let zt2 = Tolerance::ZERO_TOLERANCE * Tolerance::ZERO_TOLERANCE;
        let max_pass = points.len();
        let mut changed = true;

        for _ in 0..max_pass {
            if !changed || points.len() < 3 {
                break;
            }

            changed = false;
            let m = points.len();
            let mut out = Vec::new();

            for i in 0..m {
                let p = (i + m - 1) % m;
                let nx = (i + 1) % m;

                if !closed && (i == 0 || i == m - 1) {
                    out.push(points[i].clone());
                    continue;
                }

                let a = &points[i] - &points[p];
                let b = &points[nx] - &points[i];
                let a2 = a.magnitude_squared();
                let b2 = b.magnitude_squared();

                if a2 < zt2 || b2 < zt2 || a.cross(&b).magnitude_squared() < tol * tol * a2 * b2 {
                    changed = true;
                } else {
                    out.push(points[i].clone());
                }
            }

            points = out;
        }

        if closed && !points.is_empty() {
            points.push(points[0].clone());
        }

        self.coords = Polyline::new(points).coords;
        self.recompute_plane_if_needed();
    }

    /// Drop consecutive points closer than tol.
    pub fn remove_consecutive_duplicates(&mut self, tol: f64) {
        let tol_sq = tol * tol;
        let mut cleaned: Vec<Point> = Vec::with_capacity(self.point_count());

        for p in &self.get_points() {
            if cleaned.is_empty() || (p - &cleaned[cleaned.len() - 1]).magnitude_squared() >= tol_sq
            {
                cleaned.push(p.clone());
            }
        }

        self.coords = Polyline::new(cleaned).coords;
        self.recompute_plane_if_needed();
    }

    /// Return a Ramer-Douglas-Peucker copy.
    pub fn simplify(&self, tolerance: f64) -> Polyline {
        Polyline::new(Self::simplify_points(&self.get_points(), tolerance))
    }

    /// Return the part on one side of plane; flip picks the normal side, unset keeps the arc-length midpoint side.
    pub fn cut_by_plane(&self, plane: &Plane, flip: Option<bool>) -> Self {
        let n = self.point_count();

        if n < 2 {
            return self.duplicate();
        }

        let normal = plane.z_axis();
        let origin = plane.origin();
        let keep_sign = match flip {
            Some(true) => 1.0,
            Some(false) => -1.0,
            None if normal.dot(&(&self.point_at_length(self.length() * 0.5) - &origin)) >= 0.0 => {
                1.0
            }
            None => -1.0,
        };
        let mut result = Vec::new();

        for i in 0..n - 1 {
            let a = self.point(i);
            let b = self.point(i + 1);
            let da = normal.dot(&(&a - &origin));
            let db = normal.dot(&(&b - &origin));

            if da * keep_sign >= 0.0 {
                result.push(a.clone());
            }

            if (da > 0.0) != (db > 0.0) {
                result.push(&a + &(&(&b - &a) * (da / (da - db))));
            }
        }

        let last = self.point(n - 1);

        if normal.dot(&(&last - &origin)) * keep_sign >= 0.0 {
            result.push(last);
        }

        let mut cut = Polyline::new(result);
        cut.remove_consecutive_duplicates(1e-6);

        cut
    }
}

impl Default for Polyline {
    /// Construct an empty polyline.
    fn default() -> Self {
        Self {
            guid: OnceLock::new(),
            plane_dirty: true,
            name: "my_polyline".to_string(),
            coords: Vec::new(),
            plane: Plane::default(),
            width: 1.0,
            dash: Vec::new(),
            linecolor: Color::black(),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
impl PartialEq for Polyline {
    /// Compare name, coordinates to 1e-6, width and linecolor; guid ignored.
    fn eq(&self, other: &Self) -> bool {
        if self.name != other.name {
            return false;
        }

        if self.point_count() != other.point_count() {
            return false;
        }

        let r = 10f64.powi(Tolerance::ROUNDING);

        for i in 0..self.coords.len() {
            if (self.coords[i] * r).round() != (other.coords[i] * r).round() {
                return false;
            }
        }

        if (self.width * r).round() != (other.width * r).round() {
            return false;
        }

        self.linecolor == other.linecolor
    }
}

impl Index<usize> for Polyline {
    type Output = [f64];

    /// Return the [x, y, z] slice of the point at index.
    fn index(&self, index: usize) -> &Self::Output {
        if index >= self.point_count() {
            panic!("Index out of range");
        }

        let idx = index * 3;

        &self.coords[idx..idx + 3]
    }
}

impl IndexMut<usize> for Polyline {
    /// Return the mutable [x, y, z] slice of the point at index.
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        if index >= self.point_count() {
            panic!("Index out of range");
        }

        let idx = index * 3;

        &mut self.coords[idx..idx + 3]
    }
}

impl AddAssign<&Vector> for Polyline {
    /// Translate in place.
    fn add_assign(&mut self, v: &Vector) {
        for i in 0..self.point_count() {
            self.coords[i * 3] += v[0];
            self.coords[i * 3 + 1] += v[1];
            self.coords[i * 3 + 2] += v[2];
        }

        self.plane = Plane::new(
            &self.plane.origin() + v,
            self.plane.x_axis(),
            self.plane.y_axis(),
        );
    }
}

impl SubAssign<&Vector> for Polyline {
    /// Translate back in place.
    fn sub_assign(&mut self, v: &Vector) {
        for i in 0..self.point_count() {
            self.coords[i * 3] -= v[0];
            self.coords[i * 3 + 1] -= v[1];
            self.coords[i * 3 + 2] -= v[2];
        }

        self.plane = Plane::new(
            &self.plane.origin() - v,
            self.plane.x_axis(),
            self.plane.y_axis(),
        );
    }
}

impl MulAssign<f64> for Polyline {
    /// Scale every point in place.
    fn mul_assign(&mut self, factor: f64) {
        for i in 0..self.coords.len() {
            self.coords[i] *= factor;
        }
    }
}

impl DivAssign<f64> for Polyline {
    /// Divide every point in place.
    fn div_assign(&mut self, factor: f64) {
        for i in 0..self.coords.len() {
            self.coords[i] /= factor;
        }
    }
}

impl Add<&Vector> for Polyline {
    type Output = Polyline;

    /// Return a translated copy.
    fn add(self, v: &Vector) -> Polyline {
        let mut result = self;
        result += v;

        result
    }
}

impl Sub<&Vector> for Polyline {
    type Output = Polyline;

    /// Return a copy translated back.
    fn sub(self, v: &Vector) -> Polyline {
        let mut result = self;
        result -= v;

        result
    }
}

impl Mul<f64> for Polyline {
    type Output = Polyline;

    /// Return a scaled copy.
    fn mul(self, factor: f64) -> Polyline {
        let mut result = self;
        result *= factor;

        result
    }
}

impl Div<f64> for Polyline {
    type Output = Polyline;

    /// Return a divided copy.
    fn div(self, factor: f64) -> Polyline {
        let mut result = self;
        result /= factor;

        result
    }
}

impl Neg for Polyline {
    type Output = Polyline;

    /// Return a reversed copy.
    fn neg(self) -> Polyline {
        self.reversed()
    }
}

impl Add<&Vector> for &Polyline {
    type Output = Polyline;

    /// Return a translated copy.
    fn add(self, v: &Vector) -> Polyline {
        let mut result = self.duplicate();
        result += v;

        result
    }
}

impl Sub<&Vector> for &Polyline {
    type Output = Polyline;

    /// Return a copy translated back.
    fn sub(self, v: &Vector) -> Polyline {
        let mut result = self.duplicate();
        result -= v;

        result
    }
}

impl Mul<f64> for &Polyline {
    type Output = Polyline;

    /// Return a scaled copy.
    fn mul(self, factor: f64) -> Polyline {
        let mut result = self.duplicate();
        result *= factor;

        result
    }
}

impl Div<f64> for &Polyline {
    type Output = Polyline;

    /// Return a divided copy.
    fn div(self, factor: f64) -> Polyline {
        let mut result = self.duplicate();
        result /= factor;

        result
    }
}

impl Neg for &Polyline {
    type Output = Polyline;

    /// Return a reversed copy.
    fn neg(self) -> Polyline {
        self.reversed()
    }
}

impl Polyline {
    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform in place.
    pub fn transform(&mut self, xform: &Xform) {
        for i in 0..self.point_count() {
            let mut point = self.point(i);
            point.transform(xform);
            self.set_point(i, &point);
        }
    }

    /// Return a transformed copy.
    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut result = self.duplicate();
        result.transform(xform);

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Segment utilities
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the point at parameter t (0 = start, 1 = end).
    pub fn point_at(start: &Point, end: &Point, t: f64) -> Point {
        let s = 1.0 - t;

        Point::new(
            if start[0] == end[0] {
                start[0]
            } else {
                s * start[0] + t * end[0]
            },
            if start[1] == end[1] {
                start[1]
            } else {
                s * start[1] + t * end[1]
            },
            if start[2] == end[2] {
                start[2]
            } else {
                s * start[2] + t * end[2]
            },
        )
    }

    /// Compute the parameter t of the closest point on the line through line_start and line_end.
    pub fn closest_point_to_line(point: &Point, line_start: &Point, line_end: &Point) -> f64 {
        let d = line_end - line_start;
        let dod = d.magnitude_squared();

        if dod <= 0.0 {
            return 0.0;
        }

        let to_start = point - line_start;
        let to_end = point - line_end;

        if to_start.magnitude_squared() <= to_end.magnitude_squared() {
            return to_start.dot(&d) / dod;
        }

        1.0 + to_end.dot(&d) / dod
    }

    /// Compute the collinear overlap of two segments; None when none or a single point.
    pub fn line_line_overlap(
        line0_start: &Point,
        line0_end: &Point,
        line1_start: &Point,
        line1_end: &Point,
    ) -> Option<(Point, Point)> {
        let (do_overlap, overlap_start, overlap_end) =
            Self::line_line_overlap_points(line0_start, line0_end, line1_start, line1_end);

        if !do_overlap {
            return None;
        }

        Some((overlap_start, overlap_end))
    }

    /// Compute the midpoints of the paired starts and ends.
    pub fn line_line_average(
        line0_start: &Point,
        line0_end: &Point,
        line1_start: &Point,
        line1_end: &Point,
    ) -> (Point, Point) {
        let output_start = Point::mid_point(line0_start, line1_start);
        let output_end = Point::mid_point(line0_end, line1_end);

        (output_start, output_end)
    }

    /// Compute the longer of the two midpoint pairings of the mutual overlaps.
    pub fn line_line_overlap_average(
        line0_start: &Point,
        line0_end: &Point,
        line1_start: &Point,
        line1_end: &Point,
    ) -> (Point, Point) {
        let (_, line_a_start, line_a_end) =
            Self::line_line_overlap_points(line0_start, line0_end, line1_start, line1_end);
        let (_, line_b_start, line_b_end) =
            Self::line_line_overlap_points(line1_start, line1_end, line0_start, line0_end);
        let (mid_line0_start, mid_line0_end) =
            Self::line_line_average(&line_a_start, &line_a_end, &line_b_start, &line_b_end);
        let (mid_line1_start, mid_line1_end) =
            Self::line_line_average(&line_a_start, &line_a_end, &line_b_end, &line_b_start);

        if (&mid_line0_end - &mid_line0_start).magnitude_squared()
            > (&mid_line1_end - &mid_line1_start).magnitude_squared()
        {
            return (mid_line0_start, mid_line0_end);
        }

        (mid_line1_start, mid_line1_end)
    }

    /// Compute the extreme sub-segment of the line spanned by the projected points; None when a single point.
    pub fn line_from_projected_points(
        line_start: &Point,
        line_end: &Point,
        points: &[Point],
    ) -> Option<(Point, Point)> {
        if points.is_empty() {
            return None;
        }

        let mut t_values = Vec::with_capacity(points.len());

        for point in points {
            t_values.push(Self::closest_point_to_line(point, line_start, line_end));
        }

        t_values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let output_start = Self::point_at(line_start, line_end, t_values[0]);
        let output_end = Self::point_at(line_start, line_end, t_values[t_values.len() - 1]);

        if (t_values[0] - t_values[t_values.len() - 1]).abs() <= Tolerance::ZERO_TOLERANCE {
            return None;
        }

        Some((output_start, output_end))
    }

    /// Move both ends by dist, or by proportion of the length when non-zero.
    pub fn extend_segment_equally_static(
        segment_start: &mut Point,
        segment_end: &mut Point,
        dist: f64,
        proportion: f64,
    ) {
        if dist == 0.0 && proportion == 0.0 {
            return;
        }

        let mut v = &*segment_end - &*segment_start;

        if proportion != 0.0 {
            *segment_start = &*segment_start - &(&v * proportion);
            *segment_end = &*segment_end + &(&v * proportion);
        } else {
            v.normalize_self();
            *segment_start = &*segment_start - &(&v * dist);
            *segment_end = &*segment_end + &(&v * dist);
        }
    }

    /// Move start by d0 and end by d1 along the unit direction.
    pub fn extend_line_segment(start: &mut Point, end: &mut Point, d0: f64, d1: f64) {
        let mut v = &*end - &*start;
        v.normalize_self();
        *start = &*start - &(&v * d0);
        *end = &*end + &(&v * d1);
    }

    /// Move both ends inward by dist as a fraction of the length.
    pub fn shrink_line_segment(start: &mut Point, end: &mut Point, dist: f64) {
        let v = &*end - &*start;
        *start = &*start + &(&v * dist);
        *end = &*end - &(&v * dist);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Polygon utilities
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the pointwise blend; polyline0 when the counts differ.
    pub fn tween_two_polylines(
        polyline0: &Polyline,
        polyline1: &Polyline,
        weight: f64,
    ) -> Polyline {
        if polyline0.point_count() != polyline1.point_count() {
            return polyline0.duplicate();
        }

        let mut result = Polyline::default();
        result.coords.reserve(polyline0.coords.len());

        for i in 0..polyline0.point_count() {
            let p0 = polyline0.point(i);
            let p1 = polyline1.point(i);
            result.add_point(&p0 + &(&(&p1 - &p0) * weight));
        }

        result
    }

    /// Return steps points between from and to; kind 0 none, 1 both, 2 start endpoint.
    pub fn interpolate_points(from: &Point, to: &Point, steps: usize, kind: usize) -> Vec<Point> {
        Point::interpolate(from, to, steps, kind)
    }

    /// Return the convex hull in the polygon's average plane.
    pub fn quick_hull(polygon: &Polyline) -> Polyline {
        let (origin, xa, ya, _) = polygon.get_average_plane();

        let pts2d = polygon.project_to_plane(&origin, &xa, &ya);
        let mut ai = 0;
        let mut bi = 0;

        for i in 1..pts2d.len() {
            if pts2d[i][0] < pts2d[ai][0] {
                ai = i;
            }

            if pts2d[i][0] >= pts2d[bi][0] {
                bi = i;
            }
        }

        let ax = pts2d[ai][0];
        let ay = pts2d[ai][1];
        let bx = pts2d[bi][0];
        let by = pts2d[bi][1];
        let mut left = Vec::new();
        let mut right = Vec::new();

        for p in &pts2d {
            if ccw_2d(ax, ay, bx, by, p[0], p[1]) > 0.0 {
                left.push(*p);
            } else {
                right.push(*p);
            }
        }

        let mut hull = vec![[ax, ay]];
        Self::quick_hull_recurse(&left, ax, ay, bx, by, &mut hull);
        hull.push([bx, by]);
        Self::quick_hull_recurse(&right, bx, by, ax, ay, &mut hull);

        let mut pts3d = Vec::with_capacity(hull.len());

        for h in &hull {
            pts3d.push(Self::unproject(&origin, &xa, &ya, h[0], h[1]));
        }

        Polyline::new(pts3d)
    }

    /// Return the minimum-area rectangle of the hull as a closed 5-point polyline.
    pub fn bounding_rectangle(polygon: &Polyline) -> Option<Polyline> {
        let hull = Self::quick_hull(polygon);

        if hull.point_count() <= 2 {
            return None;
        }

        let (origin, xa, ya, _) = polygon.get_average_plane();

        let hull2d = hull.project_to_plane(&origin, &xa, &ya);
        let mut best_area = f64::MAX;
        let mut best_min_u = 0.0;
        let mut best_max_u = 0.0;
        let mut best_min_v = 0.0;
        let mut best_max_v = 0.0;
        let mut best_angle = 0.0;
        let hn = hull2d.len();

        for i in 0..hn {
            let j = (i + 1) % hn;
            let ex = hull2d[j][0] - hull2d[i][0];
            let ey = hull2d[j][1] - hull2d[i][1];
            let len = (ex * ex + ey * ey).sqrt();

            if len < 1e-12 {
                continue;
            }

            let ca = ex / len;
            let sa = ey / len;
            let mut min_u = f64::MAX;
            let mut max_u = -f64::MAX;
            let mut min_v = f64::MAX;
            let mut max_v = -f64::MAX;

            for h in &hull2d {
                let u = h[0] * ca + h[1] * sa;
                let v = -h[0] * sa + h[1] * ca;
                min_u = min_u.min(u);
                max_u = max_u.max(u);
                min_v = min_v.min(v);
                max_v = max_v.max(v);
            }

            let area = (max_u - min_u) * (max_v - min_v);

            if area < best_area {
                best_area = area;
                best_min_u = min_u;
                best_max_u = max_u;
                best_min_v = min_v;
                best_max_v = max_v;
                best_angle = ey.atan2(ex);
            }
        }

        let ca = best_angle.cos();
        let sa = best_angle.sin();
        let uv = [
            [best_min_u, best_min_v],
            [best_min_u, best_max_v],
            [best_max_u, best_max_v],
            [best_max_u, best_min_v],
        ];
        let mut pts3d = Vec::with_capacity(5);

        for c in &uv {
            pts3d.push(Self::unproject(
                &origin,
                &xa,
                &ya,
                c[0] * ca - c[1] * sa,
                c[0] * sa + c[1] * ca,
            ));
        }

        pts3d.push(pts3d[0].clone());

        Some(Polyline::new(pts3d))
    }

    /// Return a grid of interior points spaced div_dist, on the polygon miter-offset by offset_dist.
    pub fn grid_of_points_in_polygon(
        polygon: &Polyline,
        offset_dist: f64,
        div_dist: f64,
        max_pts: usize,
    ) -> Vec<Point> {
        if div_dist < 1e-12 {
            return Vec::new();
        }

        let (origin, xa, ya, _) = polygon.get_average_plane();

        let mut poly2d = polygon.project_to_plane(&origin, &xa, &ya);

        if poly2d.len() > 1
            && polygon
                .point(0)
                .distance(&polygon.point(polygon.point_count() - 1), None)
                < 1e-10
        {
            poly2d.pop();
        }

        if poly2d.is_empty() {
            return Vec::new();
        }

        Self::offset_polygon_2d(&mut poly2d, offset_dist);
        let mut x_min = f64::MAX;
        let mut x_max = -f64::MAX;
        let mut y_min = f64::MAX;
        let mut y_max = -f64::MAX;

        for p in &poly2d {
            x_min = x_min.min(p[0]);
            x_max = x_max.max(p[0]);
            y_min = y_min.min(p[1]);
            y_max = y_max.max(p[1]);
        }

        let mut result = Vec::new();
        let mut u = x_min;

        while u <= x_max + 1e-10 && result.len() < max_pts {
            let mut v = y_min;

            while v <= y_max + 1e-10 && result.len() < max_pts {
                if Self::point_in_polygon(&poly2d, u, v) {
                    result.push(Self::unproject(&origin, &xa, &ya, u, v));
                }

                v += div_dist;
            }

            u += div_dist;
        }

        result
    }

    /// Return the largest inscribed circle of polylines[0] minus the holes polylines[1..]: center, plane, radius.
    pub fn polylabel(polylines: &[Polyline], precision: f64) -> (Point, Plane, f64) {
        if polylines.is_empty() {
            return (Point::new(0.0, 0.0, 0.0), Plane::default(), 0.0);
        }

        let (origin, xa, ya, za) = polylines[0].get_average_plane();

        let mut rings2d: Vec<Vec<[f64; 2]>> = Vec::with_capacity(polylines.len());
        let mut sizes: Vec<f64> = Vec::with_capacity(polylines.len());

        for pl in polylines {
            let mut ring = pl.project_to_plane(&origin, &xa, &ya);

            if ring.len() > 1 && pl.point(0).distance(&pl.point(pl.point_count() - 1), None) < 1e-10
            {
                ring.pop();
            }

            let mut mnx = f64::INFINITY;
            let mut mny = f64::INFINITY;
            let mut mxx = f64::NEG_INFINITY;
            let mut mxy = f64::NEG_INFINITY;

            for uv in &ring {
                mnx = mnx.min(uv[0]);
                mxx = mxx.max(uv[0]);
                mny = mny.min(uv[1]);
                mxy = mxy.max(uv[1]);
            }

            rings2d.push(ring);
            sizes.push((mxx - mnx) * (mxx - mnx) + (mxy - mny) * (mxy - mny));
        }

        let mut order: Vec<(f64, usize)> = Vec::with_capacity(rings2d.len());

        for (i, size) in sizes.iter().enumerate() {
            order.push((-size, i));
        }

        order.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mut polygon: Vec<Vec<[f64; 2]>> = Vec::with_capacity(rings2d.len());

        for item in &order {
            polygon.push(std::mem::take(&mut rings2d[item.1]));
        }

        let cr = mapbox_polylabel(&polygon, precision);
        let center = Self::unproject(&origin, &xa, &ya, cr[0], cr[1]);

        (center, Plane::from_frame(origin, xa, ya, za), cr[2])
    }

    /// Return division points on the polylabel circle scaled by scale, oriented to the closest edge or division_direction_in_3d.
    pub fn polylabel_circle_division_points(
        division_direction_in_3d: &Vector,
        polylines: &[Polyline],
        division: usize,
        scale: f64,
        precision: f64,
        orient_to_closest_edge: bool,
    ) -> Vec<Point> {
        let (center, plane, r) = Self::polylabel(polylines, precision);
        let radius = r * scale;
        let is_direction_valid = division_direction_in_3d[0] != 0.0
            || division_direction_in_3d[1] != 0.0
            || division_direction_in_3d[2] != 0.0;
        let (found, edge_i, edge_j) = Self::closest_edge(&center, polylines);
        let found = orient_to_closest_edge && found;
        let mut x_axis = plane.x_axis();
        let mut y_axis = plane.y_axis();
        let mut z_axis = plane.z_axis();

        if is_direction_valid || orient_to_closest_edge {
            let dir = if found {
                &polylines[edge_i].point(edge_j + 1) - &polylines[edge_i].point(edge_j)
            } else {
                division_direction_in_3d.clone()
            };
            x_axis = dir.clone();
            y_axis = dir.cross(&z_axis);
        }

        x_axis.normalize_self();
        y_axis.normalize_self();
        z_axis.normalize_self();

        let mut points = Vec::with_capacity(division);
        let chunk = 360.0 / division as f64;

        for i in 0..division {
            let rad = (45.0 + i as f64 * chunk) * PI / 180.0;
            points.push(Self::unproject(
                &center,
                &x_axis,
                &y_axis,
                radius * rad.cos(),
                radius * rad.sin(),
            ));
        }

        points
    }

    /// Return the Vatti boolean of two closed polylines on x and y, or in plane's local frame; clip_type 0 intersection, 1 union, 2 a minus b.
    pub fn boolean_op(
        a: &Polyline,
        b: &Polyline,
        clip_type: i32,
        plane: Option<&Plane>,
    ) -> Vec<Polyline> {
        let Some(plane) = plane else {
            return crate::boolean_polyline::BooleanPolyline::compute(a, b, clip_type);
        };

        let mut pa2d = Self::boolean_project(a, plane);
        let mut pb2d = Self::boolean_project(b, plane);
        Self::ensure_ccw(&mut pa2d);
        Self::ensure_ccw(&mut pb2d);

        let mut results =
            crate::boolean_polyline::BooleanPolyline::compute(&pa2d, &pb2d, clip_type);
        let o = plane.origin();
        let x = plane.x_axis();
        let y = plane.y_axis();

        for r in results.iter_mut() {
            for i in 0..r.point_count() {
                let p = Self::unproject(&o, &x, &y, r.coords[i * 3], r.coords[i * 3 + 1]);
                r.set_point(i, &p);
            }
        }

        results
    }

    /// Return the Ramer-Douglas-Peucker simplification of a point list.
    pub fn simplify_points(points: &[Point], tolerance: f64) -> Vec<Point> {
        let n = points.len();

        if n < 3 {
            return points.to_vec();
        }

        let mut keep = vec![false; n];
        keep[0] = true;
        keep[n - 1] = true;
        Self::simplify_rdp(points, 0, n - 1, tolerance, &mut keep);

        let mut result = Vec::new();

        for i in 0..n {
            if keep[i] {
                result.push(points[i].clone());
            }
        }

        result
    }

    /// Compute male rect0 and female rect1 cross-sections of radius about p along segment_vector; flip_male rotates the corners.
    pub fn two_rects_from_frame(
        p: &Point,
        segment_vector: &Vector,
        zaxis: &Vector,
        middle: bool,
        radius: f64,
        length: f64,
        flip_male: i32,
    ) -> (Polyline, Polyline) {
        let mut y_axis = zaxis.cross(segment_vector);
        let mut x_axis = y_axis.cross(segment_vector);
        x_axis.normalize_self();
        y_axis.normalize_self();
        let x_axis = &x_axis * radius;
        let y_axis = &y_axis * radius;
        let sv0 = segment_vector * (length * -0.5);
        let sv1 = segment_vector * (length * 0.5);
        let mut v: [Vector; 4] = [
            &(-&x_axis) - &y_axis,
            &x_axis - &y_axis,
            &x_axis + &y_axis,
            &(-&x_axis) + &y_axis,
        ];

        if !middle && flip_male == 1 {
            v.rotate_left(1);
        } else if !middle && flip_male == -1 {
            v.rotate_right(1);
        }

        let rect0 = Polyline::new(vec![
            &(p + &sv0) + &v[1],
            &(p + &sv1) + &v[1],
            &(p + &sv1) + &v[0],
            &(p + &sv0) + &v[0],
            &(p + &sv0) + &v[1],
        ]);
        let rect1 = Polyline::new(vec![
            &(p + &sv0) + &v[2],
            &(p + &sv1) + &v[2],
            &(p + &sv1) + &v[3],
            &(p + &sv0) + &v[3],
            &(p + &sv0) + &v[2],
        ]);

        (rect0, rect1)
    }

    /// Cut two closed 5-point rectangles at plane, keeping the side on the positive half; false when a long edge misses the plane.
    pub fn trim_rectangles_by_plane(
        first: &mut Polyline,
        second: &mut Polyline,
        plane: &Plane,
    ) -> bool {
        if first.point_count() != 5 || second.point_count() != 5 {
            return false;
        }

        let hits = [
            crate::intersection::line_plane(
                &Line::from_points(&first.point(0), &first.point(1)),
                plane,
                false,
            ),
            crate::intersection::line_plane(
                &Line::from_points(&first.point(3), &first.point(2)),
                plane,
                false,
            ),
            crate::intersection::line_plane(
                &Line::from_points(&second.point(0), &second.point(1)),
                plane,
                false,
            ),
            crate::intersection::line_plane(
                &Line::from_points(&second.point(3), &second.point(2)),
                plane,
                false,
            ),
        ];
        let mut points: Vec<Point> = Vec::with_capacity(4);

        for hit in hits {
            let Some(point) = hit else {
                return false;
            };

            for i in 0..3 {
                if !point[i].is_finite() {
                    return false;
                }
            }

            points.push(point);
        }

        if plane.has_on_negative_side(&first.point(0)) {
            first.set_point(0, &points[0]);
            first.set_point(3, &points[1]);
            first.set_point(4, &points[0]);
            second.set_point(0, &points[2]);
            second.set_point(3, &points[3]);
            second.set_point(4, &points[2]);
        } else {
            first.set_point(1, &points[0]);
            first.set_point(2, &points[1]);
            second.set_point(1, &points[2]);
            second.set_point(2, &points[3]);
        }

        true
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from a JSON object.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().expect("Failed to serialize Polyline JSON")
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).expect("Failed to parse Polyline JSON")
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.jsondump()?)?;

        Ok(())
    }

    /// Read from a JSON file.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::Polyline {
        crate::proto::Polyline {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            coords: self.coords.clone(),
            width: self.width,
            dash: self.dash.clone(),
            linecolor: Some(self.linecolor.to_proto()),
        }
    }

    /// Construct from the protobuf message.
    pub fn from_proto(proto: crate::proto::Polyline) -> Self {
        let mut polyline = Self::from_coords(proto.coords);

        if !proto.guid.is_empty() {
            polyline.set_guid(proto.guid);
        }

        polyline.name = proto.name;
        polyline.width = proto.width;
        polyline.dash = proto.dash;

        if let Some(color) = proto.linecolor {
            polyline.linecolor = Color::from_proto(color);
        }

        polyline
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Ok(Self::from_proto(crate::proto::Polyline::decode(data)?))
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&self, filepath: &str) {
        std::fs::write(filepath, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    /// Read from a protobuf file.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");

        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "[(x0, y0, z0), (x1, y1, z1), ...]".
    pub fn str(&self) -> String {
        let mut points = Vec::with_capacity(self.point_count());

        for i in 0..self.point_count() {
            points.push(format!(
                "({}, {}, {})",
                self.coords[i * 3],
                self.coords[i * 3 + 1],
                self.coords[i * 3 + 2]
            ));
        }

        format!("[{}]", points.join(", "))
    }

    /// Return "Polyline(name, N points)".
    pub fn repr(&self) -> String {
        format!("Polyline({}, {} points)", self.name, self.point_count())
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Private helpers
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the point at index, or the origin when out of range.
    fn point(&self, index: usize) -> Point {
        self.get_point(index).unwrap_or_default()
    }

    /// Mark the plane dirty so get_plane recomputes it.
    fn recompute_plane_if_needed(&mut self) {
        self.plane_dirty = true;
    }

    /// Compute the average normal by Newell's method.
    fn average_normal(&self) -> Vector {
        if self.point_count() < 3 {
            return Vector::new(0.0, 0.0, 1.0);
        }

        let n = if self.is_closed() {
            self.point_count() - 1
        } else {
            self.point_count()
        };
        let mut avg_normal = Vector::new(0.0, 0.0, 0.0);

        for i in 0..n {
            let prev = if i == 0 { n - 1 } else { i - 1 };
            let next = (i + 1) % n;
            let v1 = &self.point(i) - &self.point(prev);
            let v2 = &self.point(next) - &self.point(i);
            avg_normal += v1.cross(&v2);
        }

        avg_normal.normalize_self();

        avg_normal
    }

    /// Return the point at arc length distance from the start.
    fn point_at_length(&self, distance: f64) -> Point {
        let mut acc = 0.0;

        for i in 0..self.point_count() - 1 {
            let a = self.point(i);
            let b = self.point(i + 1);
            let seg_len = (&b - &a).magnitude();

            if acc + seg_len >= distance {
                let t = if seg_len > 1e-14 {
                    (distance - acc) / seg_len
                } else {
                    0.0
                };

                return &a + &(&(&b - &a) * t);
            }

            acc += seg_len;
        }

        self.point(0)
    }

    /// Compute the 2D coordinates of the points in the frame (origin, x_axis, y_axis).
    fn project_to_plane(&self, origin: &Point, x_axis: &Vector, y_axis: &Vector) -> Vec<[f64; 2]> {
        let mut pts2d = Vec::with_capacity(self.point_count());

        for i in 0..self.point_count() {
            let d = &self.point(i) - origin;
            pts2d.push([d.dot(x_axis), d.dot(y_axis)]);
        }

        pts2d
    }

    /// Return the 3D point of (u, v) in the frame (origin, x_axis, y_axis).
    fn unproject(origin: &Point, x_axis: &Vector, y_axis: &Vector, u: f64, v: f64) -> Point {
        &(origin + &(x_axis * u)) + &(y_axis * v)
    }

    /// Compute the collinear overlap of two segments as points; false when none or a single point.
    fn line_line_overlap_points(
        line0_start: &Point,
        line0_end: &Point,
        line1_start: &Point,
        line1_end: &Point,
    ) -> (bool, Point, Point) {
        let mut t = [0.0, 1.0, 0.0, 0.0];
        t[2] = Self::closest_point_to_line(line1_start, line0_start, line0_end);
        t[3] = Self::closest_point_to_line(line1_end, line0_start, line0_end);
        let mut do_overlap = !((t[2] < 0.0 && t[3] < 0.0) || (t[2] > 1.0 && t[3] > 1.0));
        t.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        do_overlap = do_overlap && (t[2] - t[1]).abs() > Tolerance::ZERO_TOLERANCE;
        let overlap_start = Self::point_at(line0_start, line0_end, t[1]);
        let overlap_end = Self::point_at(line0_start, line0_end, t[2]);

        (do_overlap, overlap_start, overlap_end)
    }

    /// Add the hull points of pts right of the segment (a, b) to hull.
    fn quick_hull_recurse(
        pts: &[[f64; 2]],
        ax: f64,
        ay: f64,
        bx: f64,
        by: f64,
        hull: &mut Vec<[f64; 2]>,
    ) {
        if pts.is_empty() {
            return;
        }

        let mut fi = 0;
        let mut best = f64::NEG_INFINITY;

        for (i, p) in pts.iter().enumerate() {
            let val = ccw_2d(ax, ay, bx, by, p[0], p[1]);

            if val >= best {
                best = val;
                fi = i;
            }
        }

        let fx = pts[fi][0];
        let fy = pts[fi][1];
        let mut left = Vec::new();
        let mut right = Vec::new();

        for p in pts {
            if ccw_2d(ax, ay, fx, fy, p[0], p[1]) > 0.0 {
                left.push(*p);
            }

            if ccw_2d(fx, fy, bx, by, p[0], p[1]) > 0.0 {
                right.push(*p);
            }
        }

        Self::quick_hull_recurse(&left, ax, ay, fx, fy, hull);
        hull.push([fx, fy]);
        Self::quick_hull_recurse(&right, fx, fy, bx, by, hull);
    }

    /// Miter-offset a 2D polygon in place by offset_dist.
    fn offset_polygon_2d(poly2d: &mut Vec<[f64; 2]>, offset_dist: f64) {
        let n = poly2d.len();

        if offset_dist == 0.0 || n < 3 {
            return;
        }

        let mut signed_area = 0.0;

        for i in 0..n {
            let a = poly2d[i];
            let b = poly2d[(i + 1) % n];
            signed_area += a[0] * b[1] - b[0] * a[1];
        }

        let delta = if signed_area < 0.0 {
            -offset_dist
        } else {
            offset_dist
        };
        let mut normals: Vec<[f64; 2]> = Vec::with_capacity(n);

        for i in 0..n {
            let a = poly2d[i];
            let b = poly2d[(i + 1) % n];
            let ex = b[0] - a[0];
            let ey = b[1] - a[1];
            let len = (ex * ex + ey * ey).sqrt();

            if len < 1e-12 {
                normals.push([0.0, 0.0]);
            } else {
                normals.push([ey / len, -ex / len]);
            }
        }

        let mut out: Vec<[f64; 2]> = Vec::with_capacity(n * 3);

        for i in 0..n {
            let np = normals[(i + n - 1) % n];
            let nn = normals[i];
            let cos_a = np[0] * nn[0] + np[1] * nn[1];
            let sin_a = np[0] * nn[1] - np[1] * nn[0];
            let denom = 1.0 + cos_a;
            let concave = cos_a > -0.999 && sin_a * delta < 0.0 && offset_dist > 0.0;

            if concave {
                out.push([poly2d[i][0] + np[0] * delta, poly2d[i][1] + np[1] * delta]);
                out.push([poly2d[i][0], poly2d[i][1]]);
                out.push([poly2d[i][0] + nn[0] * delta, poly2d[i][1] + nn[1] * delta]);
            } else if denom.abs() < 1e-9 {
                out.push([
                    poly2d[i][0] + (np[0] + nn[0]) * 0.5 * delta,
                    poly2d[i][1] + (np[1] + nn[1]) * 0.5 * delta,
                ]);
            } else {
                out.push([
                    poly2d[i][0] + (np[0] + nn[0]) / denom * delta,
                    poly2d[i][1] + (np[1] + nn[1]) / denom * delta,
                ]);
            }
        }

        let mut out_area = 0.0;

        for i in 0..out.len() {
            let a = out[i];
            let b = out[(i + 1) % out.len()];
            out_area += a[0] * b[1] - b[0] * a[1];
        }

        if out.len() >= 3 && out_area.abs() > 1e-4 {
            *poly2d = out;
        }
    }

    /// Return whether (px, py) is inside poly2d by ray crossing.
    fn point_in_polygon(poly2d: &[[f64; 2]], px: f64, py: f64) -> bool {
        let n = poly2d.len();
        let mut inside = false;
        let mut j = n - 1;

        for i in 0..n {
            let xi = poly2d[i][0];
            let yi = poly2d[i][1];
            let xj = poly2d[j][0];
            let yj = poly2d[j][1];

            if (yi > py) != (yj > py) && px < (xj - xi) * (py - yi) / (yj - yi) + xi {
                inside = !inside;
            }

            j = i;
        }

        inside
    }

    /// Find the polyline and edge closest to center.
    fn closest_edge(center: &Point, polylines: &[Polyline]) -> (bool, usize, usize) {
        let mut edge_i = 0;
        let mut edge_j = 0;
        let mut best_sq = f64::INFINITY;

        for (i, pl) in polylines.iter().enumerate() {
            for j in 0..pl.point_count().saturating_sub(1) {
                let a = pl.point(j);
                let e = &pl.point(j + 1) - &a;
                let len2 = e.magnitude_squared();

                if len2 <= 0.0 {
                    continue;
                }

                let t = (center - &a).dot(&e) / len2;

                if !(0.0..=1.0).contains(&t) {
                    continue;
                }

                let d2 = (center - &(&a + &(&e * t))).magnitude_squared();

                if d2 < best_sq {
                    best_sq = d2;
                    edge_i = i;
                    edge_j = j;
                }
            }
        }

        (best_sq < f64::INFINITY, edge_i, edge_j)
    }

    /// Return pl projected into plane's local frame as 2D.
    fn boolean_project(pl: &Polyline, plane: &Plane) -> Polyline {
        let o = plane.origin();
        let x = plane.x_axis();
        let y = plane.y_axis();
        let n = pl.point_count();
        let mut p2d = Polyline::default();
        p2d.coords.resize(n * 3, 0.0);

        for i in 0..n {
            let d = &pl.point(i) - &o;
            p2d.coords[i * 3] = d.dot(&x);
            p2d.coords[i * 3 + 1] = d.dot(&y);
            p2d.coords[i * 3 + 2] = 0.0;
        }

        if n >= 4 {
            let dx = p2d.coords[(n - 1) * 3] - p2d.coords[0];
            let dy = p2d.coords[(n - 1) * 3 + 1] - p2d.coords[1];

            if dx * dx + dy * dy < 1.0 {
                p2d.coords[(n - 1) * 3] = p2d.coords[0];
                p2d.coords[(n - 1) * 3 + 1] = p2d.coords[1];
            }
        }

        p2d
    }

    /// Reverse p2d in place when it winds clockwise.
    fn ensure_ccw(p2d: &mut Polyline) {
        let n = p2d.point_count();
        let mut m = n;

        if m >= 4 {
            let dx = p2d.coords[(m - 1) * 3] - p2d.coords[0];
            let dy = p2d.coords[(m - 1) * 3 + 1] - p2d.coords[1];

            if dx * dx + dy * dy < 1e-10 {
                m -= 1;
            }
        }

        if m < 3 {
            return;
        }

        let mut area = 0.0;

        for i in 0..m {
            let j = (i + 1) % m;
            area += p2d.coords[i * 3] * p2d.coords[j * 3 + 1]
                - p2d.coords[j * 3] * p2d.coords[i * 3 + 1];
        }

        if area < 0.0 {
            p2d.reverse();
        }
    }

    /// Return the perpendicular distance from pt to the line through line_start and line_end.
    fn simplify_perp_dist(pt: &Point, line_start: &Point, line_end: &Point) -> f64 {
        let d = line_end - line_start;
        let len_sq = d.magnitude_squared();

        if len_sq == 0.0 {
            return (pt - line_start).magnitude_squared().sqrt();
        }

        let mut t = (pt - line_start).dot(&d) / len_sq;
        t = t.clamp(0.0, 1.0);

        (pt - &(line_start + &(&d * t))).magnitude_squared().sqrt()
    }

    /// Mark the points to keep between start and end by recursive Ramer-Douglas-Peucker.
    fn simplify_rdp(points: &[Point], start: usize, end: usize, tolerance: f64, keep: &mut [bool]) {
        if end <= start + 1 {
            return;
        }

        let mut max_dist = 0.0_f64;
        let mut max_idx = start;

        for i in (start + 1)..end {
            let d = Self::simplify_perp_dist(&points[i], &points[start], &points[end]);

            if d > max_dist {
                max_dist = d;
                max_idx = i;
            }
        }

        if max_dist <= tolerance {
            return;
        }

        keep[max_idx] = true;
        Self::simplify_rdp(points, start, max_idx, tolerance, keep);
        Self::simplify_rdp(points, max_idx, end, tolerance, keep);
    }
}

impl<'de> Deserialize<'de> for Polyline {
    /// Deserialize from flat JSON fields.
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct PolylineData {
            #[serde(default)]
            guid: Option<String>,
            #[serde(default)]
            name: Option<String>,
            #[serde(default)]
            coords: Option<Vec<f64>>,
            #[serde(default)]
            points: Option<Vec<Point>>,
            #[serde(default)]
            width: Option<f64>,
            #[serde(default)]
            dash: Vec<f64>,
            #[serde(default)]
            linecolor: Option<Color>,
        }

        let data = PolylineData::deserialize(deserializer)?;
        let mut polyline = Polyline::default();

        if let Some(guid) = data.guid {
            polyline.set_guid(guid);
        }

        if let Some(name) = data.name {
            polyline.name = name;
        }

        if let Some(coords) = data.coords {
            polyline.coords = coords;
        } else if let Some(points) = data.points {
            for point in points {
                polyline.add_point(point);
            }
        }

        if let Some(width) = data.width {
            polyline.width = width;
        }

        polyline.dash = data.dash;

        if let Some(linecolor) = data.linecolor {
            polyline.linecolor = linecolor;
        }

        polyline.recompute_plane_if_needed();

        Ok(polyline)
    }
}

impl fmt::Display for Polyline {
    /// Write the polyline string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.repr())
    }
}
