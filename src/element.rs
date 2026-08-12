use crate::brep::BRep;
use crate::line::Line;
use crate::mesh::Mesh;
use crate::obb::OBB;
use crate::plane::Plane;
use crate::point::Point;
use crate::polyline::Polyline;
use crate::vector::Vector;
use crate::xform::Xform;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::fs;

#[derive(Debug, Clone)]
pub enum ElementGeometry {
    None,
    Mesh(Mesh),
    BRep(BRep),
}

// Strip closing duplicate if polygon is closed (last == first)
fn strip_closing(pts: &[Point]) -> Vec<Point> {
    if pts.len() > 3 {
        let f = &pts[0];
        let l = &pts[pts.len() - 1];
        if (f[0] - l[0]).abs() < 1e-6 && (f[1] - l[1]).abs() < 1e-6 && (f[2] - l[2]).abs() < 1e-6 {
            return pts[..pts.len() - 1].to_vec();
        }
    }
    pts.to_vec()
}

#[derive(Debug, Clone)]
pub enum ElementKind {
    Generic,
    Column { width: f64, depth: f64, height: f64 },
    Beam { width: f64, depth: f64, length: f64 },
    Plate {
        polygon: Vec<Point>,
        polygon_top: Vec<Point>,
        thickness: f64,
        joint_types: Vec<i32>,
        j_mf: Vec<Vec<(i32, bool, f64)>>,
        key: String,
        component_plane: Option<Plane>,
    },
}

#[derive(Debug, Clone)]
pub struct Element {
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub kind: ElementKind,
    geometry: ElementGeometry,
    features: Vec<fn(Mesh) -> Mesh>,
    is_dirty: bool,
    cached_aabb: Option<OBB>,
    cached_obb: Option<OBB>,
    cached_collision_mesh: Option<Mesh>,
    cached_point: Option<Point>,
    cached_polylines: Option<Vec<Polyline>>,
    cached_planes: Option<Vec<Plane>>,
    cached_edge_vectors: Option<Vec<Vector>>,
    cached_axis: Option<Line>,
}

impl Element {
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn new(name: &str) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            kind: ElementKind::Generic,
            geometry: ElementGeometry::None,
            features: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
            cached_polylines: None,
            cached_planes: None,
            cached_edge_vectors: None,
            cached_axis: None,
        }
    }


    pub fn from_mesh(geometry: Mesh, name: &str) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            kind: ElementKind::Generic,
            geometry: ElementGeometry::Mesh(geometry),
            features: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
            cached_polylines: None,
            cached_planes: None,
            cached_edge_vectors: None,
            cached_axis: None,
        }
    }


    pub fn from_brep(geometry: BRep, name: &str) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            kind: ElementKind::Generic,
            geometry: ElementGeometry::BRep(geometry),
            features: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
            cached_polylines: None,
            cached_planes: None,
            cached_edge_vectors: None,
            cached_axis: None,
        }
    }


    pub fn column(width: f64, depth: f64, height: f64, name: &str) -> Self {
        let kind = ElementKind::Column { width, depth, height };
        let geometry = ElementGeometry::Mesh(Self::compute_box_geometry(width, depth, height));
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            kind,
            geometry,
            features: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
            cached_polylines: None,
            cached_planes: None,
            cached_edge_vectors: None,
            cached_axis: None,
        }
    }


    pub fn beam(width: f64, depth: f64, length: f64, name: &str) -> Self {
        let kind = ElementKind::Beam { width, depth, length };
        let geometry = ElementGeometry::Mesh(Self::compute_box_geometry(width, depth, length));
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            kind,
            geometry,
            features: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
            cached_polylines: None,
            cached_planes: None,
            cached_edge_vectors: None,
            cached_axis: None,
        }
    }


    pub fn plate(polygon: Vec<Point>, thickness: f64, name: &str) -> Self {
        let pts: Vec<Point> = polygon.iter().map(|p| Point::new(p[0], p[1], p[2])).collect();
        let polygon_top = Self::offset_polygon_top(&pts, thickness);
        let geometry = ElementGeometry::Mesh(Self::compute_plate_geometry(&pts, thickness));
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            kind: ElementKind::Plate { polygon: pts, polygon_top, thickness, joint_types: Vec::new(), j_mf: Vec::new(), key: String::new(), component_plane: None },
            geometry,
            features: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
            cached_polylines: None,
            cached_planes: None,
            cached_edge_vectors: None,
            cached_axis: None,
        }
    }


    pub fn plate_from_top_bottom(bottom: Vec<Point>, top: Vec<Point>, name: &str) -> Self {
        let mut bot = strip_closing(&bottom);
        let mut tp = strip_closing(&top);
        // Ensure bottom normal points toward top
        let norm = Self::polygon_normal(&bot);
        let np = bot.len().min(tp.len());
        let mut d = 0.0;
        for k in 0..np {
            d += (tp[k][0] - bot[k][0]) * norm[0]
               + (tp[k][1] - bot[k][1]) * norm[1]
               + (tp[k][2] - bot[k][2]) * norm[2];
        }
        if d < 0.0 { std::mem::swap(&mut bot, &mut tp); }
        // Compute thickness from average distance
        let mut thickness = 0.0;
        for k in 0..np {
            let dx = tp[k][0] - bot[k][0];
            let dy = tp[k][1] - bot[k][1];
            let dz = tp[k][2] - bot[k][2];
            thickness += (dx * dx + dy * dy + dz * dz).sqrt();
        }
        thickness /= np as f64;
        let geometry = ElementGeometry::Mesh(Self::compute_plate_geometry_explicit(&bot, &tp));
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            kind: ElementKind::Plate { polygon: bot, polygon_top: tp, thickness, joint_types: Vec::new(), j_mf: Vec::new(), key: String::new(), component_plane: None },
            geometry,
            features: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
            cached_polylines: None,
            cached_planes: None,
            cached_edge_vectors: None,
            cached_axis: None,
        }
    }


    pub fn plate_default() -> Self {
        let polygon = vec![
            Point::new(-0.5, -0.5, 0.0), Point::new(0.5, -0.5, 0.0),
            Point::new(0.5, 0.5, 0.0),   Point::new(-0.5, 0.5, 0.0),
        ];
        Self::plate(polygon, 0.1, "my_plate")
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    pub fn duplicate(&self) -> Self {
        let mut result = self.clone();
        result.guid = std::sync::OnceLock::new();
        result.is_dirty = true;
        result.cached_aabb = None;
        result.cached_obb = None;
        result.cached_collision_mesh = None;
        result.cached_point = None;
        result.cached_polylines = None;
        result.cached_planes = None;
        result.cached_edge_vectors = None;
        result.cached_axis = None;
        result
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Properties
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn geometry(&self) -> &ElementGeometry { &self.geometry }
    pub fn is_dirty(&self) -> bool { self.is_dirty }

    pub fn has_geometry(&self) -> bool {
        !matches!(self.geometry, ElementGeometry::None)
    }

    pub fn geometry_type_name(&self) -> &str {
        match &self.geometry {
            ElementGeometry::None => "None",
            ElementGeometry::Mesh(_) => "Mesh",
            ElementGeometry::BRep(_) => "BRep",
        }
    }

    /// The element's geometry placed by `xform`. The placement is supplied by the caller —
    /// an Element no longer stores one; the Session does. Pass identity for local geometry.
    pub fn session_geometry(&self, xform: &Xform) -> ElementGeometry {
        match &self.geometry {
            ElementGeometry::None => ElementGeometry::None,
            ElementGeometry::Mesh(mesh) => {
                let mut geo = mesh.clone();
                for f in &self.features { geo = f(geo); }
                if !xform.is_identity() {
                    geo.transform(xform);
                }
                ElementGeometry::Mesh(geo)
            }
            ElementGeometry::BRep(brep) => {
                let mut geo = brep.clone();
                if !xform.is_identity() {
                    geo.transform(xform);
                }
                ElementGeometry::BRep(geo)
            }
        }
    }

    pub fn aabb(&mut self) -> OBB {
        if self.is_dirty || self.cached_aabb.is_none() {
            self.cached_aabb = Some(self.compute_aabb());
        }
        self.cached_aabb.clone().unwrap()
    }

    pub fn obb(&mut self) -> OBB {
        if self.is_dirty || self.cached_obb.is_none() {
            self.cached_obb = Some(self.compute_obb());
        }
        self.cached_obb.clone().unwrap()
    }

    pub fn collision_mesh(&mut self) -> Mesh {
        if self.is_dirty || self.cached_collision_mesh.is_none() {
            self.cached_collision_mesh = Some(self.compute_collision_mesh());
        }
        self.cached_collision_mesh.clone().unwrap()
    }

    pub fn point(&mut self) -> Point {
        if self.is_dirty || self.cached_point.is_none() {
            self.cached_point = Some(self.compute_point());
        }
        self.cached_point.clone().unwrap()
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Column/Beam/Plate Accessors
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn width(&self) -> Option<f64> {
        match &self.kind {
            ElementKind::Column { width, .. } | ElementKind::Beam { width, .. } => Some(*width),
            _ => None,
        }
    }

    pub fn depth(&self) -> Option<f64> {
        match &self.kind {
            ElementKind::Column { depth, .. } | ElementKind::Beam { depth, .. } => Some(*depth),
            _ => None,
        }
    }

    pub fn height(&self) -> Option<f64> {
        match &self.kind {
            ElementKind::Column { height, .. } => Some(*height),
            _ => None,
        }
    }

    pub fn length(&self) -> Option<f64> {
        match &self.kind {
            ElementKind::Beam { length, .. } => Some(*length),
            _ => None,
        }
    }

    pub fn thickness(&self) -> Option<f64> {
        match &self.kind {
            ElementKind::Plate { thickness, .. } => Some(*thickness),
            _ => None,
        }
    }

    pub fn polygon(&self) -> Option<&Vec<Point>> {
        match &self.kind {
            ElementKind::Plate { polygon, .. } => Some(polygon),
            _ => None,
        }
    }

    pub fn polygon_top(&self) -> Option<&Vec<Point>> {
        match &self.kind {
            ElementKind::Plate { polygon_top, .. } => Some(polygon_top),
            _ => None,
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Column/Beam/Plate Setters
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn set_width(&mut self, v: f64) {
        match &mut self.kind {
            ElementKind::Column { width, depth, height } => {
                *width = v;
                self.geometry = ElementGeometry::Mesh(Self::compute_box_geometry(*width, *depth, *height));
            }
            ElementKind::Beam { width, depth, length } => {
                *width = v;
                self.geometry = ElementGeometry::Mesh(Self::compute_box_geometry(*width, *depth, *length));
            }
            _ => {}
        }
        self.reset();
    }

    pub fn set_depth(&mut self, v: f64) {
        match &mut self.kind {
            ElementKind::Column { width, depth, height } => {
                *depth = v;
                self.geometry = ElementGeometry::Mesh(Self::compute_box_geometry(*width, *depth, *height));
            }
            ElementKind::Beam { width, depth, length } => {
                *depth = v;
                self.geometry = ElementGeometry::Mesh(Self::compute_box_geometry(*width, *depth, *length));
            }
            _ => {}
        }
        self.reset();
    }

    pub fn set_height(&mut self, v: f64) {
        if let ElementKind::Column { width, depth, height } = &mut self.kind {
            *height = v;
            self.geometry = ElementGeometry::Mesh(Self::compute_box_geometry(*width, *depth, *height));
            self.reset();
        }
    }

    pub fn set_length(&mut self, v: f64) {
        if let ElementKind::Beam { width, depth, length } = &mut self.kind {
            *length = v;
            self.geometry = ElementGeometry::Mesh(Self::compute_box_geometry(*width, *depth, *length));
            self.reset();
        }
    }

    pub fn set_thickness(&mut self, v: f64) {
        if let ElementKind::Plate { polygon, polygon_top, thickness, .. } = &mut self.kind {
            *thickness = v;
            *polygon_top = Self::offset_polygon_top(polygon, *thickness);
            self.geometry = ElementGeometry::Mesh(Self::compute_plate_geometry(polygon, *thickness));
            self.reset();
        }
    }

    pub fn set_polygon(&mut self, pts: Vec<Point>) {
        if let ElementKind::Plate { polygon, polygon_top, thickness, .. } = &mut self.kind {
            *polygon = pts.iter().map(|p| Point::new(p[0], p[1], p[2])).collect();
            *polygon_top = Self::offset_polygon_top(polygon, *thickness);
            self.geometry = ElementGeometry::Mesh(Self::compute_plate_geometry(polygon, *thickness));
            self.reset();
        }
    }

    pub fn center_line(&self) -> Option<Line> {
        match &self.kind {
            ElementKind::Column { height, .. } => Some(Line::new(0.0, 0.0, 0.0, 0.0, 0.0, *height)),
            ElementKind::Beam { length, .. } => Some(Line::new(0.0, 0.0, 0.0, 0.0, 0.0, *length)),
            _ => None,
        }
    }

    pub fn joint_types(&self) -> Option<&Vec<i32>> {
        match &self.kind {
            ElementKind::Plate { joint_types, .. } => Some(joint_types),
            _ => None,
        }
    }

    pub fn set_joint_types(&mut self, v: Vec<i32>) {
        if let ElementKind::Plate { joint_types, .. } = &mut self.kind {
            *joint_types = v;
        }
    }

    pub fn j_mf(&self) -> Option<&Vec<Vec<(i32, bool, f64)>>> {
        match &self.kind {
            ElementKind::Plate { j_mf, .. } => Some(j_mf),
            _ => None,
        }
    }

    pub fn set_j_mf(&mut self, v: Vec<Vec<(i32, bool, f64)>>) {
        if let ElementKind::Plate { j_mf, .. } = &mut self.kind {
            *j_mf = v;
        }
    }

    pub fn key(&self) -> Option<&str> {
        match &self.kind {
            ElementKind::Plate { key, .. } => Some(key),
            _ => None,
        }
    }

    pub fn set_key(&mut self, v: String) {
        if let ElementKind::Plate { key, .. } = &mut self.kind {
            *key = v;
        }
    }

    pub fn component_plane(&self) -> Option<&Plane> {
        match &self.kind {
            ElementKind::Plate { component_plane, .. } => component_plane.as_ref(),
            _ => None,
        }
    }

    pub fn set_component_plane(&mut self, v: Plane) {
        if let ElementKind::Plate { component_plane, .. } = &mut self.kind {
            *component_plane = Some(v);
        }
    }

    pub fn extend(&mut self, distance: f64) {
        match &mut self.kind {
            ElementKind::Column { width, depth, height } => {
                *height += distance * 2.0;
                self.geometry = ElementGeometry::Mesh(Self::compute_box_geometry(*width, *depth, *height));
            }
            ElementKind::Beam { width, depth, length } => {
                *length += distance * 2.0;
                self.geometry = ElementGeometry::Mesh(Self::compute_box_geometry(*width, *depth, *length));
            }
            _ => {}
        }
        self.reset();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Mutators
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_feature(&mut self, f: fn(Mesh) -> Mesh) {
        self.features.push(f);
        self.is_dirty = true;
    }

    /// Bake a placement into this element's own geometry, invalidating the cached boxes.
    /// The Session owns the placement, so it hands it in here rather than the Element storing it.
    pub fn place(&mut self, xform: &Xform) {
        self.geometry = self.session_geometry(xform);
        self.is_dirty = true;
    }

    pub fn set_geometry(&mut self, geo: Mesh) {
        self.geometry = ElementGeometry::Mesh(geo);
        self.is_dirty = true;
    }

    pub fn set_brep_geometry(&mut self, geo: BRep) {
        self.geometry = ElementGeometry::BRep(geo);
        self.is_dirty = true;
    }

    pub fn set_polylines(&mut self, polys: Vec<Polyline>) {
        self.cached_polylines = Some(polys);
    }

    pub fn set_planes(&mut self, plns: Vec<Plane>) {
        self.cached_planes = Some(plns);
    }

    pub fn reset(&mut self) {
        self.is_dirty = true;
        self.cached_aabb = None;
        self.cached_obb = None;
        self.cached_collision_mesh = None;
        self.cached_point = None;
        self.cached_polylines = None;
        self.cached_planes = None;
        self.cached_edge_vectors = None;
        self.cached_axis = None;
    }

    pub fn features_count(&self) -> usize { self.features.len() }
    pub fn cached_aabb_ref(&self) -> &Option<OBB> { &self.cached_aabb }
    pub fn cached_obb_ref(&self) -> &Option<OBB> { &self.cached_obb }
    pub fn cached_collision_mesh_ref(&self) -> &Option<Mesh> { &self.cached_collision_mesh }
    pub fn cached_point_ref(&self) -> &Option<Point> { &self.cached_point }

    pub fn polylines(&mut self) -> Vec<Polyline> {
        if self.is_dirty || self.cached_polylines.is_none() {
            self.cached_polylines = Some(self.compute_polylines());
        }
        self.cached_polylines.clone().unwrap()
    }

    pub fn planes(&mut self) -> Vec<Plane> {
        if self.is_dirty || self.cached_planes.is_none() {
            self.cached_planes = Some(self.compute_planes());
        }
        self.cached_planes.clone().unwrap()
    }

    pub fn edge_vectors(&mut self) -> Vec<Vector> {
        if self.is_dirty || self.cached_edge_vectors.is_none() {
            self.cached_edge_vectors = Some(self.compute_edge_vectors());
        }
        self.cached_edge_vectors.clone().unwrap()
    }

    pub fn axis(&mut self) -> Option<Line> {
        if self.is_dirty || self.cached_axis.is_none() {
            self.cached_axis = self.compute_axis();
        }
        self.cached_axis.clone()
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Computation
    ///////////////////////////////////////////////////////////////////////////////////////////

    fn compute_aabb(&self) -> OBB {
        let geo = self.session_geometry(&Xform::identity());
        if matches!(geo, ElementGeometry::None) {
            return OBB::from_point(Point::new(0.0, 0.0, 0.0), 0.0);
        }
        Self::obb_from_geometry(&geo)
    }

    fn compute_obb(&self) -> OBB {
        let geo = self.session_geometry(&Xform::identity());
        if matches!(geo, ElementGeometry::None) {
            return OBB::from_point(Point::new(0.0, 0.0, 0.0), 0.0);
        }
        Self::obb_from_geometry(&geo)
    }

    fn compute_collision_mesh(&self) -> Mesh {
        if let ElementGeometry::Mesh(mesh) = self.session_geometry(&Xform::identity()) { return mesh; }
        Mesh::new()
    }

    fn compute_point(&self) -> Point {
        let geo = self.session_geometry(&Xform::identity());
        match &geo {
            ElementGeometry::Mesh(mesh) => {
                if mesh.vertex.is_empty() { return Point::new(0.0, 0.0, 0.0); }
                let (mut sx, mut sy, mut sz) = (0.0, 0.0, 0.0);
                for (_, v) in &mesh.vertex {
                    sx += v.x; sy += v.y; sz += v.z;
                }
                let n = mesh.vertex.len() as f64;
                Point::new(sx / n, sy / n, sz / n)
            }
            ElementGeometry::BRep(brep) => {
                if brep.m_vertices.is_empty() { return Point::new(0.0, 0.0, 0.0); }
                let (mut sx, mut sy, mut sz) = (0.0, 0.0, 0.0);
                for p in &brep.m_vertices {
                    sx += p[0]; sy += p[1]; sz += p[2];
                }
                let n = brep.m_vertices.len() as f64;
                Point::new(sx / n, sy / n, sz / n)
            }
            ElementGeometry::None => Point::new(0.0, 0.0, 0.0),
        }
    }

    fn obb_from_geometry(geo: &ElementGeometry) -> OBB {
        let inflate = 0.0;
        match geo {
            ElementGeometry::Mesh(mesh) => {
                let points: Vec<Point> = mesh.vertex.values().map(|v| v.position()).collect();
                if points.is_empty() { return OBB::from_point(Point::new(0.0, 0.0, 0.0), inflate); }
                OBB::from_points(&points, inflate)
            }
            ElementGeometry::BRep(brep) => {
                if brep.m_vertices.is_empty() { return OBB::from_point(Point::new(0.0, 0.0, 0.0), inflate); }
                OBB::from_points(&brep.m_vertices, inflate)
            }
            ElementGeometry::None => OBB::from_point(Point::new(0.0, 0.0, 0.0), inflate),
        }
    }

    fn compute_box_geometry(width: f64, depth: f64, z_extent: f64) -> Mesh {
        let hx = width * 0.5;
        let hy = depth * 0.5;
        let vertices = vec![
            Point::new(-hx, -hy, 0.0),         Point::new(hx, -hy, 0.0),
            Point::new(hx, hy, 0.0),           Point::new(-hx, hy, 0.0),
            Point::new(-hx, -hy, z_extent),    Point::new(hx, -hy, z_extent),
            Point::new(hx, hy, z_extent),      Point::new(-hx, hy, z_extent),
        ];
        let faces = vec![
            vec![0, 3, 2, 1], vec![4, 5, 6, 7],
            vec![0, 1, 5, 4], vec![2, 3, 7, 6],
            vec![0, 4, 7, 3], vec![1, 2, 6, 5],
        ];
        Mesh::from_vertices_and_faces(vertices, faces)
    }

    pub fn polygon_normal(pts: &[Point]) -> Vector {
        let (mut nx, mut ny, mut nz) = (0.0, 0.0, 0.0);
        let n = pts.len();
        for i in 0..n {
            let c = &pts[i];
            let nx_pt = &pts[(i + 1) % n];
            nx += (c[1] - nx_pt[1]) * (c[2] + nx_pt[2]);
            ny += (c[2] - nx_pt[2]) * (c[0] + nx_pt[0]);
            nz += (c[0] - nx_pt[0]) * (c[1] + nx_pt[1]);
        }
        let mag = (nx * nx + ny * ny + nz * nz).sqrt();
        if mag < 1e-12 { return Vector::new(0.0, 0.0, 1.0); }
        Vector::new(nx / mag, ny / mag, nz / mag)
    }

    pub fn compute_aabb_fast(&self, inflate: f64) -> OBB {
        match &self.kind {
            ElementKind::Plate { polygon, polygon_top, .. } => {
                let mut pts: Vec<Point> = Vec::with_capacity(polygon.len() + polygon_top.len());
                for p in polygon { pts.push(Point::new(p[0], p[1], p[2])); }
                for p in polygon_top { pts.push(Point::new(p[0], p[1], p[2])); }
                OBB::from_points(&pts, inflate)
            }
            _ => OBB::from_point(Point::new(0.0, 0.0, 0.0), inflate),
        }
    }

    fn offset_polygon_top(polygon: &[Point], thickness: f64) -> Vec<Point> {
        let normal = Self::polygon_normal(polygon);
        polygon.iter().map(|p| Point::new(
            p[0] - normal[0] * thickness,
            p[1] - normal[1] * thickness,
            p[2] - normal[2] * thickness,
        )).collect()
    }

    fn compute_plate_geometry(polygon: &[Point], thickness: f64) -> Mesh {
        let top = Self::offset_polygon_top(polygon, thickness);
        Self::compute_plate_geometry_explicit(polygon, &top)
    }

    fn compute_plate_geometry_explicit(bottom: &[Point], top: &[Point]) -> Mesh {
        let n = bottom.len().min(top.len());
        let mut vertices = Vec::with_capacity(n * 2);
        for i in 0..n {
            vertices.push(Point::new(bottom[i][0], bottom[i][1], bottom[i][2]));
        }
        for i in 0..n {
            vertices.push(Point::new(top[i][0], top[i][1], top[i][2]));
        }
        let mut faces = Vec::new();
        let bottom_face: Vec<usize> = (0..n).rev().collect();
        let top_face: Vec<usize> = (n..2 * n).collect();
        faces.push(bottom_face);
        faces.push(top_face);
        for i in 0..n {
            let a = i;
            let b = (i + 1) % n;
            let c = b + n;
            let d = a + n;
            faces.push(vec![a, b, c, d]);
        }
        Mesh::from_vertices_and_faces(vertices, faces)
    }

    fn compute_polylines(&self) -> Vec<Polyline> {
        match &self.kind {
            ElementKind::Column { width, depth, .. } | ElementKind::Beam { width, depth, .. } => {
                let z = match &self.kind {
                    ElementKind::Column { height, .. } => *height,
                    ElementKind::Beam { length, .. } => *length,
                    _ => 0.0,
                };
                let hx = width * 0.5;
                let hy = depth * 0.5;
                let b = [Point::new(-hx, -hy, 0.0), Point::new(hx, -hy, 0.0), Point::new(hx, hy, 0.0), Point::new(-hx, hy, 0.0)];
                let t = [Point::new(-hx, -hy, z), Point::new(hx, -hy, z), Point::new(hx, hy, z), Point::new(-hx, hy, z)];
                vec![
                    Polyline::new(vec![b[0].clone(), b[3].clone(), b[2].clone(), b[1].clone(), b[0].clone()]),
                    Polyline::new(vec![t[0].clone(), t[1].clone(), t[2].clone(), t[3].clone(), t[0].clone()]),
                    Polyline::new(vec![b[0].clone(), b[1].clone(), t[1].clone(), t[0].clone(), b[0].clone()]),
                    Polyline::new(vec![b[2].clone(), b[3].clone(), t[3].clone(), t[2].clone(), b[2].clone()]),
                    Polyline::new(vec![b[0].clone(), t[0].clone(), t[3].clone(), b[3].clone(), b[0].clone()]),
                    Polyline::new(vec![b[1].clone(), b[2].clone(), t[2].clone(), t[1].clone(), b[1].clone()]),
                ]
            }
            ElementKind::Plate { polygon, polygon_top, .. } => {
                let n = polygon.len().min(polygon_top.len());
                let bottom: Vec<Point> = polygon.iter().take(n).map(|p| Point::new(p[0], p[1], p[2])).collect();
                let top: Vec<Point> = polygon_top.iter().take(n).map(|p| Point::new(p[0], p[1], p[2])).collect();
                // [0]=top polyline, [1]=bottom polyline, [2+]=side polylines (matching wood)
                let mut top_pl = top.clone();
                top_pl.push(top[0].clone());
                let mut bot_pl = bottom.clone();
                bot_pl.push(bottom[0].clone());
                let mut result = vec![Polyline::new(top_pl), Polyline::new(bot_pl)];
                for i in 0..n {
                    let j = (i + 1) % n;
                    result.push(Polyline::new(vec![
                        top[i].clone(), top[j].clone(), bottom[j].clone(), bottom[i].clone(), top[i].clone(),
                    ]));
                }
                result
            }
            _ => Vec::new(),
        }
    }

    fn compute_planes(&self) -> Vec<Plane> {
        match &self.kind {
            ElementKind::Column { width, depth, .. } | ElementKind::Beam { width, depth, .. } => {
                let z = match &self.kind {
                    ElementKind::Column { height, .. } => *height,
                    ElementKind::Beam { length, .. } => *length,
                    _ => 0.0,
                };
                let hx = width * 0.5;
                let hy = depth * 0.5;
                let hz = z * 0.5;
                vec![
                    Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, -1.0)),
                    Plane::from_point_normal(Point::new(0.0, 0.0, z), Vector::new(0.0, 0.0, 1.0)),
                    Plane::from_point_normal(Point::new(0.0, -hy, hz), Vector::new(0.0, -1.0, 0.0)),
                    Plane::from_point_normal(Point::new(0.0, hy, hz), Vector::new(0.0, 1.0, 0.0)),
                    Plane::from_point_normal(Point::new(-hx, 0.0, hz), Vector::new(-1.0, 0.0, 0.0)),
                    Plane::from_point_normal(Point::new(hx, 0.0, hz), Vector::new(1.0, 0.0, 0.0)),
                ]
            }
            ElementKind::Plate { polygon, polygon_top, .. } => {
                let normal = Self::polygon_normal(polygon);
                let n = polygon.len().min(polygon_top.len());
                let bottom: Vec<Point> = polygon.iter().take(n).map(|p| Point::new(p[0], p[1], p[2])).collect();
                let top: Vec<Point> = polygon_top.iter().take(n).map(|p| Point::new(p[0], p[1], p[2])).collect();
                let (bcx, bcy, bcz) = bottom.iter().fold((0.0, 0.0, 0.0), |(x, y, z), p| (x + p[0], y + p[1], z + p[2]));
                let (tcx, tcy, tcz) = top.iter().fold((0.0, 0.0, 0.0), |(x, y, z), p| (x + p[0], y + p[1], z + p[2]));
                let nf = n as f64;
                // [0]=top plane, [1]=bottom plane, [2+]=side planes (matching wood)
                let mut result = vec![
                    Plane::from_point_normal(Point::new(tcx / nf, tcy / nf, tcz / nf), normal.clone()),
                    Plane::from_point_normal(Point::new(bcx / nf, bcy / nf, bcz / nf), Vector::new(-normal[0], -normal[1], -normal[2])),
                ];
                // Side planes from 3 actual points: cross(top[i]-top[j], bot[j]-top[j])
                for i in 0..n {
                    let j = (i + 1) % n;
                    let ax = top[i][0] - top[j][0];
                    let ay = top[i][1] - top[j][1];
                    let az = top[i][2] - top[j][2];
                    let bx = bottom[j][0] - top[j][0];
                    let by = bottom[j][1] - top[j][1];
                    let bz = bottom[j][2] - top[j][2];
                    let nx = ay * bz - az * by;
                    let ny = az * bx - ax * bz;
                    let nz = ax * by - ay * bx;
                    let mag = (nx * nx + ny * ny + nz * nz).sqrt();
                    let sn = if mag > 1e-12 {
                        Vector::new(nx / mag, ny / mag, nz / mag)
                    } else {
                        Vector::new(nx, ny, nz)
                    };
                    let cx = (top[i][0] + top[j][0] + bottom[j][0] + bottom[i][0]) * 0.25;
                    let cy = (top[i][1] + top[j][1] + bottom[j][1] + bottom[i][1]) * 0.25;
                    let cz = (top[i][2] + top[j][2] + bottom[j][2] + bottom[i][2]) * 0.25;
                    result.push(Plane::from_point_normal(Point::new(cx, cy, cz), sn));
                }
                result
            }
            _ => Vec::new(),
        }
    }

    fn compute_edge_vectors(&self) -> Vec<Vector> {
        match &self.kind {
            ElementKind::Column { .. } | ElementKind::Beam { .. } => {
                vec![
                    Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0), Vector::new(-1.0, 0.0, 0.0), Vector::new(0.0, -1.0, 0.0),
                    Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0), Vector::new(-1.0, 0.0, 0.0), Vector::new(0.0, -1.0, 0.0),
                    Vector::new(0.0, 0.0, 1.0), Vector::new(0.0, 0.0, 1.0), Vector::new(0.0, 0.0, 1.0), Vector::new(0.0, 0.0, 1.0),
                ]
            }
            ElementKind::Plate { polygon, .. } => {
                let n = polygon.len();
                let mut result = Vec::with_capacity(n);
                for i in 0..n {
                    let j = (i + 1) % n;
                    let dx = polygon[j][0] - polygon[i][0];
                    let dy = polygon[j][1] - polygon[i][1];
                    let dz = polygon[j][2] - polygon[i][2];
                    let mag = (dx * dx + dy * dy + dz * dz).sqrt();
                    if mag > 1e-12 { result.push(Vector::new(dx / mag, dy / mag, dz / mag)); }
                    else { result.push(Vector::new(0.0, 0.0, 0.0)); }
                }
                result
            }
            _ => Vec::new(),
        }
    }

    fn compute_axis(&self) -> Option<Line> {
        match &self.kind {
            ElementKind::Column { height, .. } => Some(Line::new(0.0, 0.0, 0.0, 0.0, 0.0, *height)),
            ElementKind::Beam { length, .. } => Some(Line::new(0.0, 0.0, 0.0, 0.0, 0.0, *length)),
            ElementKind::Plate { polygon, thickness, .. } => {
                let normal = Self::polygon_normal(polygon);
                let n = polygon.len() as f64;
                let cx = polygon.iter().map(|p| p[0]).sum::<f64>() / n;
                let cy = polygon.iter().map(|p| p[1]).sum::<f64>() / n;
                let cz = polygon.iter().map(|p| p[2]).sum::<f64>() / n;
                Some(Line::new(cx, cy, cz, cx - normal[0] * thickness, cy - normal[1] * thickness, cz - normal[2] * thickness))
            }
            _ => None,
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn type_name(&self) -> &str {
        match &self.kind {
            ElementKind::Generic => "Element",
            ElementKind::Column { .. } => "ElementColumn",
            ElementKind::Beam { .. } => "ElementBeam",
            ElementKind::Plate { .. } => "ElementPlate",
        }
    }

    pub fn str(&self) -> String {
        match &self.kind {
            ElementKind::Generic => format!("Element({}, {})", self.name, self.geometry_type_name()),
            ElementKind::Column { width, depth, height } =>
                format!("ElementColumn({}, {}, {}, {})", self.name, width, depth, height),
            ElementKind::Beam { width, depth, length } =>
                format!("ElementBeam({}, {}, {}, {})", self.name, width, depth, length),
            ElementKind::Plate { polygon, thickness, .. } =>
                format!("ElementPlate({}, {} pts, {})", self.name, polygon.len(), thickness),
        }
    }

    pub fn repr(&self) -> String {
        match &self.kind {
            ElementKind::Generic => format!("Element({}, {}, {})", self.guid(), self.name, self.geometry_type_name()),
            ElementKind::Column { width, depth, height } =>
                format!("ElementColumn({}, {}, {}, {}, {})", self.guid(), self.name, width, depth, height),
            ElementKind::Beam { width, depth, length } =>
                format!("ElementBeam({}, {}, {}, {}, {})", self.guid(), self.name, width, depth, length),
            ElementKind::Plate { polygon, thickness, .. } =>
                format!("ElementPlate({}, {}, {} pts, {})", self.guid(), self.name, polygon.len(), thickness),
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn jsondump(&self) -> serde_json::Value {
        match &self.kind {
            ElementKind::Generic => {
                let (geo_data, geo_type) = match &self.geometry {
                    ElementGeometry::Mesh(m) => (serde_json::to_value(m).unwrap_or(serde_json::Value::Null), "Mesh"),
                    ElementGeometry::BRep(b) => (serde_json::to_value(b).unwrap_or(serde_json::Value::Null), "BRep"),
                    ElementGeometry::None => (serde_json::Value::Null, "None"),
                };
                serde_json::json!({
                    "geometry_data": geo_data,
                    "geometry_type": geo_type,
                    "guid": self.guid(),
                    "name": self.name,
                    "type": "Element",
                })
            }
            ElementKind::Column { width, depth, height } => {
                let geo_data = match &self.geometry {
                    ElementGeometry::Mesh(m) => serde_json::to_value(m).unwrap_or(serde_json::Value::Null),
                    _ => serde_json::Value::Null,
                };
                let geo_type = if matches!(self.geometry, ElementGeometry::Mesh(_)) { "Mesh" } else { "None" };
                serde_json::json!({
                    "depth": depth,
                    "geometry_data": geo_data,
                    "geometry_type": geo_type,
                    "guid": self.guid(),
                    "height": height,
                    "name": self.name,
                    "type": "ElementColumn",
                    "width": width,
                })
            }
            ElementKind::Beam { width, depth, length } => {
                let geo_data = match &self.geometry {
                    ElementGeometry::Mesh(m) => serde_json::to_value(m).unwrap_or(serde_json::Value::Null),
                    _ => serde_json::Value::Null,
                };
                let geo_type = if matches!(self.geometry, ElementGeometry::Mesh(_)) { "Mesh" } else { "None" };
                serde_json::json!({
                    "depth": depth,
                    "geometry_data": geo_data,
                    "geometry_type": geo_type,
                    "guid": self.guid(),
                    "length": length,
                    "name": self.name,
                    "type": "ElementBeam",
                    "width": width,
                })
            }
            ElementKind::Plate { polygon, polygon_top, thickness, joint_types, j_mf, key, component_plane } => {
                let geo_data = match &self.geometry {
                    ElementGeometry::Mesh(m) => serde_json::to_value(m).unwrap_or(serde_json::Value::Null),
                    _ => serde_json::Value::Null,
                };
                let geo_type = if matches!(self.geometry, ElementGeometry::Mesh(_)) { "Mesh" } else { "None" };
                let poly_json: Vec<[f64; 3]> = polygon.iter().map(|p| [p[0], p[1], p[2]]).collect();
                let poly_top_json: Vec<[f64; 3]> = polygon_top.iter().map(|p| [p[0], p[1], p[2]]).collect();
                let cp_json = component_plane.as_ref().map(|p| serde_json::to_value(p).unwrap_or(serde_json::Value::Null));
                let j_mf_json: Vec<Vec<(i32, bool, f64)>> = j_mf.clone();
                serde_json::json!({
                    "component_plane": cp_json,
                    "geometry_data": geo_data,
                    "geometry_type": geo_type,
                    "guid": self.guid(),
                    "j_mf": j_mf_json,
                    "joint_types": joint_types,
                    "key": key,
                    "name": self.name,
                    "polygon": poly_json,
                    "polygon_top": poly_top_json,
                    "thickness": thickness,
                    "type": "ElementPlate",
                })
            }
        }
    }

    pub fn jsonload(json_str: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let data: serde_json::Value = serde_json::from_str(json_str)?;
        Ok(Self::jsonload_value(&data))
    }

    pub fn jsonload_value(data: &serde_json::Value) -> Self {
        let type_name = data["type"].as_str().unwrap_or("Element");
        match type_name {
            "ElementColumn" => {
                let w = data["width"].as_f64().unwrap_or(0.4) as f64;
                let d = data["depth"].as_f64().unwrap_or(0.4) as f64;
                let h = data["height"].as_f64().unwrap_or(3.0) as f64;
                let mut elem = Self::column(w, d, h, "my_column");
                if let Some(g) = data["guid"].as_str() { elem.set_guid(g.to_string()); }
                elem.name = data["name"].as_str().unwrap_or(&elem.name).to_string();
                elem
            }
            "ElementBeam" => {
                let w = data["width"].as_f64().unwrap_or(0.1) as f64;
                let d = data["depth"].as_f64().unwrap_or(0.2) as f64;
                let l = data["length"].as_f64().unwrap_or(3.0) as f64;
                let mut elem = Self::beam(w, d, l, "my_beam");
                if let Some(g) = data["guid"].as_str() { elem.set_guid(g.to_string()); }
                elem.name = data["name"].as_str().unwrap_or(&elem.name).to_string();
                elem
            }
            "ElementPlate" => {
                let mut polygon = Vec::new();
                if let Some(arr) = data["polygon"].as_array() {
                    for p in arr {
                        if let Some(coords) = p.as_array() {
                            if coords.len() >= 3 {
                                polygon.push(Point::new(
                                    coords[0].as_f64().unwrap_or(0.0) as f64,
                                    coords[1].as_f64().unwrap_or(0.0) as f64,
                                    coords[2].as_f64().unwrap_or(0.0) as f64,
                                ));
                            }
                        }
                    }
                }
                let mut polygon_top = Vec::new();
                if let Some(arr) = data["polygon_top"].as_array() {
                    for p in arr {
                        if let Some(coords) = p.as_array() {
                            if coords.len() >= 3 {
                                polygon_top.push(Point::new(
                                    coords[0].as_f64().unwrap_or(0.0) as f64,
                                    coords[1].as_f64().unwrap_or(0.0) as f64,
                                    coords[2].as_f64().unwrap_or(0.0) as f64,
                                ));
                            }
                        }
                    }
                }
                let thickness = data["thickness"].as_f64().unwrap_or(0.1) as f64;
                let mut elem = if polygon.is_empty() {
                    Self::plate_default()
                } else if !polygon_top.is_empty() {
                    Self::plate_from_top_bottom(polygon, polygon_top, "my_plate")
                } else {
                    Self::plate(polygon, thickness, "my_plate")
                };
                if let Some(g) = data["guid"].as_str() { elem.set_guid(g.to_string()); }
                elem.name = data["name"].as_str().unwrap_or(&elem.name).to_string();
                if let Some(jt) = data.get("joint_types") {
                    if let Some(arr) = jt.as_array() {
                        let types: Vec<i32> = arr.iter().filter_map(|v| v.as_i64().map(|i| i as i32)).collect();
                        elem.set_joint_types(types);
                    }
                }
                if let Some(jm) = data.get("j_mf") {
                    if let Some(arr) = jm.as_array() {
                        let mut result = Vec::new();
                        for face in arr {
                            if let Some(face_arr) = face.as_array() {
                                let mut fv = Vec::new();
                                for jc in face_arr {
                                    if let Some(jc_arr) = jc.as_array() {
                                        if jc_arr.len() >= 3 {
                                            fv.push((
                                                jc_arr[0].as_i64().unwrap_or(0) as i32,
                                                jc_arr[1].as_bool().unwrap_or(false),
                                                jc_arr[2].as_f64().unwrap_or(0.0) as f64,
                                            ));
                                        }
                                    }
                                }
                                result.push(fv);
                            }
                        }
                        elem.set_j_mf(result);
                    }
                }
                if let Some(k) = data.get("key") {
                    if let Some(s) = k.as_str() { elem.set_key(s.to_string()); }
                }
                if let Some(cp) = data.get("component_plane") {
                    if !cp.is_null() {
                        if let Ok(plane) = serde_json::from_value::<Plane>(cp.clone()) {
                            elem.set_component_plane(plane);
                        }
                    }
                }
                elem
            }
            _ => {
                let geo_type = data["geometry_type"].as_str().unwrap_or("None");
                let mut elem = Self::new("my_element");
                if geo_type == "Mesh" && !data["geometry_data"].is_null() {
                    if let Ok(mesh) = serde_json::from_value::<Mesh>(data["geometry_data"].clone()) {
                        elem.geometry = ElementGeometry::Mesh(mesh);
                    }
                } else if geo_type == "BRep" && !data["geometry_data"].is_null() {
                    if let Ok(brep) = serde_json::from_value::<BRep>(data["geometry_data"].clone()) {
                        elem.geometry = ElementGeometry::BRep(brep);
                    }
                }
                if let Some(g) = data["guid"].as_str() { elem.set_guid(g.to_string()); }
                elem.name = data["name"].as_str().unwrap_or(&elem.name).to_string();
                elem
            }
        }
    }

    pub fn file_json_dumps(&self) -> String {
        let sorted = crate::file_encoders::sort_json_keys(self.jsondump());
        serde_json::to_string(&sorted).unwrap_or_default()
    }

    pub fn file_json_loads(s: &str) -> Self {
        Self::jsonload(s).unwrap_or_else(|_| Self::new("my_element"))
    }

    pub fn file_json_dump(&self, filepath: &str) {
        let sorted = crate::file_encoders::sort_json_keys(self.jsondump());
        let json = serde_json::to_string_pretty(&sorted).unwrap_or_default();
        fs::write(filepath, json).expect("Failed to write JSON file");
    }

    pub fn file_json_load(filepath: &str) -> Self {
        let json = fs::read_to_string(filepath).expect("Failed to read JSON file");
        Self::file_json_loads(&json)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        self.to_proto().encode_to_vec()
    }

    /// The proto struct itself — pb_dumps encodes it; Session embeds it directly.
    pub fn to_proto(&self) -> crate::proto::Element {
        let mut proto = crate::proto::Element::default();
        proto.guid = self.guid().to_string();
        proto.name = self.name.clone();

        match &self.kind {
            ElementKind::Generic => {
                match &self.geometry {
                    ElementGeometry::Mesh(m) => {
                        proto.geometry_type = "Mesh".to_string();
                        proto.geometry_data = m.pb_dumps();
                    }
                    ElementGeometry::BRep(b) => {
                        proto.geometry_type = "BRep".to_string();
                        proto.geometry_data = b.pb_dumps();
                    }
                    ElementGeometry::None => {
                        proto.geometry_type = "None".to_string();
                    }
                }
            }
            ElementKind::Column { width, depth, height } => {
                proto.geometry_type = "ElementColumn".to_string();
                let params = serde_json::json!({"width": width, "depth": depth, "height": height});
                proto.geometry_data = params.to_string().into_bytes();
            }
            ElementKind::Beam { width, depth, length } => {
                proto.geometry_type = "ElementBeam".to_string();
                let params = serde_json::json!({"width": width, "depth": depth, "length": length});
                proto.geometry_data = params.to_string().into_bytes();
            }
            ElementKind::Plate { polygon, polygon_top, thickness, .. } => {
                proto.geometry_type = "ElementPlate".to_string();
                let poly_json: Vec<[f64; 3]> = polygon.iter().map(|p| [p[0], p[1], p[2]]).collect();
                let poly_top_json: Vec<[f64; 3]> = polygon_top.iter().map(|p| [p[0], p[1], p[2]]).collect();
                let params = serde_json::json!({"polygon": poly_json, "polygon_top": poly_top_json, "thickness": thickness});
                proto.geometry_data = params.to_string().into_bytes();
            }
        }

        if let ElementKind::Plate { joint_types, j_mf, key: k, component_plane, .. } = &self.kind {
            proto.joint_types = joint_types.clone();
            proto.j_mf = j_mf.iter().map(|face| {
                crate::proto::FaceJoints {
                    connections: face.iter().map(|(jid, is_male, param)| {
                        crate::proto::JointConnection {
                            joint_id: *jid,
                            is_male: *is_male,
                            parameter: *param as f64,
                        }
                    }).collect(),
                }
            }).collect();
            proto.key = k.clone();
            if let Some(cp) = component_plane {
                proto.component_plane = Some(crate::proto::Plane {
                    guid: String::new(),
                    name: cp.name.clone(),
                    frame: vec![
                        cp.origin()[0] as f64, cp.origin()[1] as f64, cp.origin()[2] as f64,
                        cp.x_axis()[0] as f64, cp.x_axis()[1] as f64, cp.x_axis()[2] as f64,
                        cp.y_axis()[0] as f64, cp.y_axis()[1] as f64, cp.y_axis()[2] as f64,
                        cp.z_axis()[0] as f64, cp.z_axis()[1] as f64, cp.z_axis()[2] as f64,
                    ],
                    width: 0.0,
                    linecolor: None,
                });
            }
        }

        proto
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        Self::from_proto(crate::proto::Element::decode(data)?)
    }

    /// Build from an already-decoded proto — pb_loads decodes then calls this.
    pub fn from_proto(proto: crate::proto::Element) -> Result<Self, Box<dyn std::error::Error>> {
        let geo_type = &proto.geometry_type;

        let mut elem = match geo_type.as_str() {
            "ElementColumn" => {
                let params: serde_json::Value = serde_json::from_slice(&proto.geometry_data)?;
                Self::column(
                    params["width"].as_f64().unwrap() as f64,
                    params["depth"].as_f64().unwrap() as f64,
                    params["height"].as_f64().unwrap() as f64,
                    "my_column",
                )
            }
            "ElementBeam" => {
                let params: serde_json::Value = serde_json::from_slice(&proto.geometry_data)?;
                Self::beam(
                    params["width"].as_f64().unwrap() as f64,
                    params["depth"].as_f64().unwrap() as f64,
                    params["length"].as_f64().unwrap() as f64,
                    "my_beam",
                )
            }
            "ElementPlate" => {
                let params: serde_json::Value = serde_json::from_slice(&proto.geometry_data)?;
                let mut polygon = Vec::new();
                if let Some(arr) = params["polygon"].as_array() {
                    for p in arr {
                        if let Some(coords) = p.as_array() {
                            polygon.push(Point::new(
                                coords[0].as_f64().unwrap_or(0.0) as f64,
                                coords[1].as_f64().unwrap_or(0.0) as f64,
                                coords[2].as_f64().unwrap_or(0.0) as f64,
                            ));
                        }
                    }
                }
                let mut polygon_top = Vec::new();
                if let Some(arr) = params["polygon_top"].as_array() {
                    for p in arr {
                        if let Some(coords) = p.as_array() {
                            polygon_top.push(Point::new(
                                coords[0].as_f64().unwrap_or(0.0) as f64,
                                coords[1].as_f64().unwrap_or(0.0) as f64,
                                coords[2].as_f64().unwrap_or(0.0) as f64,
                            ));
                        }
                    }
                }
                if !polygon_top.is_empty() {
                    Self::plate_from_top_bottom(polygon, polygon_top, "my_plate")
                } else {
                    Self::plate(polygon, params["thickness"].as_f64().unwrap() as f64, "my_plate")
                }
            }
            "Mesh" => {
                let mut e = Self::new("my_element");
                if !proto.geometry_data.is_empty() {
                    e.geometry = ElementGeometry::Mesh(Mesh::pb_loads(&proto.geometry_data)?);
                }
                e
            }
            "BRep" => {
                let mut e = Self::new("my_element");
                if !proto.geometry_data.is_empty() {
                    e.geometry = ElementGeometry::BRep(BRep::pb_loads(&proto.geometry_data)?);
                }
                e
            }
            _ => Self::new("my_element"),
        };

        elem.set_guid(proto.guid.clone());
        elem.name = proto.name.clone();
        if !proto.joint_types.is_empty() {
            elem.set_joint_types(proto.joint_types.clone());
        }
        if !proto.j_mf.is_empty() {
            let j_mf: Vec<Vec<(i32, bool, f64)>> = proto.j_mf.iter().map(|fj| {
                fj.connections.iter().map(|c| (c.joint_id, c.is_male, c.parameter as f64)).collect()
            }).collect();
            elem.set_j_mf(j_mf);
        }
        if !proto.key.is_empty() {
            elem.set_key(proto.key.clone());
        }
        if let Some(cp) = &proto.component_plane {
            if cp.frame.len() == 12 {
                let plane = Plane::new(
                    Point::new(cp.frame[0] as f64, cp.frame[1] as f64, cp.frame[2] as f64),
                    Vector::new(cp.frame[3] as f64, cp.frame[4] as f64, cp.frame[5] as f64),
                    Vector::new(cp.frame[6] as f64, cp.frame[7] as f64, cp.frame[8] as f64),
                );
                elem.set_component_plane(plane);
            }
        }
        Ok(elem)
    }

    pub fn pb_dump(&self, filepath: &str) {
        fs::write(filepath, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    pub fn pb_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let data = fs::read(filepath)?;
        Self::pb_loads(&data)
    }
}

impl Serialize for Element {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.jsondump().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Element {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let value = serde_json::Value::deserialize(deserializer)?;
        Ok(Self::jsonload_value(&value))
    }
}

impl PartialEq for Element {
    fn eq(&self, other: &Self) -> bool {
        match (&self.kind, &other.kind) {
            (ElementKind::Generic, ElementKind::Generic) =>
                self.name == other.name && self.geometry_type_name() == other.geometry_type_name(),
            (ElementKind::Column { width: w1, depth: d1, height: h1 },
             ElementKind::Column { width: w2, depth: d2, height: h2 }) =>
                self.name == other.name && w1 == w2 && d1 == d2 && h1 == h2,
            (ElementKind::Beam { width: w1, depth: d1, length: l1 },
             ElementKind::Beam { width: w2, depth: d2, length: l2 }) =>
                self.name == other.name && w1 == w2 && d1 == d2 && l1 == l2,
            (ElementKind::Plate { polygon: p1, thickness: t1, .. },
             ElementKind::Plate { polygon: p2, thickness: t2, .. }) => {
                if self.name != other.name || t1 != t2 || p1.len() != p2.len() { return false; }
                for (a, b) in p1.iter().zip(p2.iter()) {
                    if a[0] != b[0] || a[1] != b[1] || a[2] != b[2] { return false; }
                }
                true
            }
            _ => false,
        }
    }
}

impl fmt::Display for Element {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

#[cfg(test)]
#[path = "element_test.rs"]
mod element_test;
