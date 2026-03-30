use crate::brep::BRep;
use crate::line::Line;
use crate::mesh::Mesh;
use crate::obb::OBB;
use crate::point::Point;
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

#[derive(Debug, Clone)]
pub enum ElementKind {
    Generic,
    Column { width: f64, depth: f64, height: f64 },
    Beam { width: f64, depth: f64, length: f64 },
    Plate { polygon: Vec<Point>, thickness: f64 },
}

#[derive(Debug, Clone)]
pub struct Element {
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub kind: ElementKind,
    pub session_transformation: Xform,
    geometry: ElementGeometry,
    features: Vec<fn(Mesh) -> Mesh>,
    is_dirty: bool,
    cached_aabb: Option<OBB>,
    cached_obb: Option<OBB>,
    cached_collision_mesh: Option<Mesh>,
    cached_point: Option<Point>,
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
            session_transformation: Xform::identity(),
            geometry: ElementGeometry::None,
            features: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
        }
    }

    pub fn from_mesh(geometry: Mesh, name: &str) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            kind: ElementKind::Generic,
            session_transformation: Xform::identity(),
            geometry: ElementGeometry::Mesh(geometry),
            features: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
        }
    }

    pub fn from_brep(geometry: BRep, name: &str) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            kind: ElementKind::Generic,
            session_transformation: Xform::identity(),
            geometry: ElementGeometry::BRep(geometry),
            features: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
        }
    }

    pub fn column(width: f64, depth: f64, height: f64, name: &str) -> Self {
        let kind = ElementKind::Column { width, depth, height };
        let geometry = ElementGeometry::Mesh(Self::compute_box_geometry(width, depth, height));
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            kind,
            session_transformation: Xform::identity(),
            geometry,
            features: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
        }
    }

    pub fn beam(width: f64, depth: f64, length: f64, name: &str) -> Self {
        let kind = ElementKind::Beam { width, depth, length };
        let geometry = ElementGeometry::Mesh(Self::compute_box_geometry(width, depth, length));
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            kind,
            session_transformation: Xform::identity(),
            geometry,
            features: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
        }
    }

    pub fn plate(polygon: Vec<Point>, thickness: f64, name: &str) -> Self {
        let pts: Vec<Point> = polygon.iter().map(|p| Point::new(p[0], p[1], p[2])).collect();
        let geometry = ElementGeometry::Mesh(Self::compute_plate_geometry(&pts, thickness));
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            kind: ElementKind::Plate { polygon: pts, thickness },
            session_transformation: Xform::identity(),
            geometry,
            features: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
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

    pub fn session_geometry(&self) -> ElementGeometry {
        match &self.geometry {
            ElementGeometry::None => ElementGeometry::None,
            ElementGeometry::Mesh(mesh) => {
                let mut geo = mesh.clone();
                for f in &self.features { geo = f(geo); }
                if !self.session_transformation.is_identity() {
                    geo.xform = &self.session_transformation * &geo.xform;
                    geo.transform(None);
                }
                ElementGeometry::Mesh(geo)
            }
            ElementGeometry::BRep(brep) => {
                let mut geo = brep.clone();
                if !self.session_transformation.is_identity() {
                    geo.xform = &self.session_transformation * &geo.xform;
                    geo.transform();
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
        if let ElementKind::Plate { polygon, thickness } = &mut self.kind {
            *thickness = v;
            self.geometry = ElementGeometry::Mesh(Self::compute_plate_geometry(polygon, *thickness));
            self.reset();
        }
    }

    pub fn set_polygon(&mut self, pts: Vec<Point>) {
        if let ElementKind::Plate { polygon, thickness } = &mut self.kind {
            *polygon = pts.iter().map(|p| Point::new(p[0], p[1], p[2])).collect();
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

    pub fn set_geometry(&mut self, geo: Mesh) {
        self.geometry = ElementGeometry::Mesh(geo);
        self.is_dirty = true;
    }

    pub fn set_brep_geometry(&mut self, geo: BRep) {
        self.geometry = ElementGeometry::BRep(geo);
        self.is_dirty = true;
    }

    pub fn reset(&mut self) {
        self.is_dirty = true;
        self.cached_aabb = None;
        self.cached_obb = None;
        self.cached_collision_mesh = None;
        self.cached_point = None;
    }

    pub fn features_count(&self) -> usize { self.features.len() }
    pub fn cached_aabb_ref(&self) -> &Option<OBB> { &self.cached_aabb }
    pub fn cached_obb_ref(&self) -> &Option<OBB> { &self.cached_obb }
    pub fn cached_collision_mesh_ref(&self) -> &Option<Mesh> { &self.cached_collision_mesh }
    pub fn cached_point_ref(&self) -> &Option<Point> { &self.cached_point }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Computation
    ///////////////////////////////////////////////////////////////////////////////////////////

    fn compute_aabb(&self) -> OBB {
        let geo = self.session_geometry();
        if matches!(geo, ElementGeometry::None) {
            return OBB::from_point(Point::new(0.0, 0.0, 0.0), 0.0);
        }
        Self::obb_from_geometry(&geo)
    }

    fn compute_obb(&self) -> OBB {
        let geo = self.session_geometry();
        if matches!(geo, ElementGeometry::None) {
            return OBB::from_point(Point::new(0.0, 0.0, 0.0), 0.0);
        }
        Self::obb_from_geometry(&geo)
    }

    fn compute_collision_mesh(&self) -> Mesh {
        if let ElementGeometry::Mesh(mesh) = self.session_geometry() { return mesh; }
        Mesh::new()
    }

    fn compute_point(&self) -> Point {
        let geo = self.session_geometry();
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

    fn polygon_normal(pts: &[Point]) -> Vector {
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

    fn compute_plate_geometry(polygon: &[Point], thickness: f64) -> Mesh {
        let normal = Self::polygon_normal(polygon);
        let n = polygon.len();
        let mut vertices = Vec::with_capacity(n * 2);
        for p in polygon {
            vertices.push(Point::new(p[0], p[1], p[2]));
        }
        for p in polygon {
            vertices.push(Point::new(
                p[0] - normal[0] * thickness,
                p[1] - normal[1] * thickness,
                p[2] - normal[2] * thickness,
            ));
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

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn type_name(&self) -> &str {
        match &self.kind {
            ElementKind::Generic => "Element",
            ElementKind::Column { .. } => "ColumnElement",
            ElementKind::Beam { .. } => "BeamElement",
            ElementKind::Plate { .. } => "PlateElement",
        }
    }

    pub fn str(&self) -> String {
        match &self.kind {
            ElementKind::Generic => format!("Element({}, {})", self.name, self.geometry_type_name()),
            ElementKind::Column { width, depth, height } =>
                format!("ColumnElement({}, {}, {}, {})", self.name, width, depth, height),
            ElementKind::Beam { width, depth, length } =>
                format!("BeamElement({}, {}, {}, {})", self.name, width, depth, length),
            ElementKind::Plate { polygon, thickness } =>
                format!("PlateElement({}, {} pts, {})", self.name, polygon.len(), thickness),
        }
    }

    pub fn repr(&self) -> String {
        match &self.kind {
            ElementKind::Generic => format!("Element({}, {}, {})", self.guid(), self.name, self.geometry_type_name()),
            ElementKind::Column { width, depth, height } =>
                format!("ColumnElement({}, {}, {}, {}, {})", self.guid(), self.name, width, depth, height),
            ElementKind::Beam { width, depth, length } =>
                format!("BeamElement({}, {}, {}, {}, {})", self.guid(), self.name, width, depth, length),
            ElementKind::Plate { polygon, thickness } =>
                format!("PlateElement({}, {}, {} pts, {})", self.guid(), self.name, polygon.len(), thickness),
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
                    "session_transformation": serde_json::to_value(&self.session_transformation).unwrap(),
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
                    "session_transformation": serde_json::to_value(&self.session_transformation).unwrap(),
                    "type": "ColumnElement",
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
                    "session_transformation": serde_json::to_value(&self.session_transformation).unwrap(),
                    "type": "BeamElement",
                    "width": width,
                })
            }
            ElementKind::Plate { polygon, thickness } => {
                let geo_data = match &self.geometry {
                    ElementGeometry::Mesh(m) => serde_json::to_value(m).unwrap_or(serde_json::Value::Null),
                    _ => serde_json::Value::Null,
                };
                let geo_type = if matches!(self.geometry, ElementGeometry::Mesh(_)) { "Mesh" } else { "None" };
                let poly_json: Vec<[f64; 3]> = polygon.iter().map(|p| [p[0], p[1], p[2]]).collect();
                serde_json::json!({
                    "geometry_data": geo_data,
                    "geometry_type": geo_type,
                    "guid": self.guid(),
                    "name": self.name,
                    "polygon": poly_json,
                    "session_transformation": serde_json::to_value(&self.session_transformation).unwrap(),
                    "thickness": thickness,
                    "type": "PlateElement",
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
            "ColumnElement" => {
                let w = data["width"].as_f64().unwrap_or(0.4);
                let d = data["depth"].as_f64().unwrap_or(0.4);
                let h = data["height"].as_f64().unwrap_or(3.0);
                let mut elem = Self::column(w, d, h, "my_column");
                if let Some(g) = data["guid"].as_str() { elem.set_guid(g.to_string()); }
                elem.name = data["name"].as_str().unwrap_or(&elem.name).to_string();
                if let Some(xf_data) = data.get("session_transformation") {
                    if let Ok(xf) = serde_json::from_value::<Xform>(xf_data.clone()) {
                        elem.session_transformation = xf;
                    }
                }
                elem
            }
            "BeamElement" => {
                let w = data["width"].as_f64().unwrap_or(0.1);
                let d = data["depth"].as_f64().unwrap_or(0.2);
                let l = data["length"].as_f64().unwrap_or(3.0);
                let mut elem = Self::beam(w, d, l, "my_beam");
                if let Some(g) = data["guid"].as_str() { elem.set_guid(g.to_string()); }
                elem.name = data["name"].as_str().unwrap_or(&elem.name).to_string();
                if let Some(xf_data) = data.get("session_transformation") {
                    if let Ok(xf) = serde_json::from_value::<Xform>(xf_data.clone()) {
                        elem.session_transformation = xf;
                    }
                }
                elem
            }
            "PlateElement" => {
                let mut polygon = Vec::new();
                if let Some(arr) = data["polygon"].as_array() {
                    for p in arr {
                        if let Some(coords) = p.as_array() {
                            if coords.len() >= 3 {
                                polygon.push(Point::new(
                                    coords[0].as_f64().unwrap_or(0.0),
                                    coords[1].as_f64().unwrap_or(0.0),
                                    coords[2].as_f64().unwrap_or(0.0),
                                ));
                            }
                        }
                    }
                }
                let thickness = data["thickness"].as_f64().unwrap_or(0.1);
                let mut elem = if polygon.is_empty() {
                    Self::plate_default()
                } else {
                    Self::plate(polygon, thickness, "my_plate")
                };
                if let Some(g) = data["guid"].as_str() { elem.set_guid(g.to_string()); }
                elem.name = data["name"].as_str().unwrap_or(&elem.name).to_string();
                if let Some(xf_data) = data.get("session_transformation") {
                    if let Ok(xf) = serde_json::from_value::<Xform>(xf_data.clone()) {
                        elem.session_transformation = xf;
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
                if let Some(xf_data) = data.get("session_transformation") {
                    if let Ok(xf) = serde_json::from_value::<Xform>(xf_data.clone()) {
                        elem.session_transformation = xf;
                    }
                }
                elem
            }
        }
    }

    pub fn json_dumps(&self) -> String { serde_json::to_string(&self.jsondump()).unwrap_or_default() }

    pub fn json_loads(s: &str) -> Self {
        Self::jsonload(s).unwrap_or_else(|_| Self::new("my_element"))
    }

    pub fn json_dump(&self, filepath: &str) {
        let json = serde_json::to_string_pretty(&self.jsondump()).unwrap_or_default();
        fs::write(filepath, json).expect("Failed to write JSON file");
    }

    pub fn json_load(filepath: &str) -> Self {
        let json = fs::read_to_string(filepath).expect("Failed to read JSON file");
        Self::json_loads(&json)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
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
                proto.geometry_type = "ColumnElement".to_string();
                let params = serde_json::json!({"width": width, "depth": depth, "height": height});
                proto.geometry_data = params.to_string().into_bytes();
            }
            ElementKind::Beam { width, depth, length } => {
                proto.geometry_type = "BeamElement".to_string();
                let params = serde_json::json!({"width": width, "depth": depth, "length": length});
                proto.geometry_data = params.to_string().into_bytes();
            }
            ElementKind::Plate { polygon, thickness } => {
                proto.geometry_type = "PlateElement".to_string();
                let poly_json: Vec<[f64; 3]> = polygon.iter().map(|p| [p[0], p[1], p[2]]).collect();
                let params = serde_json::json!({"polygon": poly_json, "thickness": thickness});
                proto.geometry_data = params.to_string().into_bytes();
            }
        }

        let mut xf_proto = crate::proto::Xform::default();
        xf_proto.name = self.session_transformation.name.clone();
        xf_proto.matrix = self.session_transformation.m.to_vec();
        proto.session_transformation = Some(xf_proto);
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Element::decode(data)?;
        let geo_type = &proto.geometry_type;

        let mut elem = match geo_type.as_str() {
            "ColumnElement" => {
                let params: serde_json::Value = serde_json::from_slice(&proto.geometry_data)?;
                Self::column(
                    params["width"].as_f64().unwrap(),
                    params["depth"].as_f64().unwrap(),
                    params["height"].as_f64().unwrap(),
                    "my_column",
                )
            }
            "BeamElement" => {
                let params: serde_json::Value = serde_json::from_slice(&proto.geometry_data)?;
                Self::beam(
                    params["width"].as_f64().unwrap(),
                    params["depth"].as_f64().unwrap(),
                    params["length"].as_f64().unwrap(),
                    "my_beam",
                )
            }
            "PlateElement" => {
                let params: serde_json::Value = serde_json::from_slice(&proto.geometry_data)?;
                let mut polygon = Vec::new();
                if let Some(arr) = params["polygon"].as_array() {
                    for p in arr {
                        if let Some(coords) = p.as_array() {
                            polygon.push(Point::new(
                                coords[0].as_f64().unwrap_or(0.0),
                                coords[1].as_f64().unwrap_or(0.0),
                                coords[2].as_f64().unwrap_or(0.0),
                            ));
                        }
                    }
                }
                Self::plate(polygon, params["thickness"].as_f64().unwrap(), "my_plate")
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
        elem.name = proto.name;
        if let Some(xf_proto) = proto.session_transformation {
            let mut xf = Xform::identity();
            xf.name = xf_proto.name;
            if xf_proto.matrix.len() == 16 {
                for i in 0..16 { xf.m[i] = xf_proto.matrix[i]; }
            }
            elem.session_transformation = xf;
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
            (ElementKind::Plate { polygon: p1, thickness: t1 },
             ElementKind::Plate { polygon: p2, thickness: t2 }) => {
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
