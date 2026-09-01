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

/// One modification applied to a host element - a cut, a drill, a joint pocket.
///
/// The serializable half of what [`Element::add_geometry_op`] cannot be: that takes a function
/// pointer, so an operation applied in memory vanishes the moment the Session is written.
/// Domains worked around it by adding flat arrays to Element - a joint type code per face -
/// which is how timber fields ended up in element.proto and had to be reserved out again.
///
/// The kernel does not know how to APPLY one: `feature_type` means something only to the
/// package that wrote it. It knows enough to DRAW one, which is what lets a viewer show
/// features from a package it has never heard of.
// No PartialEq in the derive: Polyline does not implement it.
#[derive(Debug, Clone, Default)]
pub struct ElementFeature {
    /// Lazily minted, like every other identity in the kernel - a feature nobody names never
    /// pays for a guid.
    ///
    /// A feature is addressable in its own right: the package that wrote a joint needs to name
    /// it again later, to update it, to report a clash against it, or to let a viewer select one
    /// of the forty cuts on a beam. The only other handle is the index in `features`, and that
    /// moves the moment an earlier feature is removed.
    guid: std::sync::OnceLock<String>,
    /// Human-readable label.
    pub name: String,
    /// What kind of modification, e.g. "cut", "drill", "joint" - the package's vocabulary.
    pub feature_type: String,
    /// Face of the host this applies to; -1 = the whole element.
    pub face_index: i32,
    /// Geometry of the modification.
    pub outlines: Vec<Polyline>,
}

impl ElementFeature {
    /// Same argument order as the C++ and Python constructors: what the modification IS, where
    /// it applies, its geometry, then the optional label.
    pub fn new(feature_type: &str, face_index: i32, outlines: Vec<Polyline>, name: &str) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            feature_type: feature_type.to_string(),
            face_index,
            outlines,
        }
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }
}

#[derive(Debug, Clone)]
pub struct Element {
    guid: std::sync::OnceLock<String>,
    pub name: String,
    geometry: ElementGeometry,
    /// Callables applied lazily when geometry is computed - NOT serializable. Renamed off
    /// "feature" so the serializable `features` below can own that name.
    geometry_ops: Vec<fn(Mesh) -> Mesh>,
    is_dirty: bool,
    cached_aabb: Option<OBB>,
    cached_obb: Option<OBB>,
    cached_collision_mesh: Option<Mesh>,
    cached_point: Option<Point>,
    cached_polylines: Option<Vec<Polyline>>,
    cached_planes: Option<Vec<Plane>>,
    cached_edge_vectors: Option<Vec<Vector>>,
    cached_axis: Option<Line>,

    // ── Polymorphic elements ────────────────────────────────────────────────────────────
    // Rust has no inheritance, so the C++ factory registry (session_cpp/src/element.cpp) has
    // no direct analogue here - there is no base class to return a derived instance through.
    // What Rust MUST do instead is not destroy the information: a downstream package writes
    // its type name and its own state into these two fields, and anything that loads a
    // Session and writes it back has to hand them on untouched. Dropping them would mean a
    // Rust tool - the viewer, say - silently strips wood's joinery data off every element it
    // round-trips, while the geometry still looks right.
    //
    // A Rust consumer that wants the domain type matches on `element_type` and decodes
    // `element_data` itself; that dispatch is the package's business, not the kernel's.
    /// Class name of the derived element, empty for a plain Element. See element.proto.
    pub element_type: String,
    /// The derived type's own state, opaque here. Empty for a plain Element.
    pub element_data: Vec<u8>,

    /// Modifications carried BY this element, and written with it.
    pub features: Vec<ElementFeature>,

    /// Direction(s) the element is inserted along when the assembly is put together. General
    /// to any assembly: it is what an assembly sequence is ordered by.
    pub insertion_vectors: Vec<Vector>,

    /// NOMINAL extents in this element's own frame - authored intent, NOT a measurement.
    /// Plate: x/y outline extent, z thickness. Beam: x/y cross-section, z length.
    ///
    /// Deliberately distinct from [`Element::obb`], which MEASURES the geometry that exists.
    /// The two are allowed to disagree: a thickness drives a loft before there is any geometry
    /// to measure. `None` = never authored, which `(0,0,0)` does not mean.
    pub dimensions: Option<Vector>,
}

impl Element {
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn new(name: &str) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            geometry: ElementGeometry::None,
            geometry_ops: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
            cached_polylines: None,
            cached_planes: None,
            cached_edge_vectors: None,
            cached_axis: None,
            element_type: String::new(),
            element_data: Vec::new(),
            features: Vec::new(),
            insertion_vectors: Vec::new(),
            dimensions: None,
        }
    }


    pub fn from_mesh(geometry: Mesh, name: &str) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            geometry: ElementGeometry::Mesh(geometry),
            geometry_ops: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
            cached_polylines: None,
            cached_planes: None,
            cached_edge_vectors: None,
            cached_axis: None,
            element_type: String::new(),
            element_data: Vec::new(),
            features: Vec::new(),
            insertion_vectors: Vec::new(),
            dimensions: None,
        }
    }


    pub fn from_brep(geometry: BRep, name: &str) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            geometry: ElementGeometry::BRep(geometry),
            geometry_ops: Vec::new(),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
            cached_polylines: None,
            cached_planes: None,
            cached_edge_vectors: None,
            cached_axis: None,
            element_type: String::new(),
            element_data: Vec::new(),
            features: Vec::new(),
            insertion_vectors: Vec::new(),
            dimensions: None,
        }
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
                for f in &self.geometry_ops { geo = f(geo); }
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

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Column/Beam/Plate Setters
    ///////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Mutators
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_geometry_op(&mut self, f: fn(Mesh) -> Mesh) {
        self.geometry_ops.push(f);
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

    pub fn geometry_ops_count(&self) -> usize { self.geometry_ops.len() }
    pub fn features_count(&self) -> usize { self.features.len() }
    pub fn cached_aabb_ref(&self) -> &Option<OBB> { &self.cached_aabb }
    pub fn cached_obb_ref(&self) -> &Option<OBB> { &self.cached_obb }
    pub fn cached_collision_mesh_ref(&self) -> &Option<Mesh> { &self.cached_collision_mesh }
    pub fn cached_point_ref(&self) -> &Option<Point> { &self.cached_point }

    /// Explicitly-set polylines. Nothing derives them any more: they used to be computed from
    /// the Column/Beam/Plate kind, and the Generic arm always returned empty.
    pub fn polylines(&mut self) -> Vec<Polyline> {
        self.cached_polylines.clone().unwrap_or_default()
    }

    /// Explicitly-set planes — see `polylines`.
    pub fn planes(&mut self) -> Vec<Plane> {
        self.cached_planes.clone().unwrap_or_default()
    }

    /// Explicitly-set edge vectors — see `polylines`.
    pub fn edge_vectors(&mut self) -> Vec<Vector> {
        self.cached_edge_vectors.clone().unwrap_or_default()
    }

    /// Explicitly-set axis — see `polylines`.
    pub fn axis(&mut self) -> Option<Line> {
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
        OBB::from_point(Point::new(0.0, 0.0, 0.0), inflate)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn type_name(&self) -> &str {
        "Element"
    }

    pub fn str(&self) -> String {
        format!("Element({}, {})", self.name, self.geometry_type_name())
    }

    pub fn repr(&self) -> String {
        format!("Element({}, {}, {})", self.guid(), self.name, self.geometry_type_name())
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn jsondump(&self) -> serde_json::Value {
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

    pub fn jsonload(json_str: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let data: serde_json::Value = serde_json::from_str(json_str)?;
        Ok(Self::jsonload_value(&data))
    }

    pub fn jsonload_value(data: &serde_json::Value) -> Self {
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
        
        // Both empty for a plain Element, and proto3 does not emit empty scalars - so a base
        // element's bytes are unchanged by this, keeping the golden files valid.
        proto.element_type = self.element_type.clone();
        proto.element_data = self.element_data.clone();

        proto.insertion_vectors = self.insertion_vectors.iter()
            .map(|v| prost::Message::decode(v.pb_dumps().as_slice()).unwrap_or_default())
            .collect();
        proto.dimensions = self.dimensions.as_ref()
            .and_then(|d| prost::Message::decode(d.pb_dumps().as_slice()).ok());
        proto.features = self.features.iter().map(|f| crate::proto::ElementFeature {
            guid: f.guid().to_string(),
            name: f.name.clone(),
            feature_type: f.feature_type.clone(),
            face_index: f.face_index,
            outlines: f.outlines.iter()
                .map(|o| prost::Message::decode(o.pb_dumps().as_slice()).unwrap_or_default())
                .collect(),
        }).collect();

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
        elem.element_type = proto.element_type.clone();
        elem.element_data = proto.element_data.clone();

        for v in &proto.insertion_vectors {
            elem.insertion_vectors.push(Vector::pb_loads(&prost::Message::encode_to_vec(v))?);
        }
        // `Option`, not a zero check: (0,0,0) is a legitimate authored value and must not be
        // confused with "never authored".
        elem.dimensions = match &proto.dimensions {
            Some(d) => Some(Vector::pb_loads(&prost::Message::encode_to_vec(d))?),
            None => None,
        };
        for f in &proto.features {
            let mut outlines = Vec::new();
            for o in &f.outlines {
                outlines.push(Polyline::pb_loads(&prost::Message::encode_to_vec(o))?);
            }
            let feature = ElementFeature {
                guid: std::sync::OnceLock::new(),
                name: f.name.clone(),
                feature_type: f.feature_type.clone(),
                face_index: f.face_index,
                outlines,
            };
            // Assigned, not minted: a feature off the wire is the SAME feature the package
            // wrote, and anything holding its guid must still find it. Empty means the file
            // predates the field, so the lazy mint is left to whoever asks first.
            if !f.guid.is_empty() {
                feature.set_guid(f.guid.clone());
            }
            elem.features.push(feature);
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
        self.name == other.name && self.geometry_type_name() == other.geometry_type_name()
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
