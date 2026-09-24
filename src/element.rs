use crate::BRep;
use crate::Line;
use crate::Mesh;
use crate::Plane;
use crate::Point;
use crate::Polyline;
use crate::Vector;
use crate::Xform;
use crate::OBB;
use serde::Deserialize;
use serde::Serialize;
use std::cell::Cell;
use std::cell::OnceCell;
use std::collections::BTreeMap;
use std::fmt;
use std::fs;
use std::sync::Mutex;
use std::sync::OnceLock;

/// Kept inline so callers can match on the variant as in C++; boxing would break them.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone)]
pub enum ElementGeometry {
    None,       // No geometry.
    Mesh(Mesh), // Mesh geometry.
    BRep(BRep), // BRep geometry.
}

// ═══════════════════════════════════════════════════════════════════════════
// Hex encoding
// ═══════════════════════════════════════════════════════════════════════════

/// Encode bytes as hex text, since element_data is opaque and JSON carries no bytes.
fn to_hex(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);

    for b in bytes {
        out.push_str(&format!("{:02x}", b));
    }

    out
}

/// Decode hex text back to bytes.
fn from_hex(hex: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(hex.len() / 2);

    for i in (0..hex.len().saturating_sub(1)).step_by(2) {
        if let Ok(b) = u8::from_str_radix(&hex[i..i + 2], 16) {
            out.push(b);
        }
    }

    out
}

// ═══════════════════════════════════════════════════════════════════════════
// ElementFeature
// ═══════════════════════════════════════════════════════════════════════════

/// One serializable modification of a host element - a cut, a drill, a joint pocket - that the kernel draws but never applies.
#[derive(Debug, Clone)]
pub struct ElementFeature {
    guid: OnceLock<String>,      // Lazily minted guid.
    pub name: String,            // Feature name.
    pub feature_type: String,    // The package's vocabulary: "cut", "drill", "joint".
    pub face_index: i32,         // Face of the host this applies to; -1 = whole element.
    pub outlines: Vec<Polyline>, // Closed outlines that bound the feature.
    pub visible: bool,           // Whether a viewer draws it.
}

impl Default for ElementFeature {
    /// Construct an empty feature.
    fn default() -> Self {
        Self::new("", -1, Vec::new(), "")
    }
}

impl ElementFeature {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from type, host face, outlines and name.
    pub fn new(feature_type: &str, face_index: i32, outlines: Vec<Polyline>, name: &str) -> Self {
        Self {
            guid: OnceLock::new(),
            name: name.to_string(),
            feature_type: feature_type.to_string(),
            face_index,
            outlines,
            visible: true,
        }
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
    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Clear the guid so a fresh one mints lazily on the next read.
    pub fn refresh_guid(&mut self) {
        self.guid = OnceLock::new();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    pub fn jsondump(&self) -> serde_json::Value {
        let mut outs = Vec::new();

        for o in &self.outlines {
            outs.push(serde_json::to_value(o).unwrap_or(serde_json::Value::Null));
        }

        serde_json::json!({
            "face_index": self.face_index,
            "feature_type": self.feature_type,
            "guid": self.guid(),
            "name": self.name,
            "outlines": outs,
            "type": "ElementFeature",
            "visible": self.visible,
        })
    }

    /// Deserialize from a JSON object.
    pub fn jsonload(data: &serde_json::Value) -> Self {
        let mut f = Self::new(
            data["feature_type"].as_str().unwrap_or(""),
            data["face_index"].as_i64().unwrap_or(-1) as i32,
            Vec::new(),
            data["name"].as_str().unwrap_or(""),
        );

        let g = data["guid"].as_str().unwrap_or("");

        if !g.is_empty() {
            f.set_guid(g.to_string());
        }

        if let Some(os) = data["outlines"].as_array() {
            for o in os {
                if let Ok(outline) = serde_json::from_value::<Polyline>(o.clone()) {
                    f.outlines.push(outline);
                }
            }
        }

        f.visible = data["visible"].as_bool().unwrap_or(true);

        f
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        let sorted = crate::file_encoders::sort_json_keys(self.jsondump());

        serde_json::to_string(&sorted).unwrap_or_default()
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(s: &str) -> Self {
        let data: serde_json::Value = serde_json::from_str(s).unwrap_or_default();

        Self::jsonload(&data)
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&self, filepath: &str) {
        let sorted = crate::file_encoders::sort_json_keys(self.jsondump());
        let json = serde_json::to_string_pretty(&sorted).unwrap_or_default();
        fs::write(filepath, json).expect("Failed to write JSON file");
    }

    /// Read from a JSON file.
    pub fn file_json_load(filepath: &str) -> Self {
        let json = fs::read_to_string(filepath).expect("Failed to read JSON file");

        Self::file_json_loads(&json)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::ElementFeature {
        let mut outlines = Vec::new();

        for o in &self.outlines {
            outlines.push(o.to_proto());
        }

        crate::proto::ElementFeature {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            feature_type: self.feature_type.clone(),
            face_index: self.face_index,
            outlines,
            visible: (!self.visible).then_some(false),
        }
    }

    /// Construct from the protobuf message.
    pub fn from_proto(proto: crate::proto::ElementFeature) -> Self {
        let mut f = Self::new(
            &proto.feature_type,
            proto.face_index,
            Vec::new(),
            &proto.name,
        );

        if !proto.guid.is_empty() {
            f.set_guid(proto.guid);
        }

        for o in proto.outlines {
            f.outlines.push(Polyline::from_proto(o));
        }

        f.visible = proto.visible.unwrap_or(true);

        f
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        prost::Message::encode_to_vec(&self.to_proto())
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        let proto: crate::proto::ElementFeature = prost::Message::decode(data)?;

        Ok(Self::from_proto(proto))
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&self, filepath: &str) {
        fs::write(filepath, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    /// Read from a protobuf file.
    pub fn pb_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let data = fs::read(filepath)?;

        Self::pb_loads(&data)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a string representation of the feature.
    pub fn str(&self) -> String {
        format!(
            "ElementFeature({}, face {}, {} outline(s))",
            self.feature_type,
            self.face_index,
            self.outlines.len()
        )
    }

    /// Return a string representation of the feature for debugging.
    pub fn repr(&self) -> String {
        self.str()
    }
}

impl PartialEq for ElementFeature {
    /// Compare name, type, face, outlines and visibility; guid ignored.
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && self.feature_type == other.feature_type
            && self.face_index == other.face_index
            && self.outlines == other.outlines
            && self.visible == other.visible
    }
}

impl fmt::Display for ElementFeature {
    /// Write the feature string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

impl Serialize for ElementFeature {
    /// Serialize through jsondump.
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.jsondump().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for ElementFeature {
    /// Deserialize through jsonload.
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let value = serde_json::Value::deserialize(deserializer)?;

        Ok(Self::jsonload(&value))
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Element
// ═══════════════════════════════════════════════════════════════════════════

/// Named geometry carrier with lazily cached boxes, features and a polymorphic type registry.
#[derive(Debug, Clone)]
pub struct Element {
    guid: OnceLock<String>,                   // Lazily minted guid.
    geometry: ElementGeometry,                // Mesh, BRep or nothing.
    model_mesh_cache: OnceCell<Mesh>,         // Model mesh with in-memory operations applied.
    model_brep_cache: OnceCell<BRep>,         // Model BRep, cached independently of the mesh.
    geometry_synced: Cell<bool>, // Whether the slot holds what compute_geometry_mesh() would write.
    computing_geometry: Cell<bool>, // Guards ensure_geometry() against re-entry from compute_geometry_mesh().
    is_dirty: bool,                 // Whether the caches must be recomputed.
    cached_aabb: Option<OBB>,       // Cached axis-aligned box.
    cached_obb: Option<OBB>,        // Cached oriented box.
    cached_collision_mesh: Option<Mesh>, // Cached collision mesh.
    cached_point: Option<Point>,    // Cached centroid.
    cached_polylines: Option<Vec<Polyline>>, // Cached face outlines.
    cached_planes: Option<Vec<Plane>>, // Cached face planes.
    cached_edge_vectors: Option<Vec<Vector>>, // Cached edge directions.
    cached_axis: Option<Line>,      // Cached main axis.
    geometry_ops: Vec<fn(Mesh) -> Mesh>, // In-memory mesh operations, never written.
    // SESSION_VIEWER
    pub features: Vec<ElementFeature>,  // Serialized modifications.
    pub insertion_vectors: Vec<Vector>, // One insertion direction per jointed face.
    pub dimensions: Option<Vector>,     // Authored nominal extents.
    pub element_type: String,           // Derived type name this element was loaded with.
    pub element_data: Vec<u8>,          // Opaque derived-type state.
    pub name: String,                   // Element name.
}

impl Element {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty element with a name.
    pub fn new(name: &str) -> Self {
        Self {
            guid: OnceLock::new(),
            geometry: ElementGeometry::None,
            model_mesh_cache: OnceCell::new(),
            model_brep_cache: OnceCell::new(),
            geometry_synced: Cell::new(false),
            computing_geometry: Cell::new(false),
            is_dirty: true,
            cached_aabb: None,
            cached_obb: None,
            cached_collision_mesh: None,
            cached_point: None,
            cached_polylines: None,
            cached_planes: None,
            cached_edge_vectors: None,
            cached_axis: None,
            geometry_ops: Vec::new(),
            features: Vec::new(),
            insertion_vectors: Vec::new(),
            dimensions: None,
            element_type: String::new(),
            element_data: Vec::new(),
            name: name.to_string(),
        }
    }

    /// Construct from a mesh with a name.
    pub fn from_mesh(geometry: Mesh, name: &str) -> Self {
        let mut e = Self::new(name);
        e.geometry = ElementGeometry::Mesh(geometry);

        e
    }

    /// Construct from a BRep with a name.
    pub fn from_brep(geometry: BRep, name: &str) -> Self {
        let mut e = Self::new(name);
        e.geometry = ElementGeometry::BRep(geometry);

        e
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
    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Clear the guid so a fresh one mints lazily on the next read.
    pub fn refresh_guid(&mut self) {
        self.guid = OnceLock::new();
    }

    /// Return the local geometry, computing it first when a domain type left the slot stale.
    pub fn geometry(&self) -> &ElementGeometry {
        self.ensure_geometry();

        &self.geometry
    }

    /// Return the element's mesh before modifications; empty when no mesh exists.
    pub fn element_geometry_mesh(&self) -> &Mesh {
        self.geometry_mesh()
    }

    /// Return the element's BRep before modifications; empty when no BRep exists.
    pub fn element_geometry_brep(&self) -> &BRep {
        self.geometry_brep()
    }

    /// Return the model mesh with in-memory operations applied, cached until invalidation.
    pub fn model_geometry_mesh(&self) -> &Mesh {
        self.model_mesh_cache
            .get_or_init(|| self.apply_geometry_ops(self.element_geometry_mesh().duplicate()))
    }

    /// Return the model BRep, cached independently until invalidation.
    pub fn model_geometry_brep(&self) -> &BRep {
        self.model_brep_cache
            .get_or_init(|| self.element_geometry_brep().duplicate())
    }

    /// Return the local mesh, computing it on demand; empty when this element has no mesh.
    pub fn geometry_mesh(&self) -> &Mesh {
        static EMPTY: OnceLock<Mesh> = OnceLock::new();

        self.compute_geometry_mesh();

        match &self.geometry {
            ElementGeometry::Mesh(mesh) => mesh,
            _ => EMPTY.get_or_init(Mesh::new),
        }
    }

    /// Return the local BRep, computing it on demand; empty when this element has no BRep.
    pub fn geometry_brep(&self) -> &BRep {
        static EMPTY: OnceLock<BRep> = OnceLock::new();

        self.compute_geometry_brep();

        match &self.geometry {
            ElementGeometry::BRep(brep) => brep,
            _ => EMPTY.get_or_init(BRep::new),
        }
    }

    /// Write the element's mesh, features and dimensions into the session slot, reusing a current mesh.
    pub fn compute_geometry_mesh(&self) {
        if self.computing_geometry.get()
            || (self.geometry_synced.get() && matches!(self.geometry, ElementGeometry::Mesh(_)))
        {
            return;
        }

        self.computing_geometry.set(true);
        self.compute_geometry_mesh_impl();
        self.computing_geometry.set(false);
        self.geometry_synced.set(true);
    }

    /// Write the element's BRep, features and dimensions into the session slot, reusing a current BRep.
    pub fn compute_geometry_brep(&self) {
        if self.computing_geometry.get()
            || (self.geometry_synced.get() && matches!(self.geometry, ElementGeometry::BRep(_)))
        {
            return;
        }

        self.computing_geometry.set(true);
        self.compute_geometry_brep_impl();
        self.computing_geometry.set(false);
        self.geometry_synced.set(true);
    }

    /// Return whether the slot already holds what compute_geometry_mesh() would write.
    pub fn geometry_synced(&self) -> bool {
        self.geometry_synced.get()
    }

    /// Mark the slot stale, so the next read computes it again; a domain type overrides this to drop its own caches too.
    pub fn invalidate_geometry(&mut self) {
        self.geometry_synced.set(false);
        self.model_mesh_cache = OnceCell::new();
        self.model_brep_cache = OnceCell::new();
    }

    /// Return whether the element carries a mesh or a BRep.
    pub fn has_geometry(&self) -> bool {
        self.ensure_geometry();

        !matches!(self.geometry, ElementGeometry::None)
    }

    /// Return "Mesh", "BRep" or "None".
    pub fn geometry_type_name(&self) -> &str {
        self.ensure_geometry();

        match &self.geometry {
            ElementGeometry::None => "None",
            ElementGeometry::Mesh(_) => "Mesh",
            ElementGeometry::BRep(_) => "BRep",
        }
    }

    /// Return the geometry placed by xform; the Session owns the placement, so pass identity for local geometry.
    pub fn session_geometry(&self, xform: &Xform) -> ElementGeometry {
        self.ensure_geometry();

        match &self.geometry {
            ElementGeometry::None => ElementGeometry::None,
            ElementGeometry::Mesh(mesh) => {
                let mut geo = self.apply_geometry_ops(mesh.clone());

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

    /// Return the mesh with in-memory operations and placement applied, empty when no mesh exists.
    pub fn session_geometry_mesh(&self, xform: &Xform) -> Mesh {
        let local = self.geometry_mesh();

        if !matches!(self.geometry, ElementGeometry::Mesh(_)) {
            return Mesh::new();
        }

        let mut placed = self.apply_geometry_ops(local.duplicate());

        if !xform.is_identity() {
            placed.transform(xform);
        }

        placed
    }

    /// Return the BRep with placement applied, empty when no BRep exists.
    pub fn session_geometry_brep(&self, xform: &Xform) -> BRep {
        let local = self.geometry_brep();
        let mut placed = local.duplicate();

        if !xform.is_identity() {
            placed.transform(xform);
        }

        placed
    }

    /// Return the cached axis-aligned box, computing it when dirty.
    pub fn aabb(&mut self) -> OBB {
        if self.is_dirty || self.cached_aabb.is_none() {
            self.cached_aabb = Some(self.compute_aabb());
            self.is_dirty = false;
        }

        self.cached_aabb.clone().unwrap()
    }

    /// Return the cached oriented box, computing it when dirty.
    pub fn obb(&mut self) -> OBB {
        if self.is_dirty || self.cached_obb.is_none() {
            self.cached_obb = Some(self.compute_obb());
            self.is_dirty = false;
        }

        self.cached_obb.clone().unwrap()
    }

    /// Return the cached collision mesh, computing it when dirty.
    pub fn collision_mesh(&mut self) -> Mesh {
        if self.is_dirty || self.cached_collision_mesh.is_none() {
            self.cached_collision_mesh = Some(self.compute_collision_mesh());
            self.is_dirty = false;
        }

        self.cached_collision_mesh.clone().unwrap()
    }

    /// Return the cached centroid, computing it when dirty.
    pub fn point(&mut self) -> Point {
        if self.is_dirty || self.cached_point.is_none() {
            self.cached_point = Some(self.compute_point());
            self.is_dirty = false;
        }

        self.cached_point.clone().unwrap()
    }

    /// Return the cached face outlines, computing them when dirty.
    pub fn polylines(&mut self) -> Vec<Polyline> {
        if self.is_dirty || self.cached_polylines.is_none() {
            self.cached_polylines = Some(self.compute_polylines());
            self.is_dirty = false;
        }

        self.cached_polylines.clone().unwrap()
    }

    /// Return the cached face planes, computing them when dirty.
    pub fn planes(&mut self) -> Vec<Plane> {
        if self.is_dirty || self.cached_planes.is_none() {
            self.cached_planes = Some(self.compute_planes());
            self.is_dirty = false;
        }

        self.cached_planes.clone().unwrap()
    }

    /// Return the cached edge directions, computing them when dirty.
    pub fn edge_vectors(&mut self) -> Vec<Vector> {
        if self.is_dirty || self.cached_edge_vectors.is_none() {
            self.cached_edge_vectors = Some(self.compute_edge_vectors());
            self.is_dirty = false;
        }

        self.cached_edge_vectors.clone().unwrap()
    }

    /// Return the cached main axis, computing it when dirty.
    pub fn axis(&mut self) -> Option<Line> {
        if self.is_dirty || self.cached_axis.is_none() {
            self.cached_axis = self.compute_axis();
            self.is_dirty = false;
        }

        self.cached_axis.clone()
    }

    /// Return whether the caches must be recomputed.
    pub fn is_dirty(&self) -> bool {
        self.is_dirty
    }

    /// Return the cached axis-aligned box without computing it.
    pub fn cached_aabb(&self) -> &Option<OBB> {
        &self.cached_aabb
    }

    /// Return the cached oriented box without computing it.
    pub fn cached_obb(&self) -> &Option<OBB> {
        &self.cached_obb
    }

    /// Return the cached collision mesh without computing it.
    pub fn cached_collision_mesh(&self) -> &Option<Mesh> {
        &self.cached_collision_mesh
    }

    /// Return the cached centroid without computing it.
    pub fn cached_point(&self) -> &Option<Point> {
        &self.cached_point
    }

    /// Return the number of in-memory geometry operations.
    pub fn geometry_ops_count(&self) -> usize {
        self.geometry_ops.len()
    }

    /// Return the number of features.
    pub fn features_count(&self) -> usize {
        self.features.len()
    }

    /// Return the modifications carried by this element and written with it; add_geometry_op is the in-memory counterpart that is not.
    pub fn features(&self) -> &[ElementFeature] {
        self.ensure_geometry();

        &self.features
    }

    /// Return the directions the element is inserted along when the assembly is put together, one per jointed face.
    pub fn insertion_vectors(&self) -> &[Vector] {
        &self.insertion_vectors
    }

    /// Return the nominal extents in the element's own frame (plate: x/y outline, z thickness), authored intent rather than the measured obb; None = never authored.
    pub fn dimensions(&self) -> &Option<Vector> {
        self.ensure_geometry();

        &self.dimensions
    }

    /// Return the derived type name this element was loaded with, written to element_type; a plain Element authored in memory returns "".
    pub fn element_type_name(&self) -> &str {
        &self.element_type
    }

    /// Return the derived type's own state, opaque to the kernel and carried through untouched.
    pub fn element_data_dumps(&self) -> &[u8] {
        &self.element_data
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Append an in-memory mesh operation and invalidate the caches.
    pub fn add_geometry_op(&mut self, f: fn(Mesh) -> Mesh) {
        self.geometry_ops.push(f);
        self.reset();
    }

    /// Replace the features.
    pub fn set_features(&mut self, features: Vec<ElementFeature>) {
        self.features = features;
    }

    /// Append a feature.
    pub fn add_feature(&mut self, feature: ElementFeature) {
        self.features.push(feature);
    }

    /// Replace the insertion vectors.
    pub fn set_insertion_vectors(&mut self, v: Vec<Vector>) {
        self.insertion_vectors = v;
    }

    /// Set the nominal extents.
    pub fn set_dimensions(&mut self, d: Vector) {
        self.dimensions = Some(d);
    }

    /// Bake a placement into the geometry, the feature outlines and the insertion vectors, then drop the caches.
    pub fn place(&mut self, xform: &Xform) {
        self.geometry = self.session_geometry(xform);

        for feature in &mut self.features {
            for outline in &mut feature.outlines {
                outline.transform(xform);
            }
        }

        for direction in &mut self.insertion_vectors {
            direction.transform(xform);
        }

        self.reset();
    }

    /// Replace the geometry with a mesh and invalidate the caches.
    pub fn set_geometry(&mut self, geo: Mesh) {
        self.geometry = ElementGeometry::Mesh(geo);
        self.reset();
    }

    /// Replace the geometry with a BRep and invalidate the caches.
    pub fn set_brep_geometry(&mut self, geo: BRep) {
        self.geometry = ElementGeometry::BRep(geo);
        self.reset();
    }

    /// Replace the geometry with a mesh, a BRep or nothing and invalidate the caches.
    pub fn set_element_geometry(&mut self, geo: ElementGeometry) {
        self.geometry = geo;
        self.reset();
    }

    /// Override the cached face outlines, kept until the next reset.
    pub fn set_polylines(&mut self, polys: Vec<Polyline>) {
        self.cached_polylines = Some(polys);
        self.is_dirty = false;
    }

    /// Override the cached face planes, kept until the next reset.
    pub fn set_planes(&mut self, plns: Vec<Plane>) {
        self.cached_planes = Some(plns);
        self.is_dirty = false;
    }

    /// Drop every cache and mark the element dirty.
    pub fn reset(&mut self) {
        self.is_dirty = true;
        self.model_mesh_cache = OnceCell::new();
        self.model_brep_cache = OnceCell::new();
        self.cached_aabb = None;
        self.cached_obb = None;
        self.cached_collision_mesh = None;
        self.cached_point = None;
        self.cached_polylines = None;
        self.cached_planes = None;
        self.cached_edge_vectors = None;
        self.cached_axis = None;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Utilities
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a copy with a new guid, as does every feature it carries.
    pub fn duplicate(&self) -> Self {
        let mut result = self.clone();
        result.guid = OnceLock::new();

        for f in &mut result.features {
            f.refresh_guid();
        }

        result.reset();

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    pub fn jsondump(&self) -> serde_json::Value {
        self.ensure_geometry();

        let mut geo_data = serde_json::Value::Null;

        if let ElementGeometry::Mesh(mesh) = &self.geometry {
            geo_data = serde_json::to_value(mesh).unwrap_or(serde_json::Value::Null);
        }

        if let ElementGeometry::BRep(brep) = &self.geometry {
            geo_data = serde_json::to_value(brep).unwrap_or(serde_json::Value::Null);
        }

        let mut dims = serde_json::Value::Null;

        if let Some(d) = &self.dimensions {
            dims = serde_json::to_value(d).unwrap_or(serde_json::Value::Null);
        }

        let mut feats = Vec::new();

        for f in &self.features {
            feats.push(f.jsondump());
        }

        let mut ivs = Vec::new();

        for v in &self.insertion_vectors {
            ivs.push(serde_json::to_value(v).unwrap_or(serde_json::Value::Null));
        }

        serde_json::json!({
            "dimensions": dims,
            "element_data": to_hex(&self.element_data),
            "element_type": self.element_type,
            "features": feats,
            "geometry_data": geo_data,
            "geometry_type": self.geometry_type_name(),
            "guid": self.guid(),
            "insertion_vectors": ivs,
            "name": self.name,
            "type": "Element",
        })
    }

    /// Deserialize from a JSON object.
    pub fn jsonload(data: &serde_json::Value) -> Self {
        let mut elem = Self::new("my_element");
        let geo_type = data["geometry_type"].as_str().unwrap_or("None");
        let has_data = !data["geometry_data"].is_null();

        if geo_type == "Mesh" && has_data {
            if let Ok(mesh) = serde_json::from_value::<Mesh>(data["geometry_data"].clone()) {
                elem.geometry = ElementGeometry::Mesh(mesh);
            }
        }

        if geo_type == "BRep" && has_data {
            if let Ok(brep) = serde_json::from_value::<BRep>(data["geometry_data"].clone()) {
                elem.geometry = ElementGeometry::BRep(brep);
            }
        }

        let g = data["guid"].as_str().unwrap_or("");

        if !g.is_empty() {
            elem.set_guid(g.to_string());
        }

        elem.name = data["name"].as_str().unwrap_or(&elem.name).to_string();

        if !data["dimensions"].is_null() {
            elem.dimensions = serde_json::from_value(data["dimensions"].clone()).ok();
        }

        elem.element_type = data["element_type"].as_str().unwrap_or("").to_string();
        elem.element_data = from_hex(data["element_data"].as_str().unwrap_or(""));

        if let Some(fs) = data["features"].as_array() {
            for f in fs {
                elem.features.push(ElementFeature::jsonload(f));
            }
        }

        if let Some(vs) = data["insertion_vectors"].as_array() {
            for v in vs {
                if let Ok(vector) = serde_json::from_value::<Vector>(v.clone()) {
                    elem.insertion_vectors.push(vector);
                }
            }
        }

        elem
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        let sorted = crate::file_encoders::sort_json_keys(self.jsondump());

        serde_json::to_string(&sorted).unwrap_or_default()
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(s: &str) -> Self {
        let data: serde_json::Value = serde_json::from_str(s).unwrap_or_default();

        Self::jsonload(&data)
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&self, filepath: &str) {
        let sorted = crate::file_encoders::sort_json_keys(self.jsondump());
        let json = serde_json::to_string_pretty(&sorted).unwrap_or_default();
        fs::write(filepath, json).expect("Failed to write JSON file");
    }

    /// Read from a JSON file.
    pub fn file_json_load(filepath: &str) -> Self {
        let json = fs::read_to_string(filepath).expect("Failed to read JSON file");

        Self::file_json_loads(&json)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::Element {
        self.ensure_geometry();

        let mut geometry_data = Vec::new();

        if let ElementGeometry::Mesh(mesh) = &self.geometry {
            geometry_data = mesh.pb_dumps();
        }

        if let ElementGeometry::BRep(brep) = &self.geometry {
            geometry_data = brep.pb_dumps();
        }

        let mut insertion_vectors = Vec::new();

        for v in &self.insertion_vectors {
            insertion_vectors.push(v[0]);
            insertion_vectors.push(v[1]);
            insertion_vectors.push(v[2]);
        }

        let mut dimensions = Vec::new();

        if let Some(d) = &self.dimensions {
            dimensions.push(d[0]);
            dimensions.push(d[1]);
            dimensions.push(d[2]);
        }

        let mut features = Vec::new();

        for f in &self.features {
            features.push(f.to_proto());
        }

        crate::proto::Element {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            geometry_type: self.geometry_type_name().to_string(),
            geometry_data,
            element_type: self.element_type.clone(),
            element_data: self.element_data.clone(),
            insertion_vectors,
            dimensions,
            features,
        }
    }

    /// Construct from the protobuf message.
    pub fn from_proto(proto: crate::proto::Element) -> Result<Self, Box<dyn std::error::Error>> {
        let mut elem = Self::new("my_element");

        if !proto.guid.is_empty() {
            elem.set_guid(proto.guid);
        }

        elem.name = proto.name;

        let has_data = !proto.geometry_data.is_empty();

        if proto.geometry_type == "Mesh" && has_data {
            elem.geometry = ElementGeometry::Mesh(Mesh::pb_loads(&proto.geometry_data)?);
        }

        if proto.geometry_type == "BRep" && has_data {
            elem.geometry = ElementGeometry::BRep(BRep::pb_loads(&proto.geometry_data)?);
        }

        elem.element_type = proto.element_type;
        elem.element_data = proto.element_data;

        for c in proto.insertion_vectors.chunks_exact(3) {
            elem.insertion_vectors.push(Vector::new(c[0], c[1], c[2]));
        }

        if proto.dimensions.len() == 3 {
            elem.dimensions = Some(Vector::new(
                proto.dimensions[0],
                proto.dimensions[1],
                proto.dimensions[2],
            ));
        }

        for f in proto.features {
            elem.features.push(ElementFeature::from_proto(f));
        }

        Ok(elem)
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        prost::Message::encode_to_vec(&self.to_proto())
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        let proto: crate::proto::Element = prost::Message::decode(data)?;

        Self::from_proto(proto)
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&self, filepath: &str) {
        fs::write(filepath, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    /// Read from a protobuf file.
    pub fn pb_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let data = fs::read(filepath)?;

        Self::pb_loads(&data)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Polymorphic registry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Register factory for type_name; re-registering the same name replaces it.
    pub fn register_type(type_name: &str, factory: ElementFactory) {
        if type_name.is_empty() {
            return;
        }

        if let Ok(mut registry) = element_registry().lock() {
            registry.insert(type_name.to_string(), factory);
        }
    }

    /// Return whether a factory is registered for type_name.
    pub fn is_registered(type_name: &str) -> bool {
        if let Ok(registry) = element_registry().lock() {
            return registry.contains_key(type_name);
        }

        false
    }

    /// Return the registered type names.
    pub fn registered_types() -> Vec<String> {
        let mut names = Vec::new();

        if let Ok(registry) = element_registry().lock() {
            for name in registry.keys() {
                names.push(name.clone());
            }
        }

        names
    }

    /// Load through the registered factory, degrading to a base Element that still carries element_type and element_data when the type is unknown or the factory declines.
    pub fn pb_loads_polymorphic(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        let proto: crate::proto::Element = prost::Message::decode(data)?;

        if let Some(derived) = build_registered(&proto.element_type, data) {
            return Ok(derived);
        }

        Self::from_proto(proto)
    }

    /// Load from JSON through the registered factory, re-encoded to proto bytes so one registration serves both formats.
    pub fn file_json_loads_polymorphic(s: &str) -> Self {
        let base = Self::file_json_loads(s);

        if let Some(derived) = build_registered(base.element_type_name(), &base.pb_dumps()) {
            return derived;
        }

        base
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a string representation of the element.
    pub fn str(&self) -> String {
        format!("Element({}, {})", self.name, self.geometry_type_name())
    }

    /// Return a string representation of the element for debugging.
    pub fn repr(&self) -> String {
        format!(
            "Element({}, {}, {})",
            self.guid(),
            self.name,
            self.geometry_type_name()
        )
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Computation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Run compute_geometry_mesh() once while the slot is stale, so every reader and the file see the current solid, features and dimensions.
    fn ensure_geometry(&self) {
        if self.geometry_synced.get() || self.computing_geometry.get() {
            return;
        }

        self.compute_geometry_mesh();
    }

    /// Compute the mesh, features and dimensions; the base element has none.
    fn compute_geometry_mesh_impl(&self) {}

    /// Compute the BRep, features and dimensions; the base element has none.
    fn compute_geometry_brep_impl(&self) {}

    /// Compute the axis-aligned box of the placed geometry.
    fn compute_aabb(&self) -> OBB {
        Self::obb_from_geometry(&self.session_geometry(&Xform::identity()))
    }

    /// Compute the oriented box of the placed geometry.
    fn compute_obb(&self) -> OBB {
        Self::obb_from_geometry(&self.session_geometry(&Xform::identity()))
    }

    /// Compute the collision mesh; a BRep yields an empty mesh.
    fn compute_collision_mesh(&self) -> Mesh {
        if let ElementGeometry::Mesh(mesh) = self.session_geometry(&Xform::identity()) {
            return mesh;
        }

        Mesh::new()
    }

    /// Compute the centroid of the placed geometry.
    fn compute_point(&self) -> Point {
        Point::centroid(&Self::points_from_geometry(
            &self.session_geometry(&Xform::identity()),
        ))
    }

    /// Compute the face outlines of a mesh solid; a domain type with its own face order overrides this.
    fn compute_polylines(&self) -> Vec<Polyline> {
        if let ElementGeometry::Mesh(mesh) = &self.geometry {
            return mesh.face_outlines();
        }

        Vec::new()
    }

    /// Compute one plane per face outline: centroid origin, Newell normal, closing point dropped first.
    fn compute_planes(&self) -> Vec<Plane> {
        let mut planes = Vec::new();

        for outline in self.compute_polylines() {
            let mut points = outline.get_points();

            if points.len() > 1 && points[0] == points[points.len() - 1] {
                points.pop();
            }

            if points.len() < 3 {
                continue;
            }

            planes.push(Plane::from_point_normal(
                Point::centroid(&points),
                Vector::average_normal(&points),
                None,
            ));
        }

        planes
    }

    /// Compute the edge directions; the base element has none.
    fn compute_edge_vectors(&self) -> Vec<Vector> {
        Vec::new()
    }

    /// Compute the main axis; the base element has none.
    fn compute_axis(&self) -> Option<Line> {
        None
    }

    /// Run the in-memory operations over a mesh.
    fn apply_geometry_ops(&self, mut geo: Mesh) -> Mesh {
        for f in &self.geometry_ops {
            geo = f(geo);
        }

        geo
    }

    /// Return the vertices of a mesh or a BRep.
    fn points_from_geometry(geo: &ElementGeometry) -> Vec<Point> {
        let mut points = Vec::new();

        if let ElementGeometry::Mesh(mesh) = geo {
            for v in mesh.vertex.values() {
                points.push(v.position());
            }
        }

        if let ElementGeometry::BRep(brep) = geo {
            points = brep.vertex_points();
        }

        points
    }

    /// Return the world-aligned box of the geometry vertices, or a zero box when empty.
    fn obb_from_geometry(geo: &ElementGeometry) -> OBB {
        let points = Self::points_from_geometry(geo);

        if points.is_empty() {
            return OBB::from_point(&Point::new(0.0, 0.0, 0.0), 0.0);
        }

        OBB::from_points(&points, 0.0, None)
    }
}

/// The factory a package registers for its own `element_type`: takes full serialized `session_proto.Element` bytes and returns `None` to decline.
pub type ElementFactory = fn(&[u8]) -> Option<Element>;

type Registry = Mutex<BTreeMap<String, ElementFactory>>;

/// Function-local so a package registering from a static initializer finds it built.
fn element_registry() -> &'static Registry {
    static REGISTRY: OnceLock<Registry> = OnceLock::new();

    REGISTRY.get_or_init(|| Mutex::new(BTreeMap::new()))
}

/// A factory that declines degrades to the base exactly like an unregistered type.
fn build_registered(type_name: &str, data: &[u8]) -> Option<Element> {
    if type_name.is_empty() {
        return None;
    }

    let factory = *element_registry().lock().ok()?.get(type_name)?;

    factory(data)
}

impl PartialEq for Element {
    /// Compare every field that survives a round trip; guid ignored.
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && self.geometry_type_name() == other.geometry_type_name()
            && self.element_type == other.element_type
            && self.element_data == other.element_data
            && self.insertion_vectors == other.insertion_vectors
            && self.dimensions == other.dimensions
            && self.features == other.features
    }
}

impl fmt::Display for Element {
    /// Write the element string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

impl Serialize for Element {
    /// Serialize through jsondump.
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.jsondump().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Element {
    /// Deserialize through jsonload.
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let value = serde_json::Value::deserialize(deserializer)?;

        Ok(Self::jsonload(&value))
    }
}
