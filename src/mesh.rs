use crate::polyline::Polyline;
use crate::remesh_cdt;
use crate::spatial_aabbtree::SpatialAABBTree;
use crate::tolerance::PI;
use crate::{Color, Line, Plane, Point, SpatialBVH, Tolerance, Vector, Xform, AABB, OBB};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

/// Predicate over a key and its merged attributes.
pub type KeyPredicate<'a, K> = &'a dyn Fn(K, &HashMap<String, f64>) -> bool;

/// Miter plate contours of one face: (top_chamfered, bot_chamfered, top_raw, bot_raw, face_normal).
pub type MiterContour = (Vec<Point>, Vec<Point>, Vec<Point>, Vec<Point>, Vector);

/// Dihedral angles per edge, their arcs and their label points.
pub type DihedralAngles = (BTreeMap<(usize, usize), f64>, Vec<Polyline>, Vec<Point>);

/// Faces touching an undirected edge as (face_key, u, v) triples.
type EdgeFaces = HashMap<(usize, usize), Vec<(usize, usize, usize)>>;

/// Which stored colors a mesh renders with.
#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ColorMode {
    #[default]
    OBJECTCOLOR,
    POINTCOLORS,
    FACECOLORS,
    NONE,
}

impl ColorMode {
    /// Return the protobuf enum value.
    fn to_i32(&self) -> i32 {
        match self {
            Self::OBJECTCOLOR => 0,
            Self::POINTCOLORS => 1,
            Self::FACECOLORS => 2,
            Self::NONE => 3,
        }
    }

    /// Return the mode of a protobuf enum value, objectcolor when unknown.
    fn from_i32(v: i32) -> Self {
        match v {
            1 => Self::POINTCOLORS,
            2 => Self::FACECOLORS,
            3 => Self::NONE,
            _ => Self::OBJECTCOLOR,
        }
    }

    /// Return the lowercase name.
    fn to_str(&self) -> &'static str {
        match self {
            Self::OBJECTCOLOR => "objectcolor",
            Self::POINTCOLORS => "pointcolors",
            Self::FACECOLORS => "facecolors",
            Self::NONE => "none",
        }
    }

    /// Return the mode named s, objectcolor when unknown.
    fn from_str(s: &str) -> Self {
        match s {
            "pointcolors" => Self::POINTCOLORS,
            "facecolors" => Self::FACECOLORS,
            "none" => Self::NONE,
            _ => Self::OBJECTCOLOR,
        }
    }
}

/// Weighting scheme for vertex normals.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NormalWeighting {
    Area,
    Angle,
    Uniform,
}

/// A vertex's attribute map, allocated only once something is stored in it.
#[derive(Debug, Clone, Default, PartialEq)]
#[allow(clippy::box_collection)]
pub struct Attributes(Option<Box<BTreeMap<String, f64>>>);

impl Attributes {
    /// Construct an empty map.
    pub fn new() -> Self {
        Self(None)
    }

    /// Return the value named key.
    pub fn get(&self, key: &str) -> Option<&f64> {
        self.0.as_ref().and_then(|m| m.get(key))
    }

    /// Return whether key is stored.
    pub fn contains_key(&self, key: &str) -> bool {
        self.get(key).is_some()
    }

    /// Return the number of entries.
    pub fn len(&self) -> usize {
        self.0.as_ref().map_or(0, |m| m.len())
    }

    /// Return whether nothing is stored.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Free the map.
    pub fn clear(&mut self) {
        self.0 = None
    }

    /// Store a value, allocating the map on first use; the only entry point that can allocate.
    pub fn insert(&mut self, key: String, value: f64) -> Option<f64> {
        self.0
            .get_or_insert_with(Default::default)
            .insert(key, value)
    }

    /// Remove key and free the map when it becomes empty; returns the old value.
    pub fn remove(&mut self, key: &str) -> Option<f64> {
        let out = self.0.as_mut().and_then(|m| m.remove(key));

        if self.is_empty() {
            self.0 = None
        }

        out
    }

    /// Return the map iterator, or a shared empty one.
    pub fn iter(&self) -> std::collections::btree_map::Iter<'_, String, f64> {
        static EMPTY: std::sync::OnceLock<BTreeMap<String, f64>> = std::sync::OnceLock::new();

        match &self.0 {
            Some(m) => m.iter(),
            None => EMPTY.get_or_init(Default::default).iter(),
        }
    }

    /// Iterate the names.
    pub fn keys(&self) -> impl Iterator<Item = &String> {
        self.iter().map(|(k, _)| k)
    }

    /// Iterate the values.
    pub fn values(&self) -> impl Iterator<Item = &f64> {
        self.iter().map(|(_, v)| v)
    }
}

impl<'a> IntoIterator for &'a Attributes {
    type Item = (&'a String, &'a f64);
    type IntoIter = std::collections::btree_map::Iter<'a, String, f64>;

    /// Iterate the entries.
    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl IntoIterator for Attributes {
    type Item = (String, f64);
    type IntoIter = std::collections::btree_map::IntoIter<String, f64>;

    /// Iterate the entries.
    fn into_iter(self) -> Self::IntoIter {
        self.0.map_or_else(Default::default, |m| *m).into_iter()
    }
}

impl FromIterator<(String, f64)> for Attributes {
    /// Collect entries, allocating only when there are any.
    fn from_iter<T: IntoIterator<Item = (String, f64)>>(it: T) -> Self {
        let m: BTreeMap<String, f64> = it.into_iter().collect();

        if m.is_empty() {
            Self(None)
        } else {
            Self(Some(Box::new(m)))
        }
    }
}

impl Extend<(String, f64)> for Attributes {
    /// Store every entry of it.
    fn extend<T: IntoIterator<Item = (String, f64)>>(&mut self, it: T) {
        for (k, v) in it {
            self.insert(k, v);
        }
    }
}

impl Serialize for Attributes {
    /// Serialize as a JSON map.
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeMap;
        let mut m = s.serialize_map(Some(self.len()))?;

        for (k, v) in self.iter() {
            m.serialize_entry(k, v)?;
        }

        m.end()
    }
}

impl<'de> Deserialize<'de> for Attributes {
    /// Deserialize from a JSON map, allocating only when it is not empty.
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let m = BTreeMap::<String, f64>::deserialize(d)?;

        Ok(if m.is_empty() {
            Self(None)
        } else {
            Self(Some(Box::new(m)))
        })
    }
}

/// Vertex position and attributes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VertexData {
    pub x: f64,                 // Position x.
    pub y: f64,                 // Position y.
    pub z: f64,                 // Position z.
    pub attributes: Attributes, // Vertex attributes.
}

impl VertexData {
    /// Construct at a point.
    pub fn new(point: Point) -> Self {
        Self {
            x: point[0],
            y: point[1],
            z: point[2],
            attributes: Attributes::new(),
        }
    }

    /// Return the position as a Point.
    pub fn position(&self) -> Point {
        Point::new(self.x, self.y, self.z)
    }

    /// Set the position from a Point.
    pub fn set_position(&mut self, point: Point) {
        self.x = point[0];
        self.y = point[1];
        self.z = point[2];
    }

    /// Return the vertex color as RGB, 0.5 grey when unset.
    pub fn color(&self) -> [f64; 3] {
        [
            self.attributes.get("r").copied().unwrap_or(0.5),
            self.attributes.get("g").copied().unwrap_or(0.5),
            self.attributes.get("b").copied().unwrap_or(0.5),
        ]
    }

    /// Set the vertex color.
    pub fn set_color(&mut self, r: f64, g: f64, b: f64) {
        self.attributes.insert("r".to_string(), r);
        self.attributes.insert("g".to_string(), g);
        self.attributes.insert("b".to_string(), b);
    }

    /// Return the vertex normal when set.
    pub fn normal(&self) -> Option<[f64; 3]> {
        let nx = self.attributes.get("nx")?;
        let ny = self.attributes.get("ny")?;
        let nz = self.attributes.get("nz")?;

        Some([*nx, *ny, *nz])
    }

    /// Set the vertex normal.
    pub fn set_normal(&mut self, nx: f64, ny: f64, nz: f64) {
        self.attributes.insert("nx".to_string(), nx);
        self.attributes.insert("ny".to_string(), ny);
        self.attributes.insert("nz".to_string(), nz);
    }
}

/// A halfedge mesh data structure for representing polygonal surfaces.
#[derive(Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Mesh")]
pub struct Mesh {
    pub halfedge: HashMap<usize, HashMap<usize, Option<usize>>>, // Halfedge connectivity.
    pub vertex: HashMap<usize, VertexData>,                      // Vertex data.
    pub face: HashMap<usize, Vec<usize>>,                        // Face vertex lists.
    #[serde(skip)]
    pub face_holes: HashMap<usize, Vec<Vec<usize>>>, // Face hole rings.
    pub facedata: HashMap<usize, HashMap<String, f64>>,          // Face attributes.
    pub edgedata: HashMap<(usize, usize), HashMap<String, f64>>, // Edge attributes.
    pub default_vertex_attributes: HashMap<String, f64>,         // Default vertex attrs.
    pub default_face_attributes: HashMap<String, f64>,           // Default face attrs.
    pub default_edge_attributes: HashMap<String, f64>,           // Default edge attrs.
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: std::sync::OnceLock<String>, // Lazy guid.
    pub name: String,                                            // Mesh name.
    #[serde(skip)]
    pub color_mode: ColorMode,                 // Active color mode.
    #[serde(skip)]
    pointcolors: Vec<Color>,                   // Vertex colors.
    #[serde(skip)]
    facecolors: Vec<Color>,                    // Face colors.
    #[serde(skip)]
    linecolors: Vec<Color>,                    // Edge colors.
    #[serde(skip)]
    widths: Vec<f64>,                          // Edge widths.
    #[serde(skip)]
    objectcolor: Color,                        // Object color.
    max_vertex: usize,                                           // Next vertex key.
    max_face: usize,                                             // Next face key.
    #[serde(skip)]
    pub triangulation: HashMap<usize, Vec<[usize; 3]>>, // Cached triangulations.
    #[serde(skip)]
    triangle_bvh_built: bool,                  // Whether the triangle caches are current.
    #[serde(skip)]
    pub tri_bvh: Option<SpatialBVH>,           // BVH over the cached triangle AABBs.
    #[serde(skip)]
    tri_aabbs: Vec<AABB>,                      // Per-triangle AABBs.
    #[serde(skip)]
    pub tri_tris: Vec<[usize; 3]>, // Triangle vertex indices into tri_vertices; session_viewer reads it for memory accounting.
    #[serde(skip)]
    tri_face_subidx: Vec<(usize, usize)>, // Face index and sub-triangle index per triangle.
    #[serde(skip)]
    pub tri_vertices: Vec<Point>, // Sequential vertex positions; session_viewer reads it for memory accounting.
    #[serde(skip)]
    tri_aabb_tree: Option<SpatialAABBTree>, // AABB tree over the cached triangle AABBs.
    #[serde(skip)]
    pub(crate) gpu_cache: crate::render_mesh::GpuCache, // Cached GPU buffers, built by render_mesh and dropped on any geometry or color change.
}

impl Default for Mesh {
    /// Construct an empty mesh.
    fn default() -> Self {
        Self::new()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Unit Newell normal of a closed ring.
fn newell_normal(pts: &[Point]) -> Vector {
    let n = pts.len();
    let mut nx = 0.0;
    let mut ny = 0.0;
    let mut nz = 0.0;

    for i in 0..n {
        let a = &pts[i];
        let b = &pts[(i + 1) % n];
        nx += (a[1] - b[1]) * (a[2] + b[2]);
        ny += (a[2] - b[2]) * (a[0] + b[0]);
        nz += (a[0] - b[0]) * (a[1] + b[1]);
    }

    let mut normal = Vector::new(nx, ny, nz);

    if !normal.normalize_self() {
        return Vector::new(0.0, 0.0, 0.0);
    }

    normal
}

/// Average of a point ring.
fn ring_centroid(pts: &[Point]) -> Point {
    let mut x = 0.0;
    let mut y = 0.0;
    let mut z = 0.0;

    for p in pts {
        x += p[0];
        y += p[1];
        z += p[2];
    }

    let n = pts.len() as f64;

    Point::new(x / n, y / n, z / n)
}

/// CDT of a planar ring projected onto its own plane; indices into pts, empty when degenerate.
fn planar_cdt(pts: &[Point]) -> Vec<(usize, usize, usize)> {
    let n = pts.len();
    let mut nx = 0.0f64;
    let mut ny = 0.0f64;
    let mut nz = 0.0f64;

    for i in 0..n {
        let a = &pts[i];
        let b = &pts[(i + 1) % n];
        nx += (a[1] - b[1]) * (a[2] + b[2]);
        ny += (a[2] - b[2]) * (a[0] + b[0]);
        nz += (a[0] - b[0]) * (a[1] + b[1]);
    }

    let nlen = (nx * nx + ny * ny + nz * nz).sqrt();

    if nlen <= 1e-12 {
        return Vec::new();
    }

    nx /= nlen;
    ny /= nlen;
    nz /= nlen;
    let mut ux = 1.0f64;
    let mut uy = 0.0f64;
    let mut uz = 0.0f64;

    if nx.abs() > 0.9 {
        ux = 0.0;
        uy = 1.0;
    }

    let dot = ux * nx + uy * ny + uz * nz;
    ux -= dot * nx;
    uy -= dot * ny;
    uz -= dot * nz;
    let um = (ux * ux + uy * uy + uz * uz).sqrt();
    ux /= um;
    uy /= um;
    uz /= um;
    let vx = ny * uz - nz * uy;
    let vy = nz * ux - nx * uz;
    let vz = nx * uy - ny * ux;
    let mut bpts = Vec::with_capacity(n);

    for p in pts {
        bpts.push(Point::new(
            p[0] * ux + p[1] * uy + p[2] * uz,
            p[0] * vx + p[1] * vy + p[2] * vz,
            0.0,
        ));
    }

    remesh_cdt::cdt_triangulate(&bpts, &[])
}

/// Twice the signed 2D area of a ring.
fn signed_area_2d(pts: &[(f64, f64)]) -> f64 {
    let mut area = 0.0;
    let n = pts.len();

    for i in 0..n {
        let j = (i + 1) % n;
        area += pts[i].0 * pts[j].1 - pts[j].0 * pts[i].1;
    }

    area
}

// ═══════════════════════════════════════════════════════════════════════════
// Loft
// ═══════════════════════════════════════════════════════════════════════════
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoftFaceRole {
    TopCap,
    BotCap,
    QuadWall,
    TriWall,
}

/// One wall face of a loft panel and the original vertices it spans.
#[derive(Default)]
pub struct LoftWallFace {
    pub face_key: usize,
    pub face_index: usize,
    pub is_quad: bool,
    pub top_v0: usize,
    pub top_v1: usize,
    pub bot_v0: usize,
    pub bot_v1: usize,
}

/// One lofted panel with its cap faces, walls and vertex maps.
#[derive(Default)]
pub struct LoftPanel {
    pub mesh: Mesh,
    pub top_face_key: Option<usize>,
    pub bot_face_key: Option<usize>,
    pub wall_faces: Vec<LoftWallFace>,
    pub face_roles: HashMap<usize, LoftFaceRole>,
    pub orig_top_to_local: HashMap<usize, usize>,
    pub orig_bot_to_local: HashMap<usize, usize>,
    pub top_vertices: Vec<usize>,
    pub bot_vertices: Vec<usize>,
}

/// Two panel walls that face each other.
pub struct LoftAdjPair {
    pub pi: usize,
    pub wi: usize,
    pub pj: usize,
    pub wj: usize,
}

/// Panels of loft_panels with their wall adjacency.
pub struct LoftResult {
    pub panels: Vec<LoftPanel>,
    pub adjacency: Vec<LoftAdjPair>,
    pub top_mesh: Mesh,
    pub bot_mesh: Mesh,
}

struct LoftFrame {
    origin: Point,
    xaxis: Vector,
    yaxis: Vector,
}

#[derive(Clone, Copy)]
struct LoftRing {
    off: usize,
    n: usize,
}

struct LoftPoly {
    bot: LoftRing,
    top: LoftRing,
}

/// Return the 2D coordinates of p in the frame.
fn loft_project(frame: &LoftFrame, p: &Point) -> (f64, f64) {
    let d = p - &frame.origin;

    (d.dot(&frame.xaxis), d.dot(&frame.yaxis))
}

/// Polyline points without the closing duplicate.
fn loft_open_points(pl: &Polyline) -> Vec<Point> {
    let mut pts = pl.get_points();

    if pts.len() > 1 {
        let f = &pts[0];
        let b = &pts[pts.len() - 1];

        if (f[0] - b[0]).abs() < 1e-12 && (f[1] - b[1]).abs() < 1e-12 && (f[2] - b[2]).abs() < 1e-12
        {
            pts.pop();
        }
    }

    pts
}

/// Return the signed area of pts projected on the frame.
fn loft_signed_area(frame: &LoftFrame, pts: &[Point]) -> f64 {
    let mut pts2d = Vec::with_capacity(pts.len());

    for p in pts {
        pts2d.push(loft_project(frame, p));
    }

    signed_area_2d(&pts2d) * 0.5
}

/// Index of the polyline with the largest bbox diagonal.
fn loft_border_index(polylines: &[Polyline]) -> usize {
    let mut border_idx = 0;
    let mut max_diag = 0.0;

    for (i, pl) in polylines.iter().enumerate() {
        let pts = pl.get_points();

        if pts.is_empty() {
            continue;
        }

        let mut minx = pts[0][0];
        let mut miny = pts[0][1];
        let mut minz = pts[0][2];
        let mut maxx = minx;
        let mut maxy = miny;
        let mut maxz = minz;

        for p in &pts {
            minx = minx.min(p[0]);
            maxx = maxx.max(p[0]);
            miny = miny.min(p[1]);
            maxy = maxy.max(p[1]);
            minz = minz.min(p[2]);
            maxz = maxz.max(p[2]);
        }

        let dx = maxx - minx;
        let dy = maxy - miny;
        let dz = maxz - minz;
        let diag = (dx * dx + dy * dy + dz * dz).sqrt();

        if diag > max_diag {
            max_diag = diag;
            border_idx = i;
        }
    }

    border_idx
}

/// Projection frame of the border polyline, y flipped so z points from bottom to top.
fn loft_frame(bottom: &Polyline, top: &Polyline) -> LoftFrame {
    let (origin, xaxis, mut yaxis, zaxis) = bottom.get_average_plane();
    let c0 = bottom.center();
    let c1 = top.center();
    let bottom_to_top = &c1 - &c0;

    if zaxis.dot(&bottom_to_top) < 0.0 {
        yaxis = -yaxis;
    }

    LoftFrame {
        origin,
        xaxis,
        yaxis,
    }
}

/// Vertex keys of pts; consecutive same-position points share one key.
fn loft_add_vkeys(mesh: &mut Mesh, pts: &[Point]) -> Vec<usize> {
    let mut keys: Vec<usize> = Vec::with_capacity(pts.len());

    for i in 0..pts.len() {
        if i > 0 {
            let prev = &pts[i - 1];
            let curr = &pts[i];
            let dx = curr[0] - prev[0];
            let dy = curr[1] - prev[1];
            let dz = curr[2] - prev[2];

            if dx * dx + dy * dy + dz * dz < 1e-20 {
                keys.push(keys[keys.len() - 1]);
                continue;
            }
        }

        keys.push(mesh.add_vertex(pts[i].clone(), None));
    }

    keys
}

/// Split triangle j, which spans corners a and c, at boundary vertex b.
fn loft_split_triangle(tris: &mut Vec<[usize; 3]>, j: usize, a: usize, c: usize, b: usize) {
    let ft = tris[j];
    let (t1, t2) = if (ft[0] == a || ft[0] == c) && (ft[1] == a || ft[1] == c) {
        ([ft[0], b, ft[2]], [b, ft[1], ft[2]])
    } else if (ft[1] == a || ft[1] == c) && (ft[2] == a || ft[2] == c) {
        ([ft[0], ft[1], b], [ft[0], b, ft[2]])
    } else {
        ([ft[0], ft[1], b], [b, ft[1], ft[2]])
    };
    tris[j] = t1;
    tris.push(t2);
}

/// Insert boundary vertices the CDT skipped as collinear, one per pass.
fn loft_fix_collinear(tris: &mut Vec<[usize; 3]>, fvkeys: &[usize]) {
    let n = fvkeys.len();

    for _pass in 0..n {
        let mut used: HashSet<usize> = HashSet::new();

        for t in tris.iter() {
            for v in t {
                used.insert(*v);
            }
        }

        let mut changed = false;

        for k in 0..n {
            if changed {
                break;
            }

            let b = fvkeys[k];

            if used.contains(&b) {
                continue;
            }

            let a = fvkeys[(k + n - 1) % n];
            let c = fvkeys[(k + 1) % n];

            for j in 0..tris.len() {
                let has_a = tris[j][0] == a || tris[j][1] == a || tris[j][2] == a;
                let has_c = tris[j][0] == c || tris[j][1] == c || tris[j][2] == c;

                if !has_a || !has_c {
                    continue;
                }

                loft_split_triangle(tris, j, a, c, b);
                changed = true;
                break;
            }
        }

        if !changed {
            return;
        }
    }
}

/// Drop triangles with zero area in the projected integer grid.
fn loft_drop_degenerate(tris: &[[usize; 3]], mesh: &Mesh, frame: &LoftFrame) -> Vec<[usize; 3]> {
    let sc = 1e6;
    let mut kept = Vec::with_capacity(tris.len());

    for t in tris {
        let (u0, v0) = loft_project(frame, &mesh.vertex[&t[0]].position());
        let (u1, v1) = loft_project(frame, &mesh.vertex[&t[1]].position());
        let (u2, v2) = loft_project(frame, &mesh.vertex[&t[2]].position());
        let iu0 = (u0 * sc).round() as i64;
        let iv0 = (v0 * sc).round() as i64;
        let iu1 = (u1 * sc).round() as i64;
        let iv1 = (v1 * sc).round() as i64;
        let iu2 = (u2 * sc).round() as i64;
        let iv2 = (v2 * sc).round() as i64;

        if (iu1 - iu0) * (iv2 - iv0) - (iv1 - iv0) * (iu2 - iu0) != 0 {
            kept.push(*t);
        }
    }

    kept
}

/// One n-gon cap with stored CDT triangulation and hole rings; reversed for the bottom.
fn loft_cap(
    mesh: &mut Mesh,
    frame: &LoftFrame,
    rings: &[LoftRing],
    pts: &[Point],
    vkeys: &[usize],
    reverse: bool,
    fix_collinear: bool,
) {
    let mut border_2d: Vec<Point> = Vec::new();
    let mut outer: Vec<usize> = Vec::new();

    for i in 0..rings[0].n {
        let vi = rings[0].off + i;

        if !outer.is_empty() && vkeys[vi] == vkeys[outer[outer.len() - 1]] {
            continue;
        }

        let (u, v) = loft_project(frame, &pts[vi]);
        border_2d.push(Point::new(u, v, 0.0));
        outer.push(vi);
    }

    let mut flat = outer.clone();
    let mut holes_2d: Vec<Vec<Point>> = Vec::new();
    let mut hole_rings: Vec<Vec<usize>> = Vec::new();

    for r in &rings[1..] {
        let mut hole = Vec::new();
        let mut ring = Vec::new();

        for i in r.off..r.off + r.n {
            let (u, v) = loft_project(frame, &pts[i]);
            hole.push(Point::new(u, v, 0.0));
            flat.push(i);
            ring.push(vkeys[i]);
        }

        holes_2d.push(hole);
        hole_rings.push(ring);
    }

    let tris = remesh_cdt::cdt_triangulate(&border_2d, &holes_2d);
    let mut fvkeys = Vec::with_capacity(outer.len());

    for i in 0..outer.len() {
        fvkeys.push(vkeys[outer[if reverse { outer.len() - 1 - i } else { i }]]);
    }

    let Some(fk) = mesh.add_face(fvkeys.clone(), None) else {
        return;
    };
    let mut tri_list: Vec<[usize; 3]> = Vec::with_capacity(tris.len());

    for &(a, b, c) in &tris {
        if reverse {
            tri_list.push([vkeys[flat[a]], vkeys[flat[c]], vkeys[flat[b]]]);
        } else {
            tri_list.push([vkeys[flat[a]], vkeys[flat[b]], vkeys[flat[c]]]);
        }
    }

    if fix_collinear {
        loft_fix_collinear(&mut tri_list, &fvkeys);
        tri_list = loft_drop_degenerate(&tri_list, mesh, frame);
    }

    let has_holes = !hole_rings.is_empty() && !tri_list.is_empty();
    mesh.set_face_triangulation(fk, tri_list);

    if has_holes {
        mesh.set_face_holes(fk, hole_rings);
    }
}

/// Squared 2D length of edge i of a ring.
fn loft_edge_sq_2d(frame: &LoftFrame, pts: &[Point], i: usize) -> f64 {
    let j = (i + 1) % pts.len();
    let (xi, yi) = loft_project(frame, &pts[i]);
    let (xj, yj) = loft_project(frame, &pts[j]);
    let dx = xj - xi;
    let dy = yj - yi;

    dx * dx + dy * dy
}

/// Start offsets (ia, ib): the longest bottom edge, and the top offset that minimizes the projected gap.
fn loft_wall_start(frame: &LoftFrame, bpts: &[Point], tpts: &[Point]) -> (usize, usize) {
    let bot_n = bpts.len();
    let top_n = tpts.len();
    let mut ia = 0;
    let mut ib = 0;
    let mut max_b = 0.0;

    for k in 0..bot_n {
        let v = loft_edge_sq_2d(frame, bpts, k);

        if v > max_b {
            max_b = v;
            ia = k;
        }
    }

    if bot_n != top_n {
        return (ia, ib);
    }

    let mut min_total = f64::MAX;

    for cand in 0..top_n {
        let mut total = 0.0;

        for k in 0..bot_n {
            let (xb, yb) = loft_project(frame, &bpts[(ia + k) % bot_n]);
            let (xt, yt) = loft_project(frame, &tpts[(cand + k) % top_n]);
            total += (xt - xb) * (xt - xb) + (yt - yb) * (yt - yb);
        }

        if total < min_total {
            min_total = total;
            ib = cand;
        }
    }

    (ia, ib)
}

/// Return whether a and b coincide within 1e-10.
fn loft_same_point(a: &Point, b: &Point) -> bool {
    (a[0] - b[0]).abs() < 1e-10 && (a[1] - b[1]).abs() < 1e-10 && (a[2] - b[2]).abs() < 1e-10
}

/// Quad walls for equal counts; a collapsed bottom or top edge gives a triangle.
fn loft_walls_quads(
    mesh: &mut Mesh,
    poly: &LoftPoly,
    start: (usize, usize),
    bpts: &[Point],
    tpts: &[Point],
    bot_vkeys: &[usize],
    top_vkeys: &[usize],
) {
    let (ia, ib) = start;
    let bot_n = poly.bot.n;
    let top_n = poly.top.n;

    for k in 0..bot_n {
        let cb = poly.bot.off + (ia + k) % bot_n;
        let ct = poly.top.off + (ib + k) % top_n;
        let nb = poly.bot.off + (ia + k + 1) % bot_n;
        let nt = poly.top.off + (ib + k + 1) % top_n;
        let bot_col = loft_same_point(&bpts[(ia + k) % bot_n], &bpts[(ia + k + 1) % bot_n]);
        let top_col = loft_same_point(&tpts[(ib + k) % top_n], &tpts[(ib + k + 1) % top_n]);

        if bot_col && top_col {
            continue;
        } else if bot_col {
            mesh.add_face(vec![bot_vkeys[cb], top_vkeys[nt], top_vkeys[ct]], None);
        } else if top_col {
            mesh.add_face(vec![bot_vkeys[cb], bot_vkeys[nb], top_vkeys[ct]], None);
        } else {
            mesh.add_face(
                vec![bot_vkeys[cb], bot_vkeys[nb], top_vkeys[nt], top_vkeys[ct]],
                None,
            );
        }
    }
}

/// Normalized arc lengths of a ring starting at offset start.
fn loft_arcs(pts: &[Point], start: usize) -> Vec<f64> {
    let n = pts.len();
    let mut arcs = vec![0.0; n + 1];

    for k in 0..n {
        let i = (start + k) % n;
        let j = (start + k + 1) % n;
        let dx = pts[j][0] - pts[i][0];
        let dy = pts[j][1] - pts[i][1];
        let dz = pts[j][2] - pts[i][2];
        arcs[k + 1] = arcs[k] + (dx * dx + dy * dy + dz * dz).sqrt();
    }

    let inv = if arcs[n] > 0.0 { 1.0 / arcs[n] } else { 1.0 };

    for a in arcs.iter_mut() {
        *a *= inv;
    }

    arcs
}

/// Zipper walls for unequal counts: quads where arc lengths meet, triangles elsewhere.
fn loft_walls_zipper(
    mesh: &mut Mesh,
    poly: &LoftPoly,
    start: (usize, usize),
    bpts: &[Point],
    tpts: &[Point],
    bot_vkeys: &[usize],
    top_vkeys: &[usize],
) {
    let (ia, ib) = start;
    let bot_n = poly.bot.n;
    let top_n = poly.top.n;
    let b_arcs = loft_arcs(bpts, ia);
    let t_arcs = loft_arcs(tpts, ib);
    let mut bi = 0;
    let mut ti = 0;

    while bi < bot_n || ti < top_n {
        let cb = poly.bot.off + (ia + bi) % bot_n;
        let ct = poly.top.off + (ib + ti) % top_n;
        let nb = poly.bot.off + (ia + bi + 1) % bot_n;
        let nt = poly.top.off + (ib + ti + 1) % top_n;

        if bi >= bot_n {
            mesh.add_face(vec![bot_vkeys[cb], top_vkeys[ct], top_vkeys[nt]], None);
            ti += 1;
        } else if ti >= top_n {
            mesh.add_face(vec![bot_vkeys[cb], bot_vkeys[nb], top_vkeys[ct]], None);
            bi += 1;
        } else if (b_arcs[bi + 1] - t_arcs[ti + 1]).abs() < 1e-9 {
            mesh.add_face(
                vec![bot_vkeys[cb], bot_vkeys[nb], top_vkeys[nt], top_vkeys[ct]],
                None,
            );
            bi += 1;
            ti += 1;
        } else if b_arcs[bi + 1] < t_arcs[ti + 1] {
            mesh.add_face(vec![bot_vkeys[cb], bot_vkeys[nb], top_vkeys[ct]], None);
            bi += 1;
        } else {
            mesh.add_face(vec![bot_vkeys[cb], top_vkeys[ct], top_vkeys[nt]], None);
            ti += 1;
        }
    }
}

/// Add the wall faces of one polygon, as quads when the rings match in size, zipped otherwise.
fn loft_walls(
    mesh: &mut Mesh,
    frame: &LoftFrame,
    poly: &LoftPoly,
    all_bot: &[Point],
    all_top: &[Point],
    bot_vkeys: &[usize],
    top_vkeys: &[usize],
) {
    let bpts = &all_bot[poly.bot.off..poly.bot.off + poly.bot.n];
    let tpts = &all_top[poly.top.off..poly.top.off + poly.top.n];
    let start = loft_wall_start(frame, bpts, tpts);

    if poly.bot.n == poly.top.n {
        loft_walls_quads(mesh, poly, start, bpts, tpts, bot_vkeys, top_vkeys);
    } else {
        loft_walls_zipper(mesh, poly, start, bpts, tpts, bot_vkeys, top_vkeys);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Loft panels
// ═══════════════════════════════════════════════════════════════════════════
/// Drop ring points collinear with their neighbors, until none is left.
fn lp_merge_collinear(pts: &mut Vec<Point>, vkeys: &mut Vec<usize>) {
    let tol = Tolerance::APPROXIMATION;
    let zt2 = Tolerance::ZERO_TOLERANCE * Tolerance::ZERO_TOLERANCE;
    let bound = pts.len();

    for _pass in 0..bound {
        let m = pts.len();

        if m < 3 {
            return;
        }

        let mut changed = false;
        let mut np: Vec<Point> = Vec::new();
        let mut nk: Vec<usize> = Vec::new();

        for i in 0..m {
            let p = (i + m - 1) % m;
            let nx = (i + 1) % m;
            let ax = pts[i][0] - pts[p][0];
            let ay = pts[i][1] - pts[p][1];
            let az = pts[i][2] - pts[p][2];
            let bx = pts[nx][0] - pts[i][0];
            let by = pts[nx][1] - pts[i][1];
            let bz = pts[nx][2] - pts[i][2];
            let cx = ay * bz - az * by;
            let cy = az * bx - ax * bz;
            let cz = ax * by - ay * bx;
            let a2 = ax * ax + ay * ay + az * az;
            let b2 = bx * bx + by * by + bz * bz;

            if a2 < zt2 || b2 < zt2 || cx * cx + cy * cy + cz * cz < tol * tol * a2 * b2 {
                changed = true;
            } else {
                np.push(pts[i].clone());
                nk.push(vkeys[i]);
            }
        }

        *pts = np;
        *vkeys = nk;

        if !changed {
            return;
        }
    }
}

/// Drop ring points closer than a thousandth of the longest edge to their predecessor.
fn lp_merge_close(pts: &mut Vec<Point>, vkeys: &mut Vec<usize>) {
    let sz = pts.len();
    let mut max_edge = 0.0f64;

    for i in 0..sz {
        max_edge = max_edge.max(pts[i].distance(&pts[(i + 1) % sz], None));
    }

    let stol = max_edge * 0.001;
    let mut tp: Vec<Point> = Vec::new();
    let mut tk: Vec<usize> = Vec::new();

    for i in 0..sz {
        if tp.is_empty() || tp[tp.len() - 1].distance(&pts[i], None) > stol {
            tp.push(pts[i].clone());
            tk.push(vkeys[i]);
        }
    }

    while tp.len() >= 3 && tp[tp.len() - 1].distance(&tp[0], None) <= stol {
        tp.pop();
        tk.pop();
    }

    if tp.len() < 3 {
        return;
    }

    *pts = tp;
    *vkeys = tk;
}

/// Return p moved by gap toward (cx, cy, cz).
fn lp_offset_toward(p: &Point, cx: f64, cy: f64, cz: f64, gap: f64) -> Point {
    let mut dx = cx - p[0];
    let mut dy = cy - p[1];
    let mut dz = cz - p[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt();

    if len > 1e-10 {
        dx *= gap / len;
        dy *= gap / len;
        dz *= gap / len;
    }

    p + Vector::new(dx, dy, dz)
}

/// Return the average of the face vertex positions.
fn lp_face_centroid(m: &Mesh, fk: usize) -> Point {
    let vkeys = m.face_vertices(fk).unwrap();
    let mut cx = 0.0;
    let mut cy = 0.0;
    let mut cz = 0.0;

    for &vk in vkeys {
        let p = m.vertex_point(vk).unwrap();
        cx += p[0];
        cy += p[1];
        cz += p[2];
    }

    let n = vkeys.len() as f64;

    Point::new(cx / n, cy / n, cz / n)
}

/// Greedy top/bottom face pairs by centroid distance, sorted by key.
fn lp_match_faces(top_mesh: &Mesh, bot_mesh: &Mesh) -> Vec<(usize, usize)> {
    let tfks = top_mesh.faces();
    let bfks = bot_mesh.faces();
    let mut dists: Vec<(f64, usize, usize)> = Vec::with_capacity(tfks.len() * bfks.len());

    for (ti, &tfk) in tfks.iter().enumerate() {
        for (bi, &bfk) in bfks.iter().enumerate() {
            let d =
                lp_face_centroid(top_mesh, tfk).distance(&lp_face_centroid(bot_mesh, bfk), None);

            dists.push((d, ti, bi));
        }
    }

    dists.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mut top_used = vec![false; tfks.len()];
    let mut bot_used = vec![false; bfks.len()];
    let mut face_match: Vec<(usize, usize)> = Vec::new();

    for &(_, ti, bi) in &dists {
        if top_used[ti] || bot_used[bi] {
            continue;
        }

        face_match.push((tfks[ti], bfks[bi]));
        top_used[ti] = true;
        bot_used[bi] = true;
    }

    face_match.sort();

    face_match
}

/// Reverse top and bottom rings so the top normal points from bottom to top and the bottom normal away.
fn lp_orient_rings(
    top_pts: &mut [Point],
    top_vkeys: &mut [usize],
    bot_pts: &mut [Point],
    bot_vkeys: &mut [usize],
) {
    let tc = ring_centroid(top_pts);
    let bc = ring_centroid(bot_pts);
    let mut axis = &tc - &bc;

    if !axis.normalize_self() {
        return;
    }

    if newell_normal(top_pts).dot(&axis) < 0.0 {
        top_pts.reverse();
        top_vkeys.reverse();
    }

    if newell_normal(bot_pts).dot(&axis) > 0.0 {
        bot_pts.reverse();
        bot_vkeys.reverse();
    }
}

/// Cap face over local keys with its planar CDT stored.
fn lp_add_cap(mesh: &mut Mesh, cap: &[usize], pts: &[Point]) -> Option<usize> {
    let fk = mesh.add_face(cap.to_vec(), None)?;

    if cap.len() < 3 {
        return Some(fk);
    }

    let tris = planar_cdt(pts);

    if tris.is_empty() {
        return Some(fk);
    }

    let mut tri_list: Vec<[usize; 3]> = Vec::with_capacity(tris.len());

    for &(a, b, c) in &tris {
        tri_list.push([cap[a], cap[b], cap[c]]);
    }

    mesh.set_face_triangulation(fk, tri_list);

    Some(fk)
}

/// Edge midpoints of a ring.
fn lp_midpoints(pts: &[Point]) -> Vec<Point> {
    let n = pts.len();
    let mut mids = Vec::with_capacity(n);

    for (i, p) in pts.iter().enumerate() {
        let q = &pts[(i + 1) % n];
        mids.push(Point::new(
            (p[0] + q[0]) * 0.5,
            (p[1] + q[1]) * 0.5,
            (p[2] + q[2]) * 0.5,
        ));
    }

    mids
}

/// Index of the point in pts nearest to p.
fn lp_nearest(p: &Point, pts: &[Point]) -> usize {
    let mut best_d = f64::MAX;
    let mut best = 0;

    for (i, q) in pts.iter().enumerate() {
        let d = p.distance(q, None);

        if d < best_d {
            best_d = d;
            best = i;
        }
    }

    best
}

/// Quad wall over matched edge j of the bottom and ti of the top, inset by edge_gap.
fn lp_add_quad(
    panel: &mut LoftPanel,
    b0: usize,
    b1: usize,
    t0: usize,
    t1: usize,
    edge_gap: f64,
) -> Option<usize> {
    if edge_gap <= 0.0 {
        return panel.mesh.add_face(vec![b0, t1, t0, b1], None);
    }

    let pb0 = panel.mesh.vertex_point(b0).unwrap();
    let pb1 = panel.mesh.vertex_point(b1).unwrap();
    let pt0 = panel.mesh.vertex_point(t0).unwrap();
    let pt1 = panel.mesh.vertex_point(t1).unwrap();
    let cx = (pb0[0] + pb1[0] + pt0[0] + pt1[0]) * 0.25;
    let cy = (pb0[1] + pb1[1] + pt0[1] + pt1[1]) * 0.25;
    let cz = (pb0[2] + pb1[2] + pt0[2] + pt1[2]) * 0.25;
    let nb0 = panel
        .mesh
        .add_vertex(lp_offset_toward(&pb0, cx, cy, cz, edge_gap), None);

    let nb1 = panel
        .mesh
        .add_vertex(lp_offset_toward(&pb1, cx, cy, cz, edge_gap), None);

    panel.mesh.add_face(vec![nb0, t1, t0, nb1], None)
}

/// A triangle from every unmatched top edge to the nearest bottom vertex.
fn lp_add_top_triangles(
    panel: &mut LoftPanel,
    top_mids: &[Point],
    top_vkeys: &[usize],
    bot_pts: &[Point],
    bot_vkeys: &[usize],
    top_used: &[bool],
) {
    let n = top_vkeys.len();

    for i in 0..n {
        if top_used[i] {
            continue;
        }

        let t0 = panel.orig_top_to_local[&top_vkeys[i]];
        let t1 = panel.orig_top_to_local[&top_vkeys[(i + 1) % n]];
        let bv = panel.orig_bot_to_local[&bot_vkeys[lp_nearest(&top_mids[i], bot_pts)]];

        if let Some(fk) = panel.mesh.add_face(vec![t1, t0, bv], None) {
            panel.wall_faces.push(LoftWallFace {
                face_key: fk,
                ..Default::default()
            });
        }
    }
}

/// Walls of one panel: a quad per mutually nearest edge pair, a triangle for every unmatched edge.
#[allow(clippy::too_many_arguments)]
fn lp_add_walls(
    panel: &mut LoftPanel,
    top_pts: &[Point],
    top_vkeys: &[usize],
    bot_pts: &[Point],
    bot_vkeys: &[usize],
    edge_gap: f64,
    edge_match_threshold: f64,
    skip_triangles: bool,
) {
    let n = top_pts.len();
    let m = bot_pts.len();
    let top_mids = lp_midpoints(top_pts);
    let bot_mids = lp_midpoints(bot_pts);
    let mut bot_to_top = vec![0usize; m];
    let mut top_to_bot = vec![0usize; n];
    let mut bot_dist = vec![0.0f64; m];

    for j in 0..m {
        bot_to_top[j] = lp_nearest(&bot_mids[j], &top_mids);
        bot_dist[j] = bot_mids[j].distance(&top_mids[bot_to_top[j]], None);
    }

    for i in 0..n {
        top_to_bot[i] = lp_nearest(&top_mids[i], &bot_mids);
    }

    let mut avg = 0.0;

    for d in &bot_dist {
        avg += d;
    }

    let threshold = avg / m as f64 * edge_match_threshold;
    let mut top_used = vec![false; n];

    for j in 0..m {
        let b0 = panel.orig_bot_to_local[&bot_vkeys[j]];
        let b1 = panel.orig_bot_to_local[&bot_vkeys[(j + 1) % m]];
        let ti = bot_to_top[j];

        if bot_dist[j] <= threshold && top_to_bot[ti] == j {
            let t0 = panel.orig_top_to_local[&top_vkeys[ti]];
            let t1 = panel.orig_top_to_local[&top_vkeys[(ti + 1) % n]];

            if let Some(fk) = lp_add_quad(panel, b0, b1, t0, t1, edge_gap) {
                panel.wall_faces.push(LoftWallFace {
                    face_key: fk,
                    is_quad: true,
                    top_v0: top_vkeys[ti],
                    top_v1: top_vkeys[(ti + 1) % n],
                    bot_v0: bot_vkeys[(j + 1) % m],
                    bot_v1: bot_vkeys[j],
                    ..Default::default()
                });
            }

            top_used[ti] = true;
        } else if !skip_triangles {
            let tv = panel.orig_top_to_local[&top_vkeys[lp_nearest(&bot_mids[j], top_pts)]];

            if let Some(fk) = panel.mesh.add_face(vec![b0, tv, b1], None) {
                panel.wall_faces.push(LoftWallFace {
                    face_key: fk,
                    ..Default::default()
                });
            }
        }
    }

    if !skip_triangles {
        lp_add_top_triangles(panel, &top_mids, top_vkeys, bot_pts, bot_vkeys, &top_used);
    }
}

/// One panel between matched faces tfk of the top mesh and bfk of the bottom mesh.
#[allow(clippy::too_many_arguments)]
fn lp_build_panel(
    top_mesh: &Mesh,
    bot_mesh: &Mesh,
    tfk: usize,
    bfk: usize,
    edge_gap: f64,
    edge_match_threshold: f64,
    add_caps: bool,
    skip_triangles: bool,
) -> LoftPanel {
    let mut panel = LoftPanel::default();
    let mut top_vkeys: Vec<usize> = top_mesh.face_vertices(tfk).unwrap().clone();
    let mut bot_vkeys: Vec<usize> = bot_mesh.face_vertices(bfk).unwrap().clone();
    let mut top_pts: Vec<Point> = Vec::new();
    let mut bot_pts: Vec<Point> = Vec::new();

    for &vk in &top_vkeys {
        top_pts.push(top_mesh.vertex_point(vk).unwrap());
    }

    for &vk in &bot_vkeys {
        bot_pts.push(bot_mesh.vertex_point(vk).unwrap());
    }

    lp_merge_collinear(&mut top_pts, &mut top_vkeys);
    lp_merge_collinear(&mut bot_pts, &mut bot_vkeys);
    lp_merge_close(&mut top_pts, &mut top_vkeys);
    lp_orient_rings(&mut top_pts, &mut top_vkeys, &mut bot_pts, &mut bot_vkeys);

    for i in 0..top_pts.len() {
        let lk = panel.mesh.add_vertex(top_pts[i].clone(), None);
        panel.orig_top_to_local.insert(top_vkeys[i], lk);
        panel.top_vertices.push(lk);
    }

    for j in 0..bot_pts.len() {
        let lk = panel.mesh.add_vertex(bot_pts[j].clone(), None);
        panel.orig_bot_to_local.insert(bot_vkeys[j], lk);
        panel.bot_vertices.push(lk);
    }

    if add_caps {
        let cap = panel.top_vertices.clone();
        panel.top_face_key = lp_add_cap(&mut panel.mesh, &cap, &top_pts);
    }

    lp_add_walls(
        &mut panel,
        &top_pts,
        &top_vkeys,
        &bot_pts,
        &bot_vkeys,
        edge_gap,
        edge_match_threshold,
        skip_triangles,
    );

    if add_caps {
        let cap = panel.bot_vertices.clone();
        panel.bot_face_key = lp_add_cap(&mut panel.mesh, &cap, &bot_pts);
    }

    let mut fkey_to_idx: HashMap<usize, usize> = HashMap::new();

    for (fi, fk) in panel.mesh.faces().into_iter().enumerate() {
        fkey_to_idx.insert(fk, fi);
    }

    for w in panel.wall_faces.iter_mut() {
        w.face_index = fkey_to_idx[&w.face_key];
        panel.face_roles.insert(
            w.face_key,
            if w.is_quad {
                LoftFaceRole::QuadWall
            } else {
                LoftFaceRole::TriWall
            },
        );
    }

    if let Some(fk) = panel.top_face_key {
        panel.face_roles.insert(fk, LoftFaceRole::TopCap);
    }

    if let Some(fk) = panel.bot_face_key {
        panel.face_roles.insert(fk, LoftFaceRole::BotCap);
    }

    panel
}

/// Quad walls of different panels that share a top edge, once per pair.
fn lp_adjacency(panels: &[LoftPanel]) -> Vec<LoftAdjPair> {
    let mut edge_to_wall: HashMap<(usize, usize), (usize, usize)> = HashMap::new();

    for (pi, panel) in panels.iter().enumerate() {
        for (wi, w) in panel.wall_faces.iter().enumerate() {
            if w.is_quad {
                edge_to_wall.insert((w.top_v0, w.top_v1), (pi, wi));
            }
        }
    }

    let mut adjacency: Vec<LoftAdjPair> = Vec::new();

    for (pi, panel) in panels.iter().enumerate() {
        for (wi, w) in panel.wall_faces.iter().enumerate() {
            if !w.is_quad {
                continue;
            }

            if let Some(&(pj, wj)) = edge_to_wall.get(&(w.top_v1, w.top_v0)) {
                if pj > pi {
                    adjacency.push(LoftAdjPair { pi, wi, pj, wj });
                }
            }
        }
    }

    adjacency
}

/// One face per panel, from its local top or bottom ring.
fn lp_ordered_mesh(panels: &[LoftPanel], top: bool) -> Mesh {
    let mut ordered = Mesh::new();

    for (i, panel) in panels.iter().enumerate() {
        let ring = if top {
            &panel.top_vertices
        } else {
            &panel.bot_vertices
        };
        let mut vks = Vec::with_capacity(ring.len());

        for &lk in ring {
            vks.push(ordered.add_vertex(panel.mesh.vertex_point(lk).unwrap(), None));
        }

        ordered.add_face(vks, Some(i));
    }

    ordered
}

// ═══════════════════════════════════════════════════════════════════════════
// Miter contours
// ═══════════════════════════════════════════════════════════════════════════
/// Corners whose interior angle is below max_angle_deg.
fn fold_chamfer_mask(pts: &[Point], max_angle_deg: f64) -> Vec<bool> {
    let n = pts.len();
    let mut mask = vec![false; n];

    for i in 0..n {
        let prev = (i + n - 1) % n;
        let next = (i + 1) % n;
        let dp = &pts[prev] - &pts[i];
        let dn = &pts[next] - &pts[i];
        let lp = dp.magnitude();
        let ln = dn.magnitude();

        if lp < 1e-12 || ln < 1e-12 {
            continue;
        }

        let cos_a = (dp.dot(&dn) / (lp * ln)).clamp(-1.0, 1.0);
        mask[i] = cos_a.acos() * Tolerance::TO_DEGREES < max_angle_deg;
    }

    mask
}

/// Ring with every masked corner cut back by s, capped at a third of the shortest edge.
fn fold_chamfer(pts: &[Point], s: f64, mask: &[bool]) -> Vec<Point> {
    let n = pts.len();

    if s <= 0.0 {
        return pts.to_vec();
    }

    let mut min_edge = f64::MAX;

    for i in 0..n {
        let j = (i + 1) % n;
        let d = &pts[j] - &pts[i];
        min_edge = min_edge.min(d.magnitude());
    }

    let sc = s.min(min_edge / 3.0);
    let mut result = Vec::with_capacity(2 * n);

    for i in 0..n {
        if !mask[i] {
            result.push(pts[i].clone());
            continue;
        }

        let prev = (i + n - 1) % n;
        let next = (i + 1) % n;
        let dp = &pts[prev] - &pts[i];
        let dn = &pts[next] - &pts[i];
        let lp = dp.magnitude();
        let ln = dn.magnitude();
        let sp = if lp > 1e-12 { sc / lp } else { 0.0 };
        let sn = if ln > 1e-12 { sc / ln } else { 0.0 };
        result.push(&pts[i] + &dp * sp);
        result.push(&pts[i] + &dn * sn);
    }

    result
}

/// Newell normal of a face, +z when the face is missing.
fn fold_face_normal(mesh: &Mesh, fk: usize) -> Vector {
    match mesh.face_points(fk) {
        Some(pts) => newell_normal(&pts),
        None => Vector::new(0.0, 0.0, 1.0),
    }
}

/// One miter plane per edge, through the edge midpoint along the averaged neighbor normal.
fn miter_planes(
    shell: &Mesh,
    efm: &HashMap<(usize, usize), usize>,
    fverts: &[usize],
    pts: &[Point],
    fn_: &Vector,
) -> Vec<Plane> {
    let n = fverts.len();
    let mut planes = Vec::with_capacity(n);

    for i in 0..n {
        let j = (i + 1) % n;
        let mut avg_n = fn_.clone();

        if let Some(&adj_fk) = efm.get(&(fverts[j], fverts[i])) {
            let adj = fold_face_normal(shell, adj_fk);
            let mut sum = fn_ + &adj;

            if sum.magnitude() > 0.1 && sum.normalize_self() {
                avg_n = sum;
            }
        }

        let mut edge_dir = &pts[j] - &pts[i];

        if !edge_dir.normalize_self() {
            return Vec::new();
        }

        let mut mn = avg_n.cross(&edge_dir);

        if !mn.normalize_self() {
            return Vec::new();
        }

        let mid = Point::new(
            (pts[i][0] + pts[j][0]) / 2.0,
            (pts[i][1] + pts[j][1]) / 2.0,
            (pts[i][2] + pts[j][2]) / 2.0,
        );
        planes.push(Plane::from_point_normal(mid, mn, None));
    }

    planes
}

/// Where the corner lines pierce a plane, empty when any misses.
fn miter_contour(corner_lines: &[Line], plane: &Plane) -> Vec<Point> {
    let mut contour = Vec::with_capacity(corner_lines.len());

    for line in corner_lines {
        let Some(p) = crate::intersection::line_plane(line, plane, false) else {
            return Vec::new();
        };
        contour.push(p);
    }

    contour
}

// ═══════════════════════════════════════════════════════════════════════════
// Cutting
// ═══════════════════════════════════════════════════════════════════════════
/// One face of a cut and the input face it came from.
#[derive(Clone)]
struct CutFace {
    rings: Vec<Vec<usize>>, // Outer ring, then hole rings.
    parent: Option<usize>,  // Input face key, none for a cap.
}

/// Positions of a key ring.
fn cut_points(ring: &[usize], points: &BTreeMap<usize, Point>) -> Vec<Point> {
    let mut result = Vec::with_capacity(ring.len());

    for key in ring {
        result.push(points[key].clone());
    }

    result
}

/// Twice the signed area of a key ring in plane coordinates.
fn cut_area(ring: &[usize], uv: &BTreeMap<usize, (f64, f64)>) -> f64 {
    let mut area = 0.0;

    for i in 0..ring.len() {
        let a = uv[&ring[i]];
        let b = uv[&ring[(i + 1) % ring.len()]];
        area += a.0 * b.1 - b.0 * a.1;
    }

    area
}

/// Whether p lies inside the key rings by the even-odd rule.
fn cut_inside(p: (f64, f64), rings: &[Vec<usize>], uv: &BTreeMap<usize, (f64, f64)>) -> bool {
    let mut inside = false;

    for ring in rings {
        for i in 0..ring.len() {
            let a = uv[&ring[i]];
            let b = uv[&ring[(i + 1) % ring.len()]];

            if (a.1 > p.1) != (b.1 > p.1) && p.0 < a.0 + (p.1 - a.1) * (b.0 - a.0) / (b.1 - a.1) {
                inside = !inside;
            }
        }
    }

    inside
}

/// Split a closed walk into simple loops where it revisits a vertex; loops under three vertices are dropped.
fn cut_split(walk: &[usize], loops: &mut Vec<Vec<usize>>) {
    let mut stack: Vec<usize> = Vec::new();
    let mut index: BTreeMap<usize, usize> = BTreeMap::new();

    for key in walk {
        let Some(&start) = index.get(key) else {
            index.insert(*key, stack.len());
            stack.push(*key);
            continue;
        };

        if stack.len() - start > 2 {
            loops.push(stack[start..].to_vec());
        }

        for removed in &stack[start + 1..] {
            index.remove(removed);
        }

        stack.truncate(start + 1);
    }

    if stack.len() > 2 {
        loops.push(stack);
    }
}

/// Closed loops of directed edges, turning sharpest left where loops meet; open chains are dropped.
fn cut_loops(
    mut edges: BTreeMap<usize, Vec<usize>>,
    uv: &BTreeMap<usize, (f64, f64)>,
) -> Vec<Vec<usize>> {
    let mut loops = Vec::new();

    while let Some(mut first) = edges.first_entry() {
        let mut walk = vec![*first.key()];
        let mut prev = walk[0];
        let mut cur = first.get_mut().pop().unwrap_or(prev);

        if first.get().is_empty() {
            first.remove();
        }

        while cur != walk[0] {
            let Some(targets) = edges.get_mut(&cur) else {
                walk.clear();
                break;
            };

            walk.push(cur);
            let ax = uv[&cur].0 - uv[&prev].0;
            let ay = uv[&cur].1 - uv[&prev].1;
            let mut best = 0;
            let mut turn = -4.0;

            for (j, target) in targets.iter().enumerate() {
                let bx = uv[target].0 - uv[&cur].0;
                let by = uv[target].1 - uv[&cur].1;
                let angle = (ax * by - ay * bx).atan2(ax * bx + ay * by);

                if angle > turn {
                    turn = angle;
                    best = j;
                }
            }

            prev = cur;
            cur = targets.remove(best);

            if targets.is_empty() {
                edges.remove(&prev);
            }
        }

        cut_split(&walk, &mut loops);
    }

    loops
}

/// Loops wound like the largest one become outer rings, each taking the opposite-wound loops inside it as holes.
fn cut_regions(loops: &[Vec<usize>], uv: &BTreeMap<usize, (f64, f64)>) -> Vec<CutFace> {
    let mut areas: Vec<f64> = Vec::with_capacity(loops.len());
    let mut largest = 0.0f64;

    for ring in loops {
        let area = cut_area(ring, uv);
        areas.push(area);

        if area.abs() > largest.abs() {
            largest = area;
        }
    }

    let mut regions: Vec<CutFace> = Vec::new();
    let mut sizes: Vec<f64> = Vec::new();

    for (ring, area) in loops.iter().zip(&areas) {
        if area * largest > 0.0 {
            regions.push(CutFace {
                rings: vec![ring.clone()],
                parent: None,
            });
            sizes.push(area.abs());
        }
    }

    for (ring, area) in loops.iter().zip(&areas) {
        if area * largest >= 0.0 {
            continue;
        }

        let a = uv[&ring[0]];
        let b = uv[&ring[1]];
        let mid = ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5);
        let mut owner: Option<usize> = None;

        for (r, region) in regions.iter().enumerate() {
            if cut_inside(mid, &region.rings[..1], uv) && owner.is_none_or(|o| sizes[r] < sizes[o])
            {
                owner = Some(r);
            }
        }

        if let Some(o) = owner {
            regions[o].rings.push(ring.clone());
        }
    }

    regions
}

/// CDT of a face with hole rings, wound like its outer ring; empty when degenerate.
fn cut_triangulation(piece: &CutFace, points: &BTreeMap<usize, Point>) -> Vec<[usize; 3]> {
    let normal = newell_normal(&cut_points(&piece.rings[0], points));

    if normal.magnitude() == 0.0 {
        return Vec::new();
    }

    let plane = Plane::from_point_normal(points[&piece.rings[0][0]].clone(), normal, None);
    let frame = LoftFrame {
        origin: plane.origin(),
        xaxis: plane.x_axis(),
        yaxis: plane.y_axis(),
    };
    let mut rings_2d: Vec<Vec<Point>> = Vec::new();
    let mut flat: Vec<usize> = Vec::new();

    for ring in &piece.rings {
        let mut ring_2d = Vec::with_capacity(ring.len());

        for key in ring {
            let (u, v) = loft_project(&frame, &points[key]);
            ring_2d.push(Point::new(u, v, 0.0));
            flat.push(*key);
        }

        rings_2d.push(ring_2d);
    }

    let mut triangles: Vec<[usize; 3]> = Vec::new();

    for (a, b, c) in remesh_cdt::cdt_triangulate(&rings_2d[0], &rings_2d[1..]) {
        triangles.push([flat[a], flat[b], flat[c]]);
    }

    loft_fix_collinear(&mut triangles, &piece.rings[0]);

    triangles
}

/// Kept pieces of one crossed face from its rings with crossing vertices inserted, split along the plane in the face frame.
fn cut_pieces(
    rings: &[Vec<usize>],
    normal: &Vector,
    xaxis: &Vector,
    distance: &BTreeMap<usize, f64>,
    points: &BTreeMap<usize, Point>,
    tolerance: f64,
) -> Vec<CutFace> {
    let frame = LoftFrame {
        origin: points[&rings[0][0]].clone(),
        xaxis: xaxis.clone(),
        yaxis: normal.cross(xaxis),
    };
    let mut uv: BTreeMap<usize, (f64, f64)> = BTreeMap::new();
    let mut edges: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    let mut lines: BTreeSet<(usize, usize)> = BTreeSet::new();
    let mut events: BTreeSet<usize> = BTreeSet::new();

    for ring in rings {
        for key in ring {
            uv.insert(*key, loft_project(&frame, &points[key]));
        }
    }

    for ring in rings {
        for i in 0..ring.len() {
            let a = ring[i];
            let b = ring[(i + 1) % ring.len()];

            if distance[&a] == 0.0 {
                events.insert(a);
            }

            if distance[&a] == 0.0 && distance[&b] == 0.0 {
                lines.insert((a.min(b), a.max(b)));
            }

            if distance[&a] >= 0.0
                && distance[&b] >= 0.0
                && (distance[&a] + distance[&b] > 0.0 || uv[&b].1 < uv[&a].1)
            {
                edges.entry(a).or_default().push(b);
            }
        }
    }

    let mut order: Vec<usize> = events.into_iter().collect();
    order.sort_by(|a, b| {
        uv[a]
            .1
            .partial_cmp(&uv[b].1)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    for pair in order.windows(2) {
        let a = uv[&pair[0]];
        let b = uv[&pair[1]];
        let mid = ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5);
        let line = (pair[0].min(pair[1]), pair[0].max(pair[1]));

        if b.1 - a.1 > tolerance && !lines.contains(&line) && cut_inside(mid, rings, &uv) {
            edges.entry(pair[1]).or_default().push(pair[0]);
        }
    }

    cut_regions(&cut_loops(edges, &uv), &uv)
}

/// Key of the vertex where edge crosses the plane, added on first use as first plus the number of crossings so far.
fn cut_crossing(
    edge: (usize, usize),
    crossings: &mut BTreeMap<(usize, usize), usize>,
    distance: &mut BTreeMap<usize, f64>,
    points: &mut BTreeMap<usize, Point>,
    first: usize,
) -> usize {
    let edge = (edge.0.min(edge.1), edge.0.max(edge.1));

    if let Some(key) = crossings.get(&edge) {
        return *key;
    }

    let key = first + crossings.len();
    let t = distance[&edge.0] / (distance[&edge.0] - distance[&edge.1]);
    let point = &points[&edge.0] + &((&points[&edge.1] - &points[&edge.0]) * t);
    points.insert(key, point);
    distance.insert(key, 0.0);
    crossings.insert(edge, key);

    key
}

/// Caps closing the loops of unpaired half-edges that lie on the plane.
fn cut_caps(
    faces: &BTreeMap<usize, CutFace>,
    distance: &BTreeMap<usize, f64>,
    points: &BTreeMap<usize, Point>,
    plane: &Plane,
) -> Vec<CutFace> {
    let frame = LoftFrame {
        origin: plane.origin(),
        xaxis: plane.y_axis(),
        yaxis: plane.x_axis(),
    };
    let mut halfedges: BTreeSet<(usize, usize)> = BTreeSet::new();
    let mut section: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    let mut uv: BTreeMap<usize, (f64, f64)> = BTreeMap::new();

    for piece in faces.values() {
        for ring in &piece.rings {
            for i in 0..ring.len() {
                halfedges.insert((ring[i], ring[(i + 1) % ring.len()]));
            }
        }
    }

    for &(a, b) in &halfedges {
        if distance[&a] == 0.0 && distance[&b] == 0.0 && !halfedges.contains(&(b, a)) {
            section.entry(b).or_default().push(a);
        }
    }

    for (key, d) in distance {
        if *d == 0.0 {
            uv.insert(*key, loft_project(&frame, &points[key]));
        }
    }

    cut_regions(&cut_loops(section, &uv), &uv)
}

impl Mesh {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty mesh.
    pub fn new() -> Self {
        let mut default_vertex_attributes = HashMap::new();
        default_vertex_attributes.insert("x".to_string(), 0.0);
        default_vertex_attributes.insert("y".to_string(), 0.0);
        default_vertex_attributes.insert("z".to_string(), 0.0);

        Mesh {
            halfedge: HashMap::new(),
            vertex: HashMap::new(),
            face: HashMap::new(),
            face_holes: HashMap::new(),
            facedata: HashMap::new(),
            edgedata: HashMap::new(),
            default_vertex_attributes,
            default_face_attributes: HashMap::new(),
            default_edge_attributes: HashMap::new(),
            guid: std::sync::OnceLock::new(),
            name: "my_mesh".to_string(),
            color_mode: ColorMode::OBJECTCOLOR,
            pointcolors: Vec::new(),
            facecolors: Vec::new(),
            linecolors: Vec::new(),
            widths: Vec::new(),
            objectcolor: Color::lightgrey(),
            max_vertex: 0,
            max_face: 0,
            triangulation: HashMap::new(),
            triangle_bvh_built: false,
            tri_bvh: None,
            tri_aabbs: Vec::new(),
            tri_tris: Vec::new(),
            tri_face_subidx: Vec::new(),
            tri_vertices: Vec::new(),
            tri_aabb_tree: None,
            gpu_cache: crate::render_mesh::GpuCache::default(),
        }
    }

    /// Copy with a new guid and the same data.
    pub fn duplicate(&self) -> Self {
        let mut m = self.clone();
        m.refresh_guid();

        m
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from a list of vertices and faces.
    pub fn from_vertices_and_faces(vertices: Vec<Point>, faces: Vec<Vec<usize>>) -> Self {
        let mut mesh = Mesh::new();

        for pt in vertices {
            mesh.add_vertex(pt, None);
        }

        for f in faces {
            mesh.add_face(f, None);
        }

        mesh
    }

    /// Vertex key of p, merged by precision grid when given and by exact bits otherwise.
    fn polylines_vertex_key(
        mesh: &mut Mesh,
        p: &Point,
        precision: Option<f64>,
        map_eps: &mut HashMap<(i64, i64, i64), usize>,
        map_exact: &mut HashMap<(u64, u64, u64), usize>,
    ) -> usize {
        if let Some(eps) = precision {
            let key = (
                (p[0] / eps).round() as i64,
                (p[1] / eps).round() as i64,
                (p[2] / eps).round() as i64,
            );

            if let Some(&vk) = map_eps.get(&key) {
                return vk;
            }

            let vk = mesh.add_vertex(p.clone(), None);
            map_eps.insert(key, vk);

            return vk;
        }

        let key = (p[0].to_bits(), p[1].to_bits(), p[2].to_bits());

        if let Some(&vk) = map_exact.get(&key) {
            return vk;
        }

        let vk = mesh.add_vertex(p.clone(), None);
        map_exact.insert(key, vk);

        vk
    }

    /// Construct from a list of polygons, merging vertices within precision when given.
    pub fn from_polylines(polygons: Vec<Vec<Point>>, precision: Option<f64>) -> Self {
        let mut mesh = Mesh::new();
        let mut map_eps: HashMap<(i64, i64, i64), usize> = HashMap::new();
        let mut map_exact: HashMap<(u64, u64, u64), usize> = HashMap::new();

        for poly in &polygons {
            if poly.len() < 3 {
                continue;
            }

            let mut vkeys: Vec<usize> = Vec::with_capacity(poly.len());

            for p in poly {
                vkeys.push(Mesh::polylines_vertex_key(
                    &mut mesh,
                    p,
                    precision,
                    &mut map_eps,
                    &mut map_exact,
                ));
            }

            if vkeys.len() > 1 && vkeys[vkeys.len() - 1] == vkeys[0] {
                vkeys.pop();
            }

            if vkeys.len() < 3 {
                continue;
            }

            let Some(fk) = mesh.add_face(vkeys.clone(), None) else {
                continue;
            };

            if vkeys.len() < 4 {
                continue;
            }

            let tris = planar_cdt(&poly[..vkeys.len()]);

            if tris.is_empty() {
                continue;
            }

            let mut tri_list: Vec<[usize; 3]> = Vec::with_capacity(tris.len());

            for &(a, b, c) in &tris {
                tri_list.push([vkeys[a], vkeys[b], vkeys[c]]);
            }

            mesh.triangulation.insert(fk, tri_list);
        }

        mesh
    }

    /// Construct from a list of polygons, merging vertices within precision when given.
    pub fn from_polylines_polyline(polylines: &[Polyline], precision: Option<f64>) -> Self {
        let mut polygons: Vec<Vec<Point>> = Vec::with_capacity(polylines.len());

        for polyline in polylines {
            polygons.push(polyline.get_points());
        }

        Self::from_polylines(polygons, precision)
    }

    /// Grid spacing for merging line endpoints: the given precision or a millionth of the bbox diagonal.
    fn lines_precision(pts: &[Point], precision: Option<f64>) -> f64 {
        let eps = precision.unwrap_or(0.0);

        if eps > 0.0 {
            return eps;
        }

        let mut minx = pts[0][0];
        let mut miny = pts[0][1];
        let mut minz = pts[0][2];
        let mut maxx = minx;
        let mut maxy = miny;
        let mut maxz = minz;

        for p in pts {
            minx = minx.min(p[0]);
            maxx = maxx.max(p[0]);
            miny = miny.min(p[1]);
            maxy = maxy.max(p[1]);
            minz = minz.min(p[2]);
            maxz = maxz.max(p[2]);
        }

        let diag = ((maxx - minx).powi(2) + (maxy - miny).powi(2) + (maxz - minz).powi(2)).sqrt();

        (diag * 1e-6).max(1e-12)
    }

    /// Index of p in verts, appending it when its grid cell is new.
    fn lines_vertex_id(
        p: &Point,
        eps: f64,
        vmap: &mut HashMap<(i64, i64, i64), usize>,
        verts: &mut Vec<Point>,
    ) -> usize {
        let key = (
            (p[0] / eps).round() as i64,
            (p[1] / eps).round() as i64,
            (p[2] / eps).round() as i64,
        );

        if let Some(&id) = vmap.get(&key) {
            return id;
        }

        let id = verts.len();
        verts.push(p.clone());
        vmap.insert(key, id);

        id
    }

    /// Face cycles of a planar graph: from u->v the next edge turns to the CW predecessor of u around v.
    fn lines_face_cycles(adj: &HashMap<usize, Vec<usize>>, nv: usize) -> Vec<Vec<usize>> {
        let mut visited: HashSet<(usize, usize)> = HashSet::new();
        let mut cycles: Vec<Vec<usize>> = Vec::new();
        let mut adj_keys: Vec<usize> = adj.keys().copied().collect();
        adj_keys.sort();

        for &u in &adj_keys {
            for &v in &adj[&u] {
                if visited.contains(&(u, v)) {
                    continue;
                }

                let mut cycle: Vec<usize> = Vec::new();
                let mut cu = u;
                let mut cv = v;
                let mut valid = true;

                while cycle.len() <= nv * 2 {
                    if visited.contains(&(cu, cv)) {
                        break;
                    }

                    visited.insert((cu, cv));
                    cycle.push(cu);
                    let Some(cv_nbrs) = adj.get(&cv) else {
                        valid = false;
                        break;
                    };
                    let Some(idx) = cv_nbrs.iter().position(|&x| x == cu) else {
                        valid = false;
                        break;
                    };
                    let prev_idx = if idx == 0 { cv_nbrs.len() - 1 } else { idx - 1 };
                    cu = cv;
                    cv = cv_nbrs[prev_idx];
                }

                if cycle.len() > nv * 2 {
                    valid = false;
                }

                if valid && cycle.len() >= 3 {
                    cycles.push(cycle);
                }
            }
        }

        cycles
    }

    /// Index of the cycle with the most negative signed area: the outer boundary.
    fn lines_outer_cycle(cycles: &[Vec<usize>], verts: &[Point]) -> usize {
        let mut min_idx = 0;
        let mut min_area = f64::MAX;

        for (i, cycle) in cycles.iter().enumerate() {
            let mut pts = Vec::with_capacity(cycle.len());

            for &vid in cycle {
                pts.push((verts[vid][0], verts[vid][1]));
            }

            let area = signed_area_2d(&pts) * 0.5;

            if area < min_area {
                min_area = area;
                min_idx = i;
            }
        }

        min_idx
    }

    /// Construct a planar mesh from a line network, optionally without its outer boundary face.
    pub fn from_lines(lines: &[Line], delete_boundary_face: bool, precision: Option<f64>) -> Self {
        if lines.is_empty() {
            return Mesh::new();
        }

        let mut all_pts: Vec<Point> = Vec::with_capacity(lines.len() * 2);

        for ln in lines {
            all_pts.push(ln.start());
            all_pts.push(ln.end());
        }

        let eps = Mesh::lines_precision(&all_pts, precision);
        let mut vmap: HashMap<(i64, i64, i64), usize> = HashMap::new();
        let mut verts: Vec<Point> = Vec::new();
        let mut adj: HashMap<usize, Vec<usize>> = HashMap::new();

        for ln in lines {
            let a = Mesh::lines_vertex_id(&ln.start(), eps, &mut vmap, &mut verts);
            let b = Mesh::lines_vertex_id(&ln.end(), eps, &mut vmap, &mut verts);

            if a == b {
                continue;
            }

            adj.entry(a).or_default().push(b);
            adj.entry(b).or_default().push(a);
        }

        for (v, nbrs) in adj.iter_mut() {
            nbrs.sort();
            nbrs.dedup();
            let vx = verts[*v][0];
            let vy = verts[*v][1];
            nbrs.sort_by(|&a, &b| {
                let aa = (verts[a][1] - vy).atan2(verts[a][0] - vx);
                let ba = (verts[b][1] - vy).atan2(verts[b][0] - vx);
                aa.partial_cmp(&ba).unwrap_or(std::cmp::Ordering::Equal)
            });
        }

        let mut cycles = Mesh::lines_face_cycles(&adj, verts.len());

        if delete_boundary_face && !cycles.is_empty() {
            let outer = Mesh::lines_outer_cycle(&cycles, &verts);
            cycles.remove(outer);
        }

        let mut mesh = Mesh::new();
        let mut vkeys: Vec<usize> = Vec::with_capacity(verts.len());

        for pt in &verts {
            vkeys.push(mesh.add_vertex(pt.clone(), None));
        }

        for cycle in &cycles {
            let mut fvkeys: Vec<usize> = Vec::with_capacity(cycle.len());

            for &vid in cycle {
                fvkeys.push(vkeys[vid]);
            }

            let Some(fk) = mesh.add_face(fvkeys, None) else {
                continue;
            };
            let mut ordered: Vec<usize> = cycle.clone();
            let mut bpts: Vec<(f64, f64)> = Vec::with_capacity(ordered.len());

            for &vid in &ordered {
                bpts.push((verts[vid][0], verts[vid][1]));
            }

            if signed_area_2d(&bpts) < 0.0 {
                bpts.reverse();
                ordered.reverse();
            }

            let mut bpts2d: Vec<Point> = Vec::with_capacity(bpts.len());

            for b in &bpts {
                bpts2d.push(Point::new(b.0, b.1, 0.0));
            }

            let tris = remesh_cdt::cdt_triangulate(&bpts2d, &[]);
            let mut tri_list: Vec<[usize; 3]> = Vec::with_capacity(tris.len());

            for &(a, b, c) in &tris {
                tri_list.push([vkeys[ordered[a]], vkeys[ordered[b]], vkeys[ordered[c]]]);
            }

            mesh.triangulation.insert(fk, tri_list);
        }

        mesh
    }

    /// Construct from a polygon boundary with optional holes; sort_by_bbox picks the largest polyline as boundary.
    pub fn from_polygon_with_holes(polylines: &[Vec<Point>], sort_by_bbox: bool) -> Self {
        if polylines.is_empty() {
            return Mesh::new();
        }

        let mut pls: Vec<Polyline> = Vec::with_capacity(polylines.len());

        for v in polylines {
            pls.push(Polyline::new(v.clone()));
        }

        crate::remesh_cdt::RemeshCDT::from_polylines(&pls, false, !sort_by_bbox)
    }

    /// Construct a loft between two sets of polylines into a mesh volume, capped when cap is true.
    pub fn loft(
        polylines0: &[Polyline],
        polylines1: &[Polyline],
        cap: bool,
        fix_collinear: bool,
    ) -> Self {
        if polylines0.is_empty() || polylines1.is_empty() {
            return Mesh::new();
        }

        if polylines0.len() != polylines1.len() {
            return Mesh::new();
        }

        let border_idx = loft_border_index(polylines0);
        let frame = loft_frame(&polylines0[border_idx], &polylines1[border_idx]);
        let mut order: Vec<usize> = vec![border_idx];

        for i in 0..polylines0.len() {
            if i != border_idx {
                order.push(i);
            }
        }

        let mut polys: Vec<LoftPoly> = Vec::new();
        let mut all_bot: Vec<Point> = Vec::new();
        let mut all_top: Vec<Point> = Vec::new();

        for oi in 0..order.len() {
            let mut bot = loft_open_points(&polylines0[order[oi]]);
            let mut top = loft_open_points(&polylines1[order[oi]]);
            let area = loft_signed_area(&frame, &bot);

            if if oi == 0 { area < 0.0 } else { area > 0.0 } {
                bot.reverse();
                top.reverse();
            }

            polys.push(LoftPoly {
                bot: LoftRing {
                    off: all_bot.len(),
                    n: bot.len(),
                },
                top: LoftRing {
                    off: all_top.len(),
                    n: top.len(),
                },
            });
            all_bot.extend(bot);
            all_top.extend(top);
        }

        let mut mesh = Mesh::new();
        let bot_vkeys = loft_add_vkeys(&mut mesh, &all_bot);
        let top_vkeys = loft_add_vkeys(&mut mesh, &all_top);

        if cap {
            let mut bot_rings: Vec<LoftRing> = Vec::new();
            let mut top_rings: Vec<LoftRing> = Vec::new();

            for poly in &polys {
                bot_rings.push(poly.bot);
                top_rings.push(poly.top);
            }

            loft_cap(
                &mut mesh,
                &frame,
                &bot_rings,
                &all_bot,
                &bot_vkeys,
                true,
                fix_collinear,
            );
            loft_cap(
                &mut mesh,
                &frame,
                &top_rings,
                &all_top,
                &top_vkeys,
                false,
                fix_collinear,
            );
        }

        for poly in &polys {
            loft_walls(
                &mut mesh, &frame, poly, &all_bot, &all_top, &bot_vkeys, &top_vkeys,
            );
        }

        mesh
    }

    /// Construct a batch of from_polygon_with_holes, parallel when asked.
    pub fn from_polygon_with_holes_many(
        inputs: Vec<Vec<Vec<Point>>>,
        sort_by_bbox: bool,
        parallel: bool,
    ) -> Vec<Self> {
        if parallel && inputs.len() > 1 {
            use rayon::prelude::*;

            return inputs
                .into_par_iter()
                .map(|input| Mesh::from_polygon_with_holes(&input, sort_by_bbox))
                .collect();
        }

        let mut results = Vec::with_capacity(inputs.len());

        for input in &inputs {
            results.push(Mesh::from_polygon_with_holes(input, sort_by_bbox));
        }

        results
    }

    /// Construct a batch of loft, parallel when asked.
    pub fn loft_many(
        pairs: Vec<(Vec<Polyline>, Vec<Polyline>)>,
        cap: bool,
        parallel: bool,
        fix_collinear: bool,
    ) -> Vec<Self> {
        if parallel && pairs.len() > 1 {
            use rayon::prelude::*;

            return pairs
                .into_par_iter()
                .map(|(p0, p1)| Mesh::loft(&p0, &p1, cap, fix_collinear))
                .collect();
        }

        let mut results = Vec::with_capacity(pairs.len());

        for (p0, p1) in &pairs {
            results.push(Mesh::loft(p0, p1, cap, fix_collinear));
        }

        results
    }

    /// Construct a loft of matched top/bottom polygon pairs into one panel each, with matched quad walls and triangle fill.
    pub fn loft_panels(
        top_polygons: Vec<Vec<Point>>,
        bot_polygons: Vec<Vec<Point>>,
        merge_precision: f64,
        edge_gap: f64,
        edge_match_threshold: f64,
        add_caps: bool,
        skip_triangles: bool,
    ) -> LoftResult {
        let top_mesh = Mesh::from_polylines(top_polygons, Some(merge_precision));
        let bot_mesh = Mesh::from_polylines(bot_polygons, Some(merge_precision));
        let face_match = lp_match_faces(&top_mesh, &bot_mesh);
        let mut panels: Vec<LoftPanel> = Vec::with_capacity(face_match.len());

        for (tfk, bfk) in face_match {
            panels.push(lp_build_panel(
                &top_mesh,
                &bot_mesh,
                tfk,
                bfk,
                edge_gap,
                edge_match_threshold,
                add_caps,
                skip_triangles,
            ));
        }

        let adjacency = lp_adjacency(&panels);
        let top_ordered = lp_ordered_mesh(&panels, true);
        let bot_ordered = lp_ordered_mesh(&panels, false);

        LoftResult {
            panels,
            adjacency,
            top_mesh: top_ordered,
            bot_mesh: bot_ordered,
        }
    }

    /// Construct a closed box centered at the origin: 8 vertices, 6 quads.
    pub fn create_box(x: f64, y: f64, z: f64) -> Self {
        let hx = x * 0.5;
        let hy = y * 0.5;
        let hz = z * 0.5;
        let vertices = vec![
            Point::new(-hx, -hy, -hz),
            Point::new(hx, -hy, -hz),
            Point::new(hx, hy, -hz),
            Point::new(-hx, hy, -hz),
            Point::new(-hx, -hy, hz),
            Point::new(hx, -hy, hz),
            Point::new(hx, hy, hz),
            Point::new(-hx, hy, hz),
        ];
        let faces = vec![
            vec![0, 3, 2, 1],
            vec![4, 5, 6, 7],
            vec![0, 1, 5, 4],
            vec![2, 3, 7, 6],
            vec![0, 4, 7, 3],
            vec![1, 2, 6, 5],
        ];

        Mesh::from_vertices_and_faces(vertices, faces)
    }

    /// Construct a dodecahedron with the given edge length.
    pub fn create_dodecahedron(edge: f64) -> Self {
        let phi = (1.0 + 5.0_f64.sqrt()) / 2.0;
        let ip = 1.0 / phi;
        let s = edge / (2.0 * ip);
        let verts = [
            Point::new(s, s, s),
            Point::new(s, s, -s),
            Point::new(s, -s, s),
            Point::new(s, -s, -s),
            Point::new(-s, s, s),
            Point::new(-s, s, -s),
            Point::new(-s, -s, s),
            Point::new(-s, -s, -s),
            Point::new(0.0, s * ip, s * phi),
            Point::new(0.0, s * ip, -s * phi),
            Point::new(0.0, -s * ip, s * phi),
            Point::new(0.0, -s * ip, -s * phi),
            Point::new(s * ip, s * phi, 0.0),
            Point::new(s * ip, -s * phi, 0.0),
            Point::new(-s * ip, s * phi, 0.0),
            Point::new(-s * ip, -s * phi, 0.0),
            Point::new(s * phi, 0.0, s * ip),
            Point::new(s * phi, 0.0, -s * ip),
            Point::new(-s * phi, 0.0, s * ip),
            Point::new(-s * phi, 0.0, -s * ip),
        ];
        let idx: [[usize; 5]; 12] = [
            [0, 8, 10, 2, 16],
            [0, 16, 17, 1, 12],
            [0, 12, 14, 4, 8],
            [1, 17, 3, 11, 9],
            [1, 9, 5, 14, 12],
            [2, 10, 6, 15, 13],
            [2, 13, 3, 17, 16],
            [3, 13, 15, 7, 11],
            [4, 14, 5, 19, 18],
            [4, 18, 6, 10, 8],
            [5, 9, 11, 7, 19],
            [6, 18, 19, 7, 15],
        ];
        let mut faces: Vec<Vec<Point>> = Vec::with_capacity(12);

        for f in &idx {
            faces.push(vec![
                verts[f[0]].clone(),
                verts[f[1]].clone(),
                verts[f[2]].clone(),
                verts[f[3]].clone(),
                verts[f[4]].clone(),
            ]);
        }

        Mesh::from_polylines(faces, Some(1e-6))
    }

    /// Number of points of a polyline without its closing duplicate.
    fn pairs_open_count(pl: &Polyline) -> usize {
        let n = pl.point_count();

        if n > 1 && pl.is_closed() {
            return n - 1;
        }

        n
    }

    /// Open polyline with every coordinate divided by scale.
    fn pairs_scaled_open(src: &Polyline, scale: f64) -> Polyline {
        let limit = Mesh::pairs_open_count(src);
        let mut pts = Vec::with_capacity(limit);

        for j in 0..limit {
            let p = &src[j];
            pts.push(Point::new(p[0] / scale, p[1] / scale, p[2] / scale));
        }

        Polyline::new(pts)
    }

    /// Construct a closed mesh from interleaved top/bottom polyline pairs [top0, bot0, ...], coordinates divided by scale.
    pub fn from_polyline_pairs(pairs: &[Polyline], scale: f64) -> Self {
        if pairs.is_empty() || !pairs.len().is_multiple_of(2) {
            return Mesh::new();
        }

        for i in (0..pairs.len()).step_by(2) {
            let a = Mesh::pairs_open_count(&pairs[i]);
            let b = Mesh::pairs_open_count(&pairs[i + 1]);

            if a != b || a < 3 {
                return Mesh::new();
            }
        }

        let mut top_polys = Vec::with_capacity(pairs.len() / 2);
        let mut bot_polys = Vec::with_capacity(pairs.len() / 2);

        for i in (0..pairs.len()).step_by(2) {
            top_polys.push(Mesh::pairs_scaled_open(&pairs[i], scale));
            bot_polys.push(Mesh::pairs_scaled_open(&pairs[i + 1], scale));
        }

        Mesh::loft(&top_polys, &bot_polys, true, true)
    }

    /// Write the flat vertex, normal and triangle arrays of a closed mesh from interleaved top/bottom polyline pairs.
    pub fn from_polyline_pairs_vnf(
        pairs: &[Polyline],
        scale: f64,
    ) -> (Vec<f64>, Vec<f64>, Vec<i32>) {
        let mut out_vertices = Vec::new();
        let mut out_normals = Vec::new();
        let mut out_triangles = Vec::new();
        let m = Mesh::from_polyline_pairs(pairs, scale);

        if m.is_empty() {
            return (out_vertices, out_normals, out_triangles);
        }

        let face_nrms = m.face_normals();

        for fk in m.faces() {
            let Some(fpts) = m.face_points(fk) else {
                continue;
            };

            if fpts.len() < 3 {
                continue;
            }

            let nrm = face_nrms
                .get(&fk)
                .cloned()
                .unwrap_or(Vector::new(0.0, 0.0, 1.0));

            for i in 1..fpts.len() - 1 {
                for p in [&fpts[0], &fpts[i], &fpts[i + 1]] {
                    out_triangles.push(out_triangles.len() as i32);
                    out_vertices.push(p[0]);
                    out_vertices.push(p[1]);
                    out_vertices.push(p[2]);
                    out_normals.push(nrm[0]);
                    out_normals.push(nrm[1]);
                    out_normals.push(nrm[2]);
                }
            }
        }

        (out_vertices, out_normals, out_triangles)
    }

    /// Construct a ruled quad mesh by projecting profile onto planes perpendicular to cross_section.
    pub fn reflex_fold(cross_section: &Polyline, profile: &Polyline) -> Self {
        let n_cs = cross_section.point_count();
        let n_p = profile.point_count();
        let mut planes = Vec::with_capacity(n_cs);

        for i in 0..n_cs {
            let mut normal = Vector::new(0.0, 0.0, 1.0);

            if i > 0 && i < n_cs - 1 {
                let ci = &cross_section[i];
                let cp = &cross_section[i - 1];
                let cn = &cross_section[i + 1];
                let v1 = Vector::new(cp[0] - ci[0], cp[1] - ci[1], cp[2] - ci[2]).normalized();
                let v2 = Vector::new(cn[0] - ci[0], cn[1] - ci[1], cn[2] - ci[2]).normalized();
                normal = &v1 + &v2;

                if !normal.normalize_self() {
                    normal = Vector::new(0.0, 0.0, 1.0);
                }
            }

            let origin = Point::new(
                cross_section[i][0],
                cross_section[i][1],
                cross_section[i][2],
            );
            planes.push(Plane::from_point_normal(origin, normal, None));
        }

        let mut all_pts = Vec::with_capacity(n_cs * n_p);

        for j in 0..n_p {
            all_pts.push(Point::new(profile[j][0], profile[j][1], profile[j][2]));
        }

        let mut faces = Vec::new();

        for i in 1..n_cs {
            let po = planes[i].origin();
            let pp = planes[i - 1].origin();
            let n1 = &po - &pp;
            let n2 = planes[i].z_axis();
            let row_start = all_pts.len();

            for j in 0..n_p {
                let pvrt = all_pts[row_start - n_p + j].clone();
                let diff = &po - &pvrt;
                let denom = n2.dot(&n1);
                let t = if denom.abs() > 1e-12 {
                    n2.dot(&diff) / denom
                } else {
                    0.0
                };
                all_pts.push(&pvrt + &n1 * t);
            }

            for j in 0..n_p - 1 {
                let new_j = row_start + j;
                let old_j = row_start - n_p + j;
                faces.push(vec![new_j, old_j, old_j + 1, new_j + 1]);
            }
        }

        Mesh::from_vertices_and_faces(all_pts, faces)
    }

    /// Compute the per-face miter plate contours of a shell: (top_chamfered, bot_chamfered, top_raw, bot_raw, face_normal).
    pub fn miter_contours(
        shell: &Mesh,
        thickness: f64,
        chamfer_bot: f64,
        chamfer_top: f64,
        _flatter: bool,
        chamfer_angle_deg: f64,
    ) -> Vec<MiterContour> {
        let mut result = Vec::new();
        let efm = shell.edge_face_map();

        for fk in shell.faces() {
            let fverts = shell.face_vertices(fk).unwrap();
            let n = fverts.len();
            let Some(pts) = shell.face_points(fk) else {
                continue;
            };

            if pts.len() != n {
                continue;
            }

            let fn_ = newell_normal(&pts);
            let cen = ring_centroid(&pts);
            let planes = miter_planes(shell, &efm, fverts, &pts, &fn_);

            if planes.len() != n {
                continue;
            }

            let mut corner_lines: Vec<Line> = Vec::with_capacity(n);

            for i in 0..n {
                let Some(line) = crate::intersection::plane_plane(&planes[i], &planes[(i + 1) % n])
                else {
                    break;
                };
                corner_lines.push(line);
            }

            if corner_lines.len() != n {
                continue;
            }

            let bot_origin = &cen + &fn_ * 2.0 * thickness;
            let top_contour = miter_contour(
                &corner_lines,
                &Plane::from_point_normal(cen.clone(), fn_.clone(), None),
            );
            let bot_contour = miter_contour(
                &corner_lines,
                &Plane::from_point_normal(bot_origin, fn_.clone(), None),
            );

            if top_contour.len() != n || bot_contour.len() != n {
                continue;
            }

            let top_mask = fold_chamfer_mask(&top_contour, chamfer_angle_deg);
            let bot_mask = fold_chamfer_mask(&bot_contour, chamfer_angle_deg);
            let top_ch = fold_chamfer(&top_contour, chamfer_bot, &top_mask);
            let bot_ch = fold_chamfer(&bot_contour, chamfer_top, &bot_mask);
            result.push((top_ch, bot_ch, top_contour, bot_contour, fn_));
        }

        result
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
        self.guid = std::sync::OnceLock::new();
    }

    /// Store vertex colors and render with them.
    pub fn set_pointcolors(&mut self, colors: Vec<Color>) {
        self.pointcolors = colors;
        self.color_mode = ColorMode::POINTCOLORS;
        self.gpu_cache.0 = None;
    }

    /// Store face colors and render with them.
    pub fn set_facecolors(&mut self, colors: Vec<Color>) {
        self.facecolors = colors;
        self.color_mode = ColorMode::FACECOLORS;
        self.gpu_cache.0 = None;
    }

    /// Store edge colors and, when given, edge widths.
    pub fn set_linecolors(&mut self, colors: Vec<Color>, line_widths: Vec<f64>) {
        self.linecolors = colors;

        if !line_widths.is_empty() {
            self.widths = line_widths;
        }
    }

    /// Store the object color.
    pub fn set_objectcolor(&mut self, color: Color) {
        self.objectcolor = color;
        self.gpu_cache.0 = None;
    }

    /// Drop vertex colors, falling back to the object color when they were active.
    pub fn clear_pointcolors(&mut self) {
        self.pointcolors.clear();

        if self.color_mode == ColorMode::POINTCOLORS {
            self.color_mode = ColorMode::OBJECTCOLOR;
        }
    }

    /// Drop face colors, falling back to the object color when they were active.
    pub fn clear_facecolors(&mut self) {
        self.facecolors.clear();

        if self.color_mode == ColorMode::FACECOLORS {
            self.color_mode = ColorMode::OBJECTCOLOR;
        }
    }

    /// Drop edge colors and widths.
    pub fn clear_linecolors(&mut self) {
        self.linecolors.clear();
        self.widths.clear();
    }

    /// Return the vertex colors.
    pub fn get_pointcolors(&self) -> &[Color] {
        &self.pointcolors
    }

    /// Return the face colors.
    pub fn get_facecolors(&self) -> &[Color] {
        &self.facecolors
    }

    /// Return the edge colors.
    pub fn get_linecolors(&self) -> &[Color] {
        &self.linecolors
    }

    /// Return the edge widths.
    pub fn get_widths(&self) -> &[f64] {
        &self.widths
    }

    /// Return the object color.
    pub fn get_objectcolor(&self) -> &Color {
        &self.objectcolor
    }

    /// Return the cached triangulation per face.
    pub fn get_triangulation(&self) -> &HashMap<usize, Vec<[usize; 3]>> {
        &self.triangulation
    }

    /// Cache the triangles of face fk.
    pub fn set_face_triangulation(&mut self, fk: usize, tris: Vec<[usize; 3]>) {
        self.triangulation.insert(fk, tris);
    }

    /// Return the hole rings per face.
    pub fn get_face_holes(&self) -> &HashMap<usize, Vec<Vec<usize>>> {
        &self.face_holes
    }

    /// Store the hole rings of face fkey.
    pub fn set_face_holes(&mut self, fkey: usize, rings: Vec<Vec<usize>>) {
        self.face_holes.insert(fkey, rings);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Boolean Queries
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the mesh has no vertices.
    pub fn is_empty(&self) -> bool {
        self.vertex.is_empty()
    }

    /// Return whether every face has at least three existing vertices.
    pub fn is_valid(&self) -> bool {
        if self.vertex.is_empty() || self.face.is_empty() {
            return false;
        }

        for vkeys in self.face.values() {
            if vkeys.len() < 3 {
                return false;
            }

            for vk in vkeys {
                if !self.vertex.contains_key(vk) {
                    return false;
                }
            }
        }

        true
    }

    /// Return whether every face edge has a twin face or a declared hole ring.
    pub fn is_closed(&self) -> bool {
        let mut hole_edges: HashSet<(usize, usize)> = HashSet::new();

        for rings in self.face_holes.values() {
            for ring in rings {
                let n = ring.len();

                for i in 0..n {
                    hole_edges.insert((ring[i], ring[(i + 1) % n]));
                    hole_edges.insert((ring[(i + 1) % n], ring[i]));
                }
            }
        }

        let dfe = self.directed_face_edges();

        for &(u, v) in &dfe {
            if !dfe.contains(&(v, u)) && !hole_edges.contains(&(v, u)) {
                return false;
            }
        }

        !self.vertex.is_empty()
    }

    /// Return whether the vertex touches a boundary edge.
    pub fn is_vertex_on_boundary(&self, vertex_key: usize) -> bool {
        let dfe = self.directed_face_edges();

        for &(u, v) in &dfe {
            if !dfe.contains(&(v, u)) && (u == vertex_key || v == vertex_key) {
                return true;
            }
        }

        false
    }

    /// Return whether the edge has a face on one side only.
    pub fn is_edge_on_boundary(&self, u: usize, v: usize) -> bool {
        let dfe = self.directed_face_edges();

        !(dfe.contains(&(u, v)) && dfe.contains(&(v, u)))
    }

    /// Return whether the face has a boundary edge.
    pub fn is_face_on_boundary(&self, face_key: usize) -> bool {
        let Some(fe) = self.face_edges(face_key) else {
            return false;
        };

        for (u, v) in fe {
            if self.is_edge_on_boundary(u, v) {
                return true;
            }
        }

        false
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Attributes
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the vertex count.
    pub fn number_of_vertices(&self) -> usize {
        self.vertex.len()
    }

    /// Return the face count.
    pub fn number_of_faces(&self) -> usize {
        self.face.len()
    }

    /// Return the undirected edge count.
    pub fn number_of_edges(&self) -> usize {
        let dfe = self.directed_face_edges();
        let mut count = 0;

        for &(u, v) in &dfe {
            if u < v || !dfe.contains(&(v, u)) {
                count += 1;
            }
        }

        count
    }

    /// Return the Euler characteristic V - E + F.
    pub fn euler(&self) -> i32 {
        self.number_of_vertices() as i32 - self.number_of_edges() as i32
            + self.number_of_faces() as i32
    }

    /// Return the sorted vertex keys.
    pub fn vertices(&self) -> Vec<usize> {
        let mut keys: Vec<usize> = self.vertex.keys().copied().collect();
        keys.sort();

        keys
    }

    /// Return the sorted face keys.
    pub fn faces(&self) -> Vec<usize> {
        let mut keys: Vec<usize> = self.face.keys().copied().collect();
        keys.sort();

        keys
    }

    /// Return the undirected edges as sorted (u, v) pairs.
    pub fn edges(&self) -> Vec<(usize, usize)> {
        let mut seen: HashSet<(usize, usize)> = HashSet::new();

        for &(u, v) in &self.directed_face_edges() {
            seen.insert((u.min(v), u.max(v)));
        }

        let mut result: Vec<(usize, usize)> = seen.into_iter().collect();
        result.sort();

        result
    }

    /// Return the vertices and faces with sequential 0-based indices.
    pub fn to_vertices_and_faces(&self) -> (Vec<Point>, Vec<Vec<usize>>) {
        let vertex_idx = self.vertex_index();
        let mut vertices: Vec<Point> = vec![Point::default(); self.vertex.len()];

        for (&key, vdata) in &self.vertex {
            vertices[vertex_idx[&key]] = vdata.position();
        }

        let mut faces = Vec::with_capacity(self.face.len());

        for key in self.faces() {
            let mut remapped = Vec::with_capacity(self.face[&key].len());

            for v in &self.face[&key] {
                remapped.push(vertex_idx[v]);
            }

            faces.push(remapped);
        }

        (vertices, faces)
    }

    /// Return the map from sparse vertex key to sequential index.
    pub fn vertex_index(&self) -> HashMap<usize, usize> {
        let mut index_map = HashMap::with_capacity(self.vertex.len());

        for (index, key) in self.vertices().into_iter().enumerate() {
            index_map.insert(key, index);
        }

        index_map
    }

    /// Return the boundary (true) or interior (false) edges.
    pub fn naked_edges(&self, boundary: bool) -> Vec<(usize, usize)> {
        let dfe = self.directed_face_edges();
        let mut seen: HashSet<(usize, usize)> = HashSet::new();

        for &(u, v) in &dfe {
            seen.insert((u.min(v), u.max(v)));
        }

        let mut sorted: Vec<(usize, usize)> = seen.into_iter().collect();
        sorted.sort();
        let mut result = Vec::new();

        for (u, v) in sorted {
            let naked = !(dfe.contains(&(u, v)) && dfe.contains(&(v, u)));

            if naked == boundary {
                result.push((u, v));
            }
        }

        result
    }

    /// Return the boundary (true) or interior (false) vertices.
    pub fn naked_vertices(&self, boundary: bool) -> Vec<usize> {
        let mut result = Vec::new();

        for vk in self.vertices() {
            if self.is_vertex_on_boundary(vk) == boundary {
                result.push(vk);
            }
        }

        result
    }

    /// Return the boundary (true) or interior (false) faces.
    pub fn naked_faces(&self, boundary: bool) -> Vec<usize> {
        let mut result = Vec::new();

        for fk in self.faces() {
            if self.is_face_on_boundary(fk) == boundary {
                result.push(fk);
            }
        }

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Vertex and Face Operations
    // ═══════════════════════════════════════════════════════════════════════════
    /// Add a vertex, with an explicit key when given; returns the key.
    pub fn add_vertex(&mut self, position: Point, vkey: Option<usize>) -> usize {
        self.ensure_halfedges();
        let vertex_key = vkey.unwrap_or(self.max_vertex);

        if vertex_key >= self.max_vertex {
            self.max_vertex = vertex_key + 1;
        }

        self.vertex.insert(vertex_key, VertexData::new(position));
        self.halfedge.entry(vertex_key).or_default();
        self.pointcolors.push(Color::white());
        self.clear_triangle_bvh();

        vertex_key
    }

    /// Add a face, with an explicit key when given; returns the key or nullopt when invalid.
    pub fn add_face(&mut self, vertices: Vec<usize>, fkey: Option<usize>) -> Option<usize> {
        self.ensure_halfedges();

        if vertices.len() < 3 {
            return None;
        }

        for v in &vertices {
            if !self.vertex.contains_key(v) {
                return None;
            }
        }

        let unique_vertices: HashSet<usize> = vertices.iter().copied().collect();

        if unique_vertices.len() != vertices.len() {
            return None;
        }

        let face_key = fkey.unwrap_or(self.max_face);

        if face_key >= self.max_face {
            self.max_face = face_key + 1;
        }

        self.face.insert(face_key, vertices.clone());
        self.triangulation.remove(&face_key);
        self.facecolors.push(Color::white());

        for i in 0..vertices.len() {
            let u = vertices[i];
            let v = vertices[(i + 1) % vertices.len()];
            let is_new_edge = !self.halfedge.entry(v).or_default().contains_key(&u);
            self.halfedge
                .entry(u)
                .or_default()
                .insert(v, Some(face_key));

            if is_new_edge {
                self.halfedge.entry(v).or_default().insert(u, None);
                self.linecolors.push(Color::black());
                self.widths.push(1.0);
            }
        }

        self.clear_triangle_bvh();

        Some(face_key)
    }

    /// Remove a vertex and every face that uses it.
    pub fn remove_vertex(&mut self, vkey: usize) {
        self.ensure_halfedges();

        if !self.vertex.contains_key(&vkey) {
            return;
        }

        let mut faces_to_remove: Vec<usize> = Vec::new();

        for (&fk, verts) in &self.face {
            if verts.contains(&vkey) {
                faces_to_remove.push(fk);
            }
        }

        for fk in faces_to_remove {
            self.remove_face(fk);
        }

        if let Some(nbrs) = self.halfedge.remove(&vkey) {
            for (v, _) in nbrs {
                if let Some(m) = self.halfedge.get_mut(&v) {
                    m.remove(&vkey);
                }
            }
        }

        self.edgedata.retain(|k, _| k.0 != vkey && k.1 != vkey);
        self.vertex.remove(&vkey);

        if self.pointcolors.len() > self.vertex.len() {
            self.pointcolors.truncate(self.vertex.len());
        }

        self.clear_triangle_bvh();
    }

    /// Remove a face and its orphaned halfedges.
    pub fn remove_face(&mut self, fkey: usize) {
        self.ensure_halfedges();
        let Some(verts) = self.face.get(&fkey).cloned() else {
            return;
        };
        let n = verts.len();

        for i in 0..n {
            let u = verts[i];
            let v = verts[(i + 1) % n];
            let Some(uv) = self.halfedge.get_mut(&u) else {
                continue;
            };

            if !uv.contains_key(&v) {
                continue;
            }

            uv.insert(v, None);
            let vu_none = self
                .halfedge
                .get(&v)
                .and_then(|m| m.get(&u))
                .map(|f| f.is_none())
                .unwrap_or(false);

            if vu_none {
                self.halfedge.get_mut(&u).unwrap().remove(&v);
                self.halfedge.get_mut(&v).unwrap().remove(&u);
            }
        }

        self.face.remove(&fkey);
        self.triangulation.remove(&fkey);
        self.facedata.remove(&fkey);
        self.face_holes.remove(&fkey);
        let n_edges = self.number_of_edges();

        if self.linecolors.len() > n_edges {
            self.linecolors.truncate(n_edges);
        }

        if self.widths.len() > n_edges {
            self.widths.truncate(n_edges);
        }

        if self.facecolors.len() > self.face.len() {
            self.facecolors.truncate(self.face.len());
        }

        self.clear_triangle_bvh();
    }

    /// Remove an edge, its adjacent faces and its halfedges.
    pub fn remove_edge(&mut self, u: usize, v: usize) {
        self.ensure_halfedges();
        let mut faces_to_remove: Vec<usize> = Vec::new();
        let f_uv = self.halfedge_face((u, v));
        let f_vu = self.halfedge_face((v, u));

        if let Some(f) = f_uv {
            faces_to_remove.push(f);
        }

        if let Some(f) = f_vu {
            if f_vu != f_uv {
                faces_to_remove.push(f);
            }
        }

        for fk in faces_to_remove {
            self.remove_face(fk);
        }

        if let Some(nbrs) = self.halfedge.get_mut(&u) {
            nbrs.remove(&v);
        }

        if let Some(nbrs) = self.halfedge.get_mut(&v) {
            nbrs.remove(&u);
        }

        self.edgedata.remove(&(u, v));
        self.edgedata.remove(&(v, u));
        let n_edges = self.number_of_edges();

        if self.linecolors.len() > n_edges {
            self.linecolors.truncate(n_edges);
        }

        if self.widths.len() > n_edges {
            self.widths.truncate(n_edges);
        }

        self.clear_triangle_bvh();
    }

    /// Reverse the winding of one face in place.
    pub fn flip_face(&mut self, fkey: usize) {
        self.ensure_halfedges();
        let Some(mut fv) = self.face.get(&fkey).cloned() else {
            return;
        };
        self.remove_face(fkey);
        fv.reverse();
        self.add_face(fv, Some(fkey));
    }

    /// Reverse the winding of every face.
    pub fn flip(&mut self) {
        for verts in self.face.values_mut() {
            verts.reverse();
        }

        self.rebuild_halfedges();
    }

    /// Clear all mesh data.
    pub fn clear(&mut self) {
        self.halfedge.clear();
        self.vertex.clear();
        self.face.clear();
        self.facedata.clear();
        self.edgedata.clear();
        self.triangulation.clear();
        self.face_holes.clear();
        self.max_vertex = 0;
        self.max_face = 0;
        self.pointcolors.clear();
        self.facecolors.clear();
        self.linecolors.clear();
        self.widths.clear();
        self.objectcolor = Color::lightgrey();
        self.color_mode = ColorMode::OBJECTCOLOR;
        self.clear_triangle_bvh();
    }

    /// Copy where every face owns its own vertices.
    pub fn unweld(&self) -> Mesh {
        let mut m = Mesh::new();

        for fkey in self.faces() {
            let mut new_vkeys = Vec::new();

            for &vk in &self.face[&fkey] {
                new_vkeys.push(m.add_vertex(self.vertex[&vk].position(), None));
            }

            m.add_face(new_vkeys, None);
        }

        m
    }

    /// Root of x in a union-find forest, halving the path on the way.
    fn weld_find(parent: &mut [usize], mut x: usize) -> usize {
        let bound = parent.len();

        for _step in 0..bound {
            if parent[x] == x {
                break;
            }

            parent[x] = parent[parent[x]];
            x = parent[x];
        }

        x
    }

    /// Copy with vertices closer than tolerance merged; degenerate faces are dropped.
    pub fn weld(&self, tolerance: f64) -> Mesh {
        if self.vertex.is_empty() {
            return Mesh::new();
        }

        let vkeys = self.vertices();
        let mut positions: Vec<Point> = Vec::with_capacity(vkeys.len());

        for &vk in &vkeys {
            positions.push(self.vertex[&vk].position());
        }

        let n = vkeys.len();
        let mut parent: Vec<usize> = (0..n).collect();

        if tolerance > 0.0 {
            let mut boxes: Vec<OBB> = Vec::with_capacity(n);

            for p in &positions {
                boxes.push(OBB::from_point(p, tolerance));
            }

            let ws = SpatialBVH::compute_world_size(&boxes);
            let bvh = SpatialBVH::from_boxes(&boxes, ws);
            let (pairs, _, _) = bvh.check_all_collisions(&boxes);

            for (i, j) in pairs {
                if positions[i].distance(&positions[j], None) > tolerance {
                    continue;
                }

                let ri = Mesh::weld_find(&mut parent, i);
                let rj = Mesh::weld_find(&mut parent, j);

                if ri != rj {
                    parent[ri] = rj;
                }
            }
        }

        let mut root_to_rep: HashMap<usize, usize> = HashMap::new();

        for (i, &vk) in vkeys.iter().enumerate() {
            let root = Mesh::weld_find(&mut parent, i);
            let entry = root_to_rep.entry(root).or_insert(vk);

            if vk < *entry {
                *entry = vk;
            }
        }

        let mut vkey_to_rep: HashMap<usize, usize> = HashMap::new();

        for (i, &vk) in vkeys.iter().enumerate() {
            let root = Mesh::weld_find(&mut parent, i);
            vkey_to_rep.insert(vk, root_to_rep[&root]);
        }

        let mut m = Mesh::new();
        let mut added: HashSet<usize> = HashSet::new();

        for &vk in &vkeys {
            let rep = vkey_to_rep[&vk];

            if added.insert(rep) {
                m.add_vertex(self.vertex[&rep].position(), Some(rep));
            }
        }

        for fk in self.faces() {
            let mut new_vkeys = Vec::new();

            for vk in &self.face[&fk] {
                new_vkeys.push(vkey_to_rep[vk]);
            }

            m.add_face(new_vkeys, Some(fk));
        }

        m
    }

    /// Unify face winding by BFS; returns true when any face was flipped.
    pub fn unify_winding(&mut self) -> bool {
        if self.face.len() < 2 {
            return false;
        }

        let mut edge_faces: EdgeFaces = HashMap::new();

        for fkey in self.faces() {
            let verts = &self.face[&fkey];
            let n = verts.len();

            for i in 0..n {
                let u = verts[i];
                let v = verts[(i + 1) % n];
                edge_faces
                    .entry((u.min(v), u.max(v)))
                    .or_default()
                    .push((fkey, u, v));
            }
        }

        let mut visited: HashSet<usize> = HashSet::new();
        let mut flipped: HashSet<usize> = HashSet::new();

        for seed in self.faces() {
            if visited.contains(&seed) {
                continue;
            }

            visited.insert(seed);
            let mut queue = vec![seed];

            while let Some(f) = queue.pop() {
                let is_flipped = flipped.contains(&f);
                let verts = self.face[&f].clone();
                let n = verts.len();

                for i in 0..n {
                    let u_orig = verts[i];
                    let v_orig = verts[(i + 1) % n];
                    let (eff_u, eff_v) = if is_flipped {
                        (v_orig, u_orig)
                    } else {
                        (u_orig, v_orig)
                    };
                    let Some(adj_list) = edge_faces.get(&(u_orig.min(v_orig), u_orig.max(v_orig)))
                    else {
                        continue;
                    };

                    for &(adj_key, adj_u, adj_v) in adj_list {
                        if adj_key == f || visited.contains(&adj_key) {
                            continue;
                        }

                        if !(adj_u == eff_v && adj_v == eff_u) {
                            flipped.insert(adj_key);
                        }

                        visited.insert(adj_key);
                        queue.push(adj_key);
                    }
                }
            }
        }

        if flipped.is_empty() {
            return false;
        }

        for &fkey in &flipped {
            self.face.get_mut(&fkey).unwrap().reverse();
        }

        self.rebuild_halfedges();
        self.orient_outward();

        true
    }

    /// Flip a closed mesh whose normals point inward; returns true when flipped.
    pub fn orient_outward(&mut self) -> bool {
        self.ensure_halfedges();

        if self.face.is_empty() || !self.naked_edges(true).is_empty() {
            return false;
        }

        let mut vol = 0.0f64;

        for fk in self.faces() {
            let verts = &self.face[&fk];
            let p0 = self.vertex_point(verts[0]).unwrap();

            for i in 1..verts.len() - 1 {
                let p1 = self.vertex_point(verts[i]).unwrap();
                let p2 = self.vertex_point(verts[i + 1]).unwrap();
                vol += p0[0] * (p1[1] * p2[2] - p1[2] * p2[1])
                    + p0[1] * (p1[2] * p2[0] - p1[0] * p2[2])
                    + p0[2] * (p1[0] * p2[1] - p1[1] * p2[0]);
            }
        }

        if vol >= 0.0 {
            return false;
        }

        for verts in self.face.values_mut() {
            verts.reverse();
        }

        self.rebuild_halfedges();

        true
    }

    /// Recreate halfedge from vertex and face alone.
    pub fn rebuild_halfedges(&mut self) {
        self.halfedge = self.compute_halfedges();
    }

    /// Build the lazy halfedge map when it is empty and faces exist.
    pub fn ensure_halfedges(&mut self) {
        if self.halfedge.is_empty() && !self.face.is_empty() {
            self.rebuild_halfedges();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Connectivity Queries
    // ═══════════════════════════════════════════════════════════════════════════
    /// Sorted neighbors of x over a directed edge set.
    fn edge_ends(dfe: &HashSet<(usize, usize)>, x: usize) -> Vec<usize> {
        let mut keys: HashSet<usize> = HashSet::new();

        for &(a, b) in dfe {
            if a == x {
                keys.insert(b);
            } else if b == x {
                keys.insert(a);
            }
        }

        let mut sorted: Vec<usize> = keys.into_iter().collect();
        sorted.sort();

        sorted
    }

    /// Return the edges sharing a vertex with (u, v), excluding (u, v) and (v, u).
    pub fn edge_edges(&self, u: usize, v: usize) -> Option<Vec<(usize, usize)>> {
        let dfe = self.directed_face_edges();

        if !dfe.contains(&(u, v)) && !dfe.contains(&(v, u)) {
            return None;
        }

        let mut edges = Vec::new();

        for w in Mesh::edge_ends(&dfe, u) {
            if w != v {
                edges.push((u, w));
            }
        }

        for w in Mesh::edge_ends(&dfe, v) {
            if w != u {
                edges.push((v, w));
            }
        }

        Some(edges)
    }

    /// Return the faces on each side of an edge.
    pub fn edge_faces(&self, u: usize, v: usize) -> Option<Vec<usize>> {
        let mut result: Vec<usize> = Vec::new();

        for fkey in self.faces() {
            let verts = &self.face[&fkey];
            let n = verts.len();

            for i in 0..n {
                let a = verts[i];
                let b = verts[(i + 1) % n];

                if !((a == u && b == v) || (a == v && b == u)) {
                    continue;
                }

                if !result.contains(&fkey) {
                    result.push(fkey);
                }
            }
        }

        if result.is_empty() {
            return None;
        }

        Some(result)
    }

    /// Return every directed face edge mapped to its face key, in one face walk.
    pub fn edge_face_map(&self) -> HashMap<(usize, usize), usize> {
        let mut m: HashMap<(usize, usize), usize> = HashMap::with_capacity(self.face.len() * 4);

        for fkey in self.faces() {
            let verts = &self.face[&fkey];
            let n = verts.len();

            for i in 0..n {
                m.insert((verts[i], verts[(i + 1) % n]), fkey);
            }
        }

        m
    }

    /// Return the edge as a Line.
    pub fn edge_line(&self, u: usize, v: usize) -> Option<Line> {
        let dfe = self.directed_face_edges();

        if !dfe.contains(&(u, v)) && !dfe.contains(&(v, u)) {
            return None;
        }

        let pu = self.vertex_point(u)?;
        let pv = self.vertex_point(v)?;

        Some(Line::from_points(&pu, &pv))
    }

    /// Return the edges of a face as (vi, vi+1) pairs.
    pub fn face_edges(&self, face_key: usize) -> Option<Vec<(usize, usize)>> {
        let verts = self.face.get(&face_key)?;
        let n = verts.len();
        let mut edges = Vec::with_capacity(n);

        for i in 0..n {
            edges.push((verts[i], verts[(i + 1) % n]));
        }

        Some(edges)
    }

    /// Return the faces sharing an edge with a face.
    pub fn face_faces(&self, face_key: usize) -> Option<Vec<usize>> {
        let fe = self.face_edges(face_key)?;
        let efm = self.edge_face_map();
        let mut neighbors = Vec::new();

        for (u, v) in fe {
            if let Some(&f) = efm.get(&(v, u)) {
                neighbors.push(f);
            }
        }

        Some(neighbors)
    }

    /// Return the points of a face.
    pub fn face_points(&self, face_key: usize) -> Option<Vec<Point>> {
        let fv = self.face_vertices(face_key)?;
        let mut pts = Vec::with_capacity(fv.len());

        for &vk in fv {
            pts.push(self.vertex_point(vk)?);
        }

        Some(pts)
    }

    /// Return the face as a Polyline.
    pub fn face_polyline(&self, face_key: usize) -> Option<Polyline> {
        Some(Polyline::new(self.face_points(face_key)?))
    }

    /// Return the vertex keys of a face.
    pub fn face_vertices(&self, face_key: usize) -> Option<&Vec<usize>> {
        self.face.get(&face_key)
    }

    /// Return the edges incident to a vertex as (vertex_key, neighbor) pairs.
    pub fn vertex_edges(&self, vertex_key: usize) -> Option<Vec<(usize, usize)>> {
        let keys = self.vertex_vertices(vertex_key)?;
        let mut edges = Vec::with_capacity(keys.len());

        for u in keys {
            edges.push((vertex_key, u));
        }

        Some(edges)
    }

    /// Return the faces incident to a vertex.
    pub fn vertex_faces(&self, vertex_key: usize) -> Option<Vec<usize>> {
        let keys = self.vertex_vertices(vertex_key)?;
        let efm = self.edge_face_map();
        let mut faces = Vec::new();

        for u in keys {
            if let Some(&f) = efm.get(&(vertex_key, u)) {
                faces.push(f);
            }
        }

        Some(faces)
    }

    /// Return the position of a vertex.
    pub fn vertex_point(&self, vertex_key: usize) -> Option<Point> {
        Some(self.vertex.get(&vertex_key)?.position())
    }

    /// Return the neighboring vertices of a vertex.
    pub fn vertex_vertices(&self, vertex_key: usize) -> Option<Vec<usize>> {
        if !self.vertex.contains_key(&vertex_key) {
            return None;
        }

        Some(Mesh::edge_ends(&self.directed_face_edges(), vertex_key))
    }

    /// Return the neighbors of a vertex, in face-cycle order when ordered is true.
    pub fn vertex_neighbors(&self, vertex_key: usize, ordered: bool) -> Option<Vec<usize>> {
        let nbrs_map = self.halfedge.get(&vertex_key)?;
        let mut nbrs: Vec<usize> = nbrs_map.keys().copied().collect();
        nbrs.sort();

        if !ordered || nbrs.len() <= 1 {
            return Some(nbrs);
        }

        let mut start = nbrs[0];

        for &n in &nbrs {
            if nbrs_map[&n].is_none() {
                start = n;
                break;
            }
        }

        let mut fkey = self.halfedge_face((start, vertex_key));
        let mut out = vec![start];

        for _step in 0..self.face.len() {
            let Some(f) = fkey else { break };
            let Some(verts) = self.face.get(&f) else {
                break;
            };
            let Some(i) = verts.iter().position(|&v| v == vertex_key) else {
                break;
            };
            let nbr = verts[(i + 1) % verts.len()];

            if nbr == start {
                break;
            }

            out.push(nbr);
            fkey = self.halfedge_face((nbr, vertex_key));
        }

        Some(out)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Boundary
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the vertices touching a boundary edge.
    pub fn vertices_on_boundary(&self) -> Vec<usize> {
        let mut out = Vec::new();

        for v in self.vertices() {
            if self.is_vertex_on_boundary(v) {
                out.push(v);
            }
        }

        out
    }

    /// Return the edges with a face on one side only.
    pub fn edges_on_boundary(&self) -> Vec<(usize, usize)> {
        let mut out = Vec::new();

        for (u, nbrs) in &self.halfedge {
            for (v, f) in nbrs {
                if f.is_none() {
                    out.push((*u, *v));
                }
            }
        }

        out.sort();

        out
    }

    /// Return the faces with a boundary edge.
    pub fn faces_on_boundary(&self) -> Vec<usize> {
        let mut out = Vec::new();

        for f in self.faces() {
            if self.is_face_on_boundary(f) {
                out.push(f);
            }
        }

        out
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Halfedge Navigation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the face of a directed edge, nullopt when unknown or on the boundary.
    pub fn halfedge_face(&self, edge: (usize, usize)) -> Option<usize> {
        *self.halfedge.get(&edge.0)?.get(&edge.1)?
    }

    /// Return the next directed edge around the face of edge.
    pub fn halfedge_after(&self, edge: (usize, usize)) -> Option<(usize, usize)> {
        let (u, v) = edge;

        if let Some(f) = self.halfedge_face(edge) {
            let verts = self.face.get(&f)?;
            let i = verts.iter().position(|&w| w == v)?;

            return Some((v, verts[(i + 1) % verts.len()]));
        }

        let nbrs = self.halfedge.get(&v)?;
        let mut keys: Vec<usize> = nbrs.keys().copied().collect();
        keys.sort();

        for w in keys {
            if w != u && nbrs[&w].is_none() {
                return Some((v, w));
            }
        }

        None
    }

    /// Return the previous directed edge around the face of edge.
    pub fn halfedge_before(&self, edge: (usize, usize)) -> Option<(usize, usize)> {
        let (u, v) = edge;

        if let Some(f) = self.halfedge_face(edge) {
            let verts = self.face.get(&f)?;
            let n = verts.len();
            let i = verts.iter().position(|&w| w == u)?;

            return Some((verts[(i + n - 1) % n], u));
        }

        let nbrs = self.halfedge.get(&u)?;
        let mut keys: Vec<usize> = nbrs.keys().copied().collect();
        keys.sort();

        for w in keys {
            if w == v {
                continue;
            }

            if self.halfedge.get(&w).and_then(|m| m.get(&u)) == Some(&None) {
                return Some((w, u));
            }
        }

        None
    }

    /// Boundary loop from edge: at each vertex continue along the other boundary edge.
    fn halfedge_loop_boundary(&self, edge: (usize, usize)) -> Vec<(usize, usize)> {
        let mut edges = vec![edge];
        let (mut u, mut v) = edge;

        for _step in 0..self.vertex.len() {
            let Some(nbrs) = self.vertex_neighbors(v, false) else {
                break;
            };

            if nbrs.len() == 2 {
                break;
            }

            let mut nbr = None;

            for temp in nbrs {
                if temp == u {
                    continue;
                }

                if self.is_edge_on_boundary(v, temp) {
                    nbr = Some(temp);
                    break;
                }
            }

            let Some(next) = nbr else { break };
            u = v;
            v = next;
            edges.push((u, v));

            if v == edges[0].0 {
                break;
            }
        }

        edges
    }

    /// Return the directed edges around the face of edge, starting at edge.
    pub fn halfedge_loop(&self, edge: (usize, usize)) -> Vec<(usize, usize)> {
        if self.is_edge_on_boundary(edge.0, edge.1) {
            return self.halfedge_loop_boundary(edge);
        }

        let mut edges = vec![edge];
        let (mut u, mut v) = edge;

        for _step in 0..self.vertex.len() {
            let Some(nbrs) = self.vertex_neighbors(v, true) else {
                break;
            };

            if nbrs.len() != 4 {
                break;
            }

            let Some(i) = nbrs.iter().position(|&w| w == u) else {
                break;
            };
            u = v;
            v = nbrs[(i + 2) % 4];
            edges.push((u, v));

            if v == edges[0].0 {
                break;
            }
        }

        edges
    }

    /// Return the directed edges straight across quads from edge until a boundary or a non-quad.
    pub fn halfedge_strip(&self, edge: (usize, usize)) -> Vec<(usize, usize)> {
        let (mut u, mut v) = edge;
        let mut edges = vec![edge];

        for _step in 0..self.face.len() {
            let Some(f) = self.halfedge_face((u, v)) else {
                break;
            };
            let Some(verts) = self.face.get(&f) else {
                break;
            };

            if verts.len() != 4 {
                break;
            }

            let Some(i) = verts.iter().position(|&w| w == u) else {
                break;
            };
            u = verts[(i + 3) % 4];
            v = verts[(i + 2) % 4];
            edges.push((u, v));

            if (u, v) == edge {
                break;
            }
        }

        edges
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Sampling
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return size keys; seed 0 takes the first keys, any other seed drives a deterministic LCG.
    fn lcg_sample<T: Clone>(keys: &[T], size: usize, seed: u32) -> Vec<T> {
        if keys.is_empty() || size == 0 {
            return Vec::new();
        }

        let n = keys.len();
        let take = size.min(n);

        if seed == 0 {
            return keys[..take].to_vec();
        }

        let mut s = seed & 0x7FFF_FFFF;

        if s == 0 {
            s = 1;
        }

        let mut used = HashSet::new();
        let mut out = Vec::with_capacity(take);

        while out.len() < take {
            s = s.wrapping_mul(1103515245).wrapping_add(12345) & 0x7FFF_FFFF;
            let i = s as usize % n;

            if used.insert(i) {
                out.push(keys[i].clone());
            }
        }

        out
    }

    /// Return size vertex keys; seed 0 takes the first keys, any other seed drives a deterministic LCG.
    pub fn vertex_sample(&self, size: usize, seed: u32) -> Vec<usize> {
        Mesh::lcg_sample(&self.vertices(), size, seed)
    }

    /// Return size edges; seed 0 takes the first edges, any other seed drives a deterministic LCG.
    pub fn edge_sample(&self, size: usize, seed: u32) -> Vec<(usize, usize)> {
        Mesh::lcg_sample(&self.edges(), size, seed)
    }

    /// Return size face keys; seed 0 takes the first keys, any other seed drives a deterministic LCG.
    pub fn face_sample(&self, size: usize, seed: u32) -> Vec<usize> {
        Mesh::lcg_sample(&self.faces(), size, seed)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Aliases
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the average of a face's vertex positions.
    pub fn face_center(&self, face_key: usize) -> Option<Point> {
        self.face_centroid(face_key)
    }

    /// Return the face as a Polyline.
    pub fn face_polygon(&self, face_key: usize) -> Option<Polyline> {
        let mut pts = self.face_points(face_key)?;

        if !pts.is_empty() && pts[0] != pts[pts.len() - 1] {
            pts.push(pts[0].clone());
        }

        Some(Polyline::new(pts))
    }

    /// Return every face as a closed outline in face-key order; faces under three vertices are skipped.
    pub fn face_outlines(&self) -> Vec<Polyline> {
        let mut outlines = Vec::with_capacity(self.face.len());

        for face_key in self.faces() {
            let Some(outline) = self.face_polygon(face_key) else {
                continue;
            };

            if outline.point_count() >= 4 {
                outlines.push(outline);
            }
        }

        outlines
    }

    /// Reverse the winding of every face.
    pub fn flip_cycles(&mut self) {
        self.flip();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Attribute API
    // ═══════════════════════════════════════════════════════════════════════════
    /// Merge attrs into the default vertex attributes.
    pub fn update_default_vertex_attributes(&mut self, attrs: &[(&str, f64)]) {
        for (k, v) in attrs {
            self.default_vertex_attributes.insert(k.to_string(), *v);
        }
    }

    /// Merge attrs into the default face attributes.
    pub fn update_default_face_attributes(&mut self, attrs: &[(&str, f64)]) {
        for (k, v) in attrs {
            self.default_face_attributes.insert(k.to_string(), *v);
        }
    }

    /// Merge attrs into the default edge attributes.
    pub fn update_default_edge_attributes(&mut self, attrs: &[(&str, f64)]) {
        for (k, v) in attrs {
            self.default_edge_attributes.insert(k.to_string(), *v);
        }
    }

    /// Return the attribute of a vertex, falling back to the default; nullopt when neither exists.
    pub fn vertex_attribute(&self, key: usize, name: &str) -> Option<f64> {
        let data = self.vertex.get(&key)?;

        if let Some(v) = data.attributes.get(name) {
            return Some(*v);
        }

        self.default_vertex_attributes.get(name).copied()
    }

    /// Store an attribute on a vertex.
    pub fn set_vertex_attribute(&mut self, key: usize, name: &str, value: f64) {
        let Some(data) = self.vertex.get_mut(&key) else {
            return;
        };
        data.attributes.insert(name.to_string(), value);
    }

    /// Return the attribute of a face, falling back to the default; nullopt when neither exists.
    pub fn face_attribute(&self, fkey: usize, name: &str) -> Option<f64> {
        if !self.face.contains_key(&fkey) {
            return None;
        }

        if let Some(v) = self.facedata.get(&fkey).and_then(|a| a.get(name)) {
            return Some(*v);
        }

        self.default_face_attributes.get(name).copied()
    }

    /// Store an attribute on a face.
    pub fn set_face_attribute(&mut self, fkey: usize, name: &str, value: f64) {
        if !self.face.contains_key(&fkey) {
            return;
        }

        self.facedata
            .entry(fkey)
            .or_default()
            .insert(name.to_string(), value);
    }

    /// Return the attribute of an edge, falling back to the default; nullopt when neither exists.
    pub fn edge_attribute(&self, edge: (usize, usize), name: &str) -> Option<f64> {
        let (u, v) = edge;
        let uv = self.halfedge.get(&u).is_some_and(|m| m.contains_key(&v));
        let vu = self.halfedge.get(&v).is_some_and(|m| m.contains_key(&u));

        if !uv && !vu {
            return None;
        }

        let data = self
            .edgedata
            .get(&(u, v))
            .or_else(|| self.edgedata.get(&(v, u)));

        if let Some(val) = data.and_then(|a| a.get(name)) {
            return Some(*val);
        }

        self.default_edge_attributes.get(name).copied()
    }

    /// Store an attribute on an edge.
    pub fn set_edge_attribute(&mut self, edge: (usize, usize), name: &str, value: f64) {
        let (u, v) = edge;
        let key = if self.edgedata.contains_key(&(v, u)) {
            (v, u)
        } else {
            (u, v)
        };
        self.edgedata
            .entry(key)
            .or_default()
            .insert(name.to_string(), value);
    }

    /// Return the attribute of every vertex in keys; keys nullptr means all; the result holds nullopt for missing values.
    pub fn vertices_attribute(&self, name: &str, keys: Option<&[usize]>) -> Vec<Option<f64>> {
        let all = self.vertices();
        let keys = keys.unwrap_or(&all);
        let mut out = Vec::with_capacity(keys.len());

        for k in keys {
            out.push(self.vertex_attribute(*k, name));
        }

        out
    }

    /// Store an attribute on every vertex in keys; keys nullptr means all.
    pub fn set_vertices_attribute(&mut self, name: &str, value: f64, keys: Option<&[usize]>) {
        let all = self.vertices();
        let keys = keys.unwrap_or(&all);

        for k in keys {
            self.set_vertex_attribute(*k, name, value);
        }
    }

    /// Return the attribute of every face in keys; keys nullptr means all; the result holds nullopt for missing values.
    pub fn faces_attribute(&self, name: &str, keys: Option<&[usize]>) -> Vec<Option<f64>> {
        let all = self.faces();
        let keys = keys.unwrap_or(&all);
        let mut out = Vec::with_capacity(keys.len());

        for k in keys {
            out.push(self.face_attribute(*k, name));
        }

        out
    }

    /// Store an attribute on every face in keys; keys nullptr means all.
    pub fn set_faces_attribute(&mut self, name: &str, value: f64, keys: Option<&[usize]>) {
        let all = self.faces();
        let keys = keys.unwrap_or(&all);

        for k in keys {
            self.set_face_attribute(*k, name, value);
        }
    }

    /// Return the attribute of every edge in keys; keys nullptr means all; the result holds nullopt for missing values.
    pub fn edges_attribute(&self, name: &str, keys: Option<&[(usize, usize)]>) -> Vec<Option<f64>> {
        let all = self.edges();
        let keys = keys.unwrap_or(&all);
        let mut out = Vec::with_capacity(keys.len());

        for e in keys {
            out.push(self.edge_attribute(*e, name));
        }

        out
    }

    /// Store an attribute on every edge in keys; keys nullptr means all.
    pub fn set_edges_attribute(&mut self, name: &str, value: f64, keys: Option<&[(usize, usize)]>) {
        let all = self.edges();
        let keys = keys.unwrap_or(&all);

        for e in keys {
            self.set_edge_attribute(*e, name, value);
        }
    }

    /// Return the vertices whose attributes match every (name, value) condition.
    pub fn vertices_where(&self, conditions: &[(&str, f64)]) -> Vec<usize> {
        let mut out = Vec::new();

        for k in self.vertices() {
            let mut ok = true;

            for (n, v) in conditions {
                let val = self.vertex_attribute(k, n);

                if val.is_none() || val != Some(*v) {
                    ok = false;
                    break;
                }
            }

            if ok {
                out.push(k);
            }
        }

        out
    }

    /// Return the faces whose attributes match every (name, value) condition.
    pub fn faces_where(&self, conditions: &[(&str, f64)]) -> Vec<usize> {
        let mut out = Vec::new();

        for k in self.faces() {
            let mut ok = true;

            for (n, v) in conditions {
                let val = self.face_attribute(k, n);

                if val.is_none() || val != Some(*v) {
                    ok = false;
                    break;
                }
            }

            if ok {
                out.push(k);
            }
        }

        out
    }

    /// Return the edges whose attributes match every (name, value) condition.
    pub fn edges_where(&self, conditions: &[(&str, f64)]) -> Vec<(usize, usize)> {
        let mut out = Vec::new();

        for e in self.edges() {
            let mut ok = true;

            for (n, v) in conditions {
                let val = self.edge_attribute(e, n);

                if val.is_none() || val != Some(*v) {
                    ok = false;
                    break;
                }
            }

            if ok {
                out.push(e);
            }
        }

        out
    }

    /// Return the vertices for which pred(key, attributes) is true.
    pub fn vertices_where_predicate(&self, pred: KeyPredicate<usize>) -> Vec<usize> {
        let mut out = Vec::new();

        for k in self.vertices() {
            let mut attrs = self.default_vertex_attributes.clone();

            for (kk, vv) in self.vertex[&k].attributes.iter() {
                attrs.insert(kk.clone(), *vv);
            }

            if pred(k, &attrs) {
                out.push(k);
            }
        }

        out
    }

    /// Return the faces for which pred(key, attributes) is true.
    pub fn faces_where_predicate(&self, pred: KeyPredicate<usize>) -> Vec<usize> {
        let mut out = Vec::new();

        for k in self.faces() {
            let mut attrs = self.default_face_attributes.clone();

            if let Some(data) = self.facedata.get(&k) {
                for (kk, vv) in data {
                    attrs.insert(kk.clone(), *vv);
                }
            }

            if pred(k, &attrs) {
                out.push(k);
            }
        }

        out
    }

    /// Return the edges for which pred(edge, attributes) is true.
    pub fn edges_where_predicate(&self, pred: KeyPredicate<(usize, usize)>) -> Vec<(usize, usize)> {
        let mut out = Vec::new();

        for e in self.edges() {
            let mut attrs = self.default_edge_attributes.clone();
            let data = self
                .edgedata
                .get(&e)
                .or_else(|| self.edgedata.get(&(e.1, e.0)));

            if let Some(data) = data {
                for (kk, vv) in data {
                    attrs.insert(kk.clone(), *vv);
                }
            }

            if pred(e, &attrs) {
                out.push(e);
            }
        }

        out
    }

    /// Return the face normal from the first three vertices; unitized false keeps twice the first-triangle area as length.
    pub fn face_normal_unitized(&self, face_key: usize, unitized: bool) -> Option<Vector> {
        let vertices = self.face_vertices(face_key)?;

        if vertices.len() < 3 {
            return None;
        }

        let p0 = self.vertex_point(vertices[0])?;
        let p1 = self.vertex_point(vertices[1])?;
        let p2 = self.vertex_point(vertices[2])?;
        let u = &p1 - &p0;
        let v = &p2 - &p0;
        let normal = u.cross(&v);

        if !unitized {
            return Some(normal);
        }

        let len = normal.magnitude();

        if len > Tolerance::ZERO_TOLERANCE {
            return Some(&normal / len);
        }

        None
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometric Properties
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the total surface area of all faces.
    pub fn area(&self) -> f64 {
        let mut total = 0.0;

        for fk in self.faces() {
            if let Some(a) = self.face_area(fk) {
                total += a;
            }
        }

        total
    }

    /// Return the average of all vertex positions.
    pub fn centroid(&self) -> Point {
        let mut x = 0.0;
        let mut y = 0.0;
        let mut z = 0.0;

        for vk in self.vertices() {
            let v = &self.vertex[&vk];
            x += v.x;
            y += v.y;
            z += v.z;
        }

        let n = if self.vertex.is_empty() {
            1.0
        } else {
            self.vertex.len() as f64
        };

        Point::new(x / n, y / n, z / n)
    }

    /// Return the dihedral angle in degrees between the two faces sharing edge (u, v), nullopt on a boundary edge.
    pub fn dihedral_angle(&self, u: usize, v: usize) -> Option<f64> {
        let ef = self.edge_faces(u, v)?;

        if ef.len() < 2 {
            return None;
        }

        let n0 = self.face_normal(ef[0])?;
        let n1 = self.face_normal(ef[1])?;
        let dot = n0.dot(&n1).clamp(-1.0, 1.0);

        Some((PI - dot.acos()) * 180.0 / PI)
    }

    /// Unit direction from the edge midpoint to the face centroid, in the plane perpendicular to the edge.
    fn dihedral_arm(centroid: &Point, mid: &Point, edge: &Vector) -> Option<Vector> {
        let mut d = centroid - mid;
        let dot = d.dot(edge);
        d -= edge * dot;
        let len = d.magnitude();

        if len < 1e-10 {
            return None;
        }

        Some(&d / len)
    }

    /// Return the dihedral angles of all interior edges as (angles, arcs, points); arcs and label points are built when asked.
    pub fn dihedral_angles(
        &self,
        scale: f64,
        with_arcs: bool,
        with_points: bool,
    ) -> DihedralAngles {
        let mut angles: BTreeMap<(usize, usize), f64> = BTreeMap::new();
        let mut arcs: Vec<Polyline> = Vec::new();
        let mut points: Vec<Point> = Vec::new();
        let arc_n: usize = 12;
        let label_color = Color::yellow();

        for (u, v) in self.edges() {
            let Some(da) = self.dihedral_angle(u, v) else {
                continue;
            };
            angles.insert((u, v), da);
            let ep0 = self.vertex_point(u).unwrap();
            let ep1 = self.vertex_point(v).unwrap();
            let mid = Point::new(
                (ep0[0] + ep1[0]) * 0.5,
                (ep0[1] + ep1[1]) * 0.5,
                (ep0[2] + ep1[2]) * 0.5,
            );

            if scale == 0.0 {
                if !with_points {
                    continue;
                }

                let mut pt = Point::new(mid[0], mid[1], mid[2]);
                pt.name = da.to_string();
                pt.pointcolor = label_color.clone();
                points.push(pt);
                continue;
            }

            let ef = self.edge_faces(u, v).unwrap();
            let mut edge = &ep1 - &ep0;

            if edge.magnitude() < 1e-10 || !edge.normalize_self() {
                continue;
            }

            let d0 = Mesh::dihedral_arm(&self.face_centroid(ef[0]).unwrap(), &mid, &edge);
            let d1 = Mesh::dihedral_arm(&self.face_centroid(ef[1]).unwrap(), &mid, &edge);
            let (Some(d0), Some(d1)) = (d0, d1) else {
                continue;
            };
            let theta = d0.dot(&d1).clamp(-1.0, 1.0).acos();

            if theta.sin().abs() < 1e-10 {
                continue;
            }

            let mut arc_pts: Vec<Point> = Vec::with_capacity(arc_n + 1);

            for j in 0..=arc_n {
                let t = j as f64 / arc_n as f64;
                let w1 = ((1.0 - t) * theta).sin() / theta.sin();
                let w2 = (t * theta).sin() / theta.sin();
                arc_pts.push(&mid + (&d0 * w1 + &d1 * w2) * scale);
            }

            if with_arcs {
                let mut arc = Polyline::new(arc_pts.clone());
                arc.name = format!("dihedral_e{}_{}={}", u, v, da);
                arc.linecolor = label_color.clone();
                arcs.push(arc);
            }

            if with_points {
                let mut pt = Point::new(
                    arc_pts[arc_n / 2][0],
                    arc_pts[arc_n / 2][1],
                    arc_pts[arc_n / 2][2],
                );
                pt.name = da.to_string();
                pt.pointcolor = label_color.clone();
                points.push(pt);
            }
        }

        (angles, arcs, points)
    }

    /// Return the area of a face.
    pub fn face_area(&self, face_key: usize) -> Option<f64> {
        let vertices = self.face_vertices(face_key)?;

        if vertices.len() < 3 {
            return Some(0.0);
        }

        let p0 = self.vertex_point(vertices[0])?;
        let mut area = 0.0;

        for i in 1..vertices.len() - 1 {
            let p1 = self.vertex_point(vertices[i])?;
            let p2 = self.vertex_point(vertices[i + 1])?;
            let u = &p1 - &p0;
            let v = &p2 - &p0;
            area += u.cross(&v).magnitude() * 0.5;
        }

        Some(area)
    }

    /// Return the average of a face's vertex positions.
    pub fn face_centroid(&self, face_key: usize) -> Option<Point> {
        let verts = self.face_vertices(face_key)?;

        if verts.is_empty() {
            return None;
        }

        let mut x = 0.0;
        let mut y = 0.0;
        let mut z = 0.0;

        for &vk in verts {
            let p = self.vertex_point(vk)?;
            x += p[0];
            y += p[1];
            z += p[2];
        }

        let n = verts.len() as f64;

        Some(Point::new(x / n, y / n, z / n))
    }

    /// Return the unit normal of a face.
    pub fn face_normal(&self, face_key: usize) -> Option<Vector> {
        self.face_normal_unitized(face_key, true)
    }

    /// Return the unit normals of all faces.
    pub fn face_normals(&self) -> HashMap<usize, Vector> {
        let mut normals = HashMap::new();

        for face_key in self.faces() {
            if let Some(normal) = self.face_normal(face_key) {
                normals.insert(face_key, normal);
            }
        }

        normals
    }

    /// Return the angle at a vertex inside a face.
    pub fn vertex_angle_in_face(&self, vertex_key: usize, face_key: usize) -> Option<f64> {
        let vertices = self.face_vertices(face_key)?;
        let vertex_index = vertices.iter().position(|&v| v == vertex_key)?;
        let n = vertices.len();
        let center = self.vertex_point(vertex_key)?;
        let prev_pos = self.vertex_point(vertices[(vertex_index + n - 1) % n])?;
        let next_pos = self.vertex_point(vertices[(vertex_index + 1) % n])?;
        let u = &prev_pos - &center;
        let v = &next_pos - &center;
        let u_len = u.magnitude();
        let v_len = v.magnitude();

        if u_len < Tolerance::ZERO_TOLERANCE || v_len < Tolerance::ZERO_TOLERANCE {
            return Some(0.0);
        }

        Some((u.dot(&v) / (u_len * v_len)).clamp(-1.0, 1.0).acos())
    }

    /// Return the area-weighted vertex normal.
    pub fn vertex_normal(&self, vertex_key: usize) -> Option<Vector> {
        self.vertex_normal_weighted(vertex_key, NormalWeighting::Area)
    }

    /// Return the vertex normal with the given weighting.
    pub fn vertex_normal_weighted(
        &self,
        vertex_key: usize,
        weighting: NormalWeighting,
    ) -> Option<Vector> {
        let faces = self.vertex_faces(vertex_key)?;

        if faces.is_empty() {
            return None;
        }

        let mut normal_acc = Vector::new(0.0, 0.0, 0.0);

        for face_key in faces {
            let Some(fn_) = self.face_normal(face_key) else {
                continue;
            };
            let weight = match weighting {
                NormalWeighting::Area => self.face_area(face_key).unwrap_or(1.0),
                NormalWeighting::Angle => self
                    .vertex_angle_in_face(vertex_key, face_key)
                    .unwrap_or(1.0),
                NormalWeighting::Uniform => 1.0,
            };
            normal_acc += &fn_ * weight;
        }

        let len = normal_acc.magnitude();

        if len > Tolerance::ZERO_TOLERANCE {
            return Some(&normal_acc / len);
        }

        None
    }

    /// Return the area-weighted normals of all vertices.
    pub fn vertex_normals(&self) -> HashMap<usize, Vector> {
        self.vertex_normals_weighted(NormalWeighting::Area)
    }

    /// Corner weight of vertex i in a face: its interior angle.
    fn corner_angle(pts: &[Point], i: usize) -> f64 {
        let n = pts.len();
        let prev = (i + n - 1) % n;
        let next = (i + 1) % n;
        let a = &pts[prev] - &pts[i];
        let b = &pts[next] - &pts[i];
        let a_len = a.magnitude();
        let b_len = b.magnitude();

        if a_len < Tolerance::ZERO_TOLERANCE || b_len < Tolerance::ZERO_TOLERANCE {
            return 0.0;
        }

        (a.dot(&b) / (a_len * b_len)).clamp(-1.0, 1.0).acos()
    }

    /// Return the normals of all vertices with the given weighting.
    pub fn vertex_normals_weighted(&self, weighting: NormalWeighting) -> HashMap<usize, Vector> {
        let mut acc: HashMap<usize, Vector> = HashMap::new();

        for fk in self.faces() {
            let vkeys = &self.face[&fk];

            if vkeys.len() < 3 {
                continue;
            }

            let Some(pts) = self.face_points(fk) else {
                continue;
            };
            let Some(normal) = self.face_normal(fk) else {
                continue;
            };
            let area = match weighting {
                NormalWeighting::Area => self.face_area(fk).unwrap_or(0.0),
                _ => 0.0,
            };

            for (i, &vk) in vkeys.iter().enumerate() {
                let weight = match weighting {
                    NormalWeighting::Uniform => 1.0,
                    NormalWeighting::Area => area,
                    NormalWeighting::Angle => Mesh::corner_angle(&pts, i),
                };
                *acc.entry(vk).or_insert(Vector::new(0.0, 0.0, 0.0)) += &normal * weight;
            }
        }

        let mut normals = HashMap::new();

        for (&vk, v) in &acc {
            let len = v.magnitude();

            if len > Tolerance::ZERO_TOLERANCE {
                normals.insert(vk, v / len);
            }
        }

        normals
    }

    /// Return the enclosed volume of a closed mesh.
    pub fn volume(&self) -> f64 {
        let mut total = 0.0;

        for fk in self.faces() {
            let vkeys = &self.face[&fk];

            if vkeys.len() < 3 {
                continue;
            }

            let Some(p0) = self.vertex_point(vkeys[0]) else {
                continue;
            };

            for i in 1..vkeys.len() - 1 {
                let (Some(p1), Some(p2)) =
                    (self.vertex_point(vkeys[i]), self.vertex_point(vkeys[i + 1]))
                else {
                    continue;
                };
                total += p0[0] * (p1[1] * p2[2] - p1[2] * p2[1])
                    + p0[1] * (p1[2] * p2[0] - p1[0] * p2[2])
                    + p0[2] * (p1[0] * p2[1] - p1[1] * p2[0]);
            }
        }

        total.abs() / 6.0
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Triangle BVH
    // ═══════════════════════════════════════════════════════════════════════════
    /// Every triangle of the mesh: the stored triangulation of an n-gon, a fan from vertex 0 otherwise.
    fn triangle_tasks(&self, faces: &[Vec<usize>]) -> Vec<(usize, usize, usize, usize, usize)> {
        let vkey_to_idx = self.vertex_index();
        let face_keys = self.faces();
        let mut tasks = Vec::new();

        for fi in 0..faces.len() {
            let fv = &faces[fi];

            if fv.len() < 3 {
                continue;
            }

            let tris = if fv.len() >= 5 {
                self.triangulation.get(&face_keys[fi])
            } else {
                None
            };

            if let Some(tris) = tris {
                for (j, t) in tris.iter().enumerate() {
                    tasks.push((
                        vkey_to_idx[&t[0]],
                        vkey_to_idx[&t[1]],
                        vkey_to_idx[&t[2]],
                        fi,
                        j,
                    ));
                }

                continue;
            }

            for j in 1..fv.len() - 1 {
                tasks.push((fv[0], fv[j], fv[j + 1], fi, j));
            }
        }

        tasks
    }

    /// AABB of a triangle, padded by a thousandth.
    fn triangle_aabb(p0: &Point, p1: &Point, p2: &Point) -> AABB {
        let min_x = p0[0].min(p1[0]).min(p2[0]) - 0.001;
        let min_y = p0[1].min(p1[1]).min(p2[1]) - 0.001;
        let min_z = p0[2].min(p1[2]).min(p2[2]) - 0.001;
        let max_x = p0[0].max(p1[0]).max(p2[0]) + 0.001;
        let max_y = p0[1].max(p1[1]).max(p2[1]) + 0.001;
        let max_z = p0[2].max(p1[2]).max(p2[2]) + 0.001;

        AABB::new(
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
            (max_x - min_x) * 0.5,
            (max_y - min_y) * 0.5,
            (max_z - min_z) * 0.5,
        )
    }

    /// World size for the BVH Morton grid: 2.2 times the largest absolute extent, at least 10.
    fn triangle_world_size(aabbs: &[AABB]) -> f64 {
        let mut extent = 0.0f64;

        for bb in aabbs {
            extent = extent.max((bb.cx - bb.hx).abs()).max((bb.cx + bb.hx).abs());
            extent = extent.max((bb.cy - bb.hy).abs()).max((bb.cy + bb.hy).abs());
            extent = extent.max((bb.cz - bb.hz).abs()).max((bb.cz + bb.hz).abs());
        }

        (2.2 * extent).max(10.0)
    }

    /// Build and cache the BVH over the triangulated faces.
    pub fn build_triangle_bvh(&mut self, force: bool) {
        if self.triangle_bvh_built && !force {
            return;
        }

        self.clear_triangle_bvh();
        let (vertices, faces) = self.to_vertices_and_faces();
        let tasks = self.triangle_tasks(&faces);
        self.tri_vertices = vertices;

        for (i0, i1, i2, face_idx, sub_idx) in tasks {
            self.tri_aabbs.push(Mesh::triangle_aabb(
                &self.tri_vertices[i0],
                &self.tri_vertices[i1],
                &self.tri_vertices[i2],
            ));
            self.tri_tris.push([i0, i1, i2]);
            self.tri_face_subidx.push((face_idx, sub_idx));
        }

        let mut bvh = SpatialBVH::new();
        bvh.build_from_aabbs(&self.tri_aabbs, Mesh::triangle_world_size(&self.tri_aabbs));
        self.tri_bvh = Some(bvh);
        self.triangle_bvh_built = true;
    }

    /// Collect the candidate triangle ids along a ray from the cached BVH; true when any.
    pub fn triangle_bvh_ray_cast(
        &mut self,
        origin: &Point,
        direction: &Vector,
        candidate_ids: &mut Vec<usize>,
        find_all: bool,
    ) -> bool {
        self.build_triangle_bvh(false);
        let Some(bvh) = &self.tri_bvh else {
            return false;
        };

        bvh.ray_cast(origin, direction, candidate_ids, find_all)
    }

    /// Look up the face index, sub-triangle index and corners of a cached triangle id; false when out of range.
    pub fn get_triangle_by_id(&self, tri_id: usize) -> Option<(usize, usize, Point, Point, Point)> {
        if tri_id >= self.tri_tris.len() || tri_id >= self.tri_face_subidx.len() {
            return None;
        }

        let tri = self.tri_tris[tri_id];
        let (face_idx, sub_idx) = self.tri_face_subidx[tri_id];

        if tri[0] >= self.tri_vertices.len()
            || tri[1] >= self.tri_vertices.len()
            || tri[2] >= self.tri_vertices.len()
        {
            return None;
        }

        Some((
            face_idx,
            sub_idx,
            self.tri_vertices[tri[0]].clone(),
            self.tri_vertices[tri[1]].clone(),
            self.tri_vertices[tri[2]].clone(),
        ))
    }

    /// Drop the cached BVH, AABB tree and triangle data.
    pub fn clear_triangle_bvh(&mut self) {
        self.triangle_bvh_built = false;
        self.tri_bvh = None;
        self.tri_aabb_tree = None;
        self.tri_aabbs.clear();
        self.tri_tris.clear();
        self.tri_face_subidx.clear();
        self.tri_vertices.clear();
        self.gpu_cache.0 = None;
    }

    /// Build and cache the AABB tree over the triangulated faces.
    pub fn build_triangle_aabb_tree(&mut self, force: bool) {
        self.build_triangle_bvh(false);

        if self.tri_aabb_tree.is_some() && !force {
            return;
        }

        let mut tree = SpatialAABBTree::new();
        tree.build(&self.tri_aabbs);
        self.tri_aabb_tree = Some(tree);
    }

    /// Return the cached triangle BVH, nullptr before build_triangle_bvh.
    pub fn get_cached_bvh(&self) -> Option<&SpatialBVH> {
        self.tri_bvh.as_ref()
    }

    /// Return the cached triangle AABB tree, nullptr before build_triangle_aabb_tree.
    pub fn get_cached_aabb_tree(&self) -> Option<&SpatialAABBTree> {
        self.tri_aabb_tree.as_ref()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform every vertex in place and drop the triangle caches; always true.
    pub fn transform(&mut self, xf: &Xform) -> bool {
        for vdata in self.vertex.values_mut() {
            let mut point = vdata.position();
            point.transform(xf);
            vdata.set_position(point);
        }

        self.clear_triangle_bvh();

        true
    }

    /// Return a transformed copy.
    pub fn transformed(&self, xf: &Xform) -> Self {
        let mut result = self.clone();
        result.transform(xf);

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Cutting
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the part on the side the plane normal points to, every section loop capped by one n-gon face, so a closed mesh stays closed; empty when nothing lies on that side, a copy when everything does.
    pub fn cut_by_plane(&self, plane: &Plane) -> Mesh {
        let big = f64::MAX;
        let mut low = Point::new(big, big, big);
        let mut high = Point::new(-big, -big, -big);
        let mut points: BTreeMap<usize, Point> = BTreeMap::new();

        for (vk, vd) in &self.vertex {
            let point = vd.position();

            for k in 0..3 {
                low[k] = low[k].min(point[k]);
                high[k] = high[k].max(point[k]);
            }

            points.insert(*vk, point);
        }

        let tolerance = 1e-9 * low.distance(&high, None);
        let mut distance: BTreeMap<usize, f64> = BTreeMap::new();
        let mut lowest = 0.0f64;
        let mut highest = 0.0f64;

        for (vk, point) in &points {
            let d = (point - &plane.origin()).dot(&plane.z_axis());
            let value = if d.abs() <= tolerance { 0.0 } else { d };
            distance.insert(*vk, value);
            lowest = lowest.min(value);
            highest = highest.max(value);
        }

        if lowest >= 0.0 {
            let mut copy = self.clone();
            copy.refresh_guid();

            return copy;
        }

        if highest <= 0.0 {
            return Mesh::new();
        }

        let mut crossings: BTreeMap<(usize, usize), usize> = BTreeMap::new();
        let mut output: BTreeMap<usize, CutFace> = BTreeMap::new();
        let mut count = self.max_face;
        let mut keys: Vec<usize> = self.face.keys().copied().collect();
        keys.sort();

        for fk in keys {
            let ring = &self.face[&fk];
            let normal = newell_normal(&cut_points(ring, &points));
            let mut rings = vec![ring.clone()];

            if let Some(holes) = self.face_holes.get(&fk) {
                for hole in holes {
                    rings.push(hole.clone());

                    if newell_normal(&cut_points(hole, &points)).dot(&normal) > 0.0 {
                        rings.last_mut().unwrap().reverse();
                    }
                }
            }

            let mut above = false;
            let mut below = false;

            for r in &rings {
                for key in r {
                    above = above || distance[key] > 0.0;
                    below = below || distance[key] < 0.0;
                }
            }

            if !above {
                continue;
            }

            if !below {
                output.insert(
                    fk,
                    CutFace {
                        rings,
                        parent: Some(fk),
                    },
                );
                continue;
            }

            let mut xaxis = plane.z_axis() - &normal * plane.z_axis().dot(&normal);

            if !xaxis.normalize_self() {
                continue;
            }

            let mut split: Vec<Vec<usize>> = Vec::new();

            for r in &rings {
                let mut walk = Vec::with_capacity(r.len());

                for i in 0..r.len() {
                    let edge = (r[i], r[(i + 1) % r.len()]);
                    walk.push(edge.0);

                    if distance[&edge.0] * distance[&edge.1] < 0.0 {
                        walk.push(cut_crossing(
                            edge,
                            &mut crossings,
                            &mut distance,
                            &mut points,
                            self.max_vertex,
                        ));
                    }
                }

                split.push(walk);
            }

            let pieces = cut_pieces(&split, &normal, &xaxis, &distance, &points, tolerance);

            for (i, mut piece) in pieces.into_iter().enumerate() {
                piece.parent = Some(fk);

                if i == 0 {
                    output.insert(fk, piece);
                } else {
                    output.insert(count, piece);
                    count += 1;
                }
            }
        }

        for cap in cut_caps(&output, &distance, &points, plane) {
            output.insert(count, cap);
            count += 1;
        }

        let mut result = Mesh::new();
        result.name = self.name.clone();
        result.objectcolor = self.objectcolor.clone();
        let mut used: BTreeSet<usize> = BTreeSet::new();

        for piece in output.values() {
            for r in &piece.rings {
                used.extend(r.iter().copied());
            }
        }

        for vk in used {
            result.add_vertex(points[&vk].clone(), Some(vk));
        }

        for (fk, piece) in &output {
            if result.add_face(piece.rings[0].clone(), Some(*fk)).is_none() {
                continue;
            }

            let whole = piece
                .parent
                .is_some_and(|parent| piece.rings[0] == self.face[&parent]);

            if piece.rings.len() > 1 {
                result.set_face_holes(*fk, piece.rings[1..].to_vec());
            }

            if let Some(data) = piece.parent.and_then(|parent| self.facedata.get(&parent)) {
                result.facedata.insert(*fk, data.clone());
            }

            if whole && self.triangulation.contains_key(fk) {
                result.set_face_triangulation(*fk, self.triangulation[fk].clone());
            }

            if !whole && (piece.rings.len() > 1 || piece.rings[0].len() > 3) {
                result.set_face_triangulation(*fk, cut_triangulation(piece, &points));
            }
        }

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Colors as a flat [r, g, b, a, ...] array.
    fn colors_to_json(colors: &[Color]) -> Vec<f32> {
        let mut arr = Vec::with_capacity(colors.len() * 4);

        for c in colors {
            arr.push(c.r);
            arr.push(c.g);
            arr.push(c.b);
            arr.push(c.a);
        }

        arr
    }

    /// Colors from a flat [r, g, b, a, ...] array.
    fn colors_from_json(arr: &serde_json::Value) -> Vec<Color> {
        let mut colors = Vec::new();
        let Some(values) = arr.as_array() else {
            return colors;
        };
        let mut i = 0;

        while i + 3 < values.len() {
            let r = values[i].as_f64().unwrap_or(0.0) as f32;
            let g = values[i + 1].as_f64().unwrap_or(0.0) as f32;
            let b = values[i + 2].as_f64().unwrap_or(0.0) as f32;
            let a = values[i + 3].as_f64().unwrap_or(0.0) as f32;
            colors.push(Color::new(r, g, b, a));
            i += 4;
        }

        colors
    }

    /// Serialize to a JSON object.
    pub fn jsondump(&self) -> serde_json::Value {
        let mut edgedata_json = serde_json::Map::new();

        for ((u, v), attrs) in &self.edgedata {
            edgedata_json.insert(format!("{},{}", u, v), serde_json::json!(attrs));
        }

        let mut face_json = serde_json::Map::new();

        for (key, vertices) in &self.face {
            face_json.insert(key.to_string(), serde_json::json!(vertices));
        }

        let mut face_holes_json = serde_json::Map::new();

        for (fk, rings) in &self.face_holes {
            face_holes_json.insert(fk.to_string(), serde_json::json!(rings));
        }

        let mut facedata_json = serde_json::Map::new();

        for (key, attrs) in &self.facedata {
            facedata_json.insert(key.to_string(), serde_json::json!(attrs));
        }

        let he = if self.halfedge.is_empty() && !self.face.is_empty() {
            self.compute_halfedges()
        } else {
            self.halfedge.clone()
        };
        let mut halfedge_json = serde_json::Map::new();

        for (u, neighbors) in &he {
            let mut neighbor_json = serde_json::Map::new();

            for (v, face_opt) in neighbors {
                neighbor_json.insert(v.to_string(), serde_json::json!(face_opt));
            }

            halfedge_json.insert(u.to_string(), serde_json::Value::Object(neighbor_json));
        }

        let mut triangulation_json = serde_json::Map::new();

        for (fk, tris) in &self.triangulation {
            let mut tri_arr = Vec::with_capacity(tris.len());

            for t in tris {
                tri_arr.push(serde_json::json!([t[0], t[1], t[2]]));
            }

            triangulation_json.insert(fk.to_string(), serde_json::Value::Array(tri_arr));
        }

        let mut vertex_json = serde_json::Map::new();

        for (key, vdata) in &self.vertex {
            vertex_json.insert(key.to_string(), serde_json::json!({"attributes": vdata.attributes, "x": vdata.x, "y": vdata.y, "z": vdata.z}));
        }

        serde_json::json!({
            "color_mode": self.color_mode.to_str(),
            "default_edge_attributes": self.default_edge_attributes,
            "default_face_attributes": self.default_face_attributes,
            "default_vertex_attributes": self.default_vertex_attributes,
            "edgedata": serde_json::Value::Object(edgedata_json),
            "face": serde_json::Value::Object(face_json),
            "face_holes": serde_json::Value::Object(face_holes_json),
            "facecolors": Mesh::colors_to_json(&self.facecolors),
            "facedata": serde_json::Value::Object(facedata_json),
            "guid": self.guid(),
            "halfedge": serde_json::Value::Object(halfedge_json),
            "linecolors": Mesh::colors_to_json(&self.linecolors),
            "max_face": self.max_face,
            "max_vertex": self.max_vertex,
            "name": self.name,
            "objectcolor": serde_json::to_value(&self.objectcolor).unwrap_or(serde_json::Value::Null),
            "pointcolors": Mesh::colors_to_json(&self.pointcolors),
            "triangulation": serde_json::Value::Object(triangulation_json),
            "type": "Mesh",
            "vertex": serde_json::Value::Object(vertex_json),
            "widths": self.widths
        })
    }

    /// Deserialize from a JSON object.
    pub fn jsonload(data: &serde_json::Value) -> Option<Self> {
        let mut mesh = Mesh::new();

        if let Some(guid) = data.get("guid").and_then(|v| v.as_str()) {
            mesh.set_guid(guid.to_string());
        }

        if let Some(name) = data.get("name").and_then(|v| v.as_str()) {
            mesh.name = name.to_string();
        }

        if let Some(halfedge_data) = data.get("halfedge") {
            mesh.halfedge = serde_json::from_value(halfedge_data.clone()).ok()?;
        }

        if let Some(vertex_data) = data.get("vertex") {
            mesh.vertex = serde_json::from_value(vertex_data.clone()).ok()?;

            for key in mesh.vertex.keys() {
                if data.get("halfedge").is_none() {
                    mesh.halfedge.entry(*key).or_default();
                }

                if *key >= mesh.max_vertex {
                    mesh.max_vertex = *key + 1;
                }
            }
        }

        if let Some(face_data) = data.get("face") {
            mesh.face = serde_json::from_value(face_data.clone()).ok()?;

            for key in mesh.face.keys() {
                if *key >= mesh.max_face {
                    mesh.max_face = *key + 1;
                }
            }
        }

        if let Some(fh) = data.get("face_holes").and_then(|v| v.as_object()) {
            for (fk_str, rings_val) in fh {
                let Ok(fk) = fk_str.parse::<usize>() else {
                    continue;
                };
                let Ok(rings) = serde_json::from_value::<Vec<Vec<usize>>>(rings_val.clone()) else {
                    continue;
                };
                mesh.face_holes.insert(fk, rings);
            }
        }

        if let Some(facedata) = data.get("facedata") {
            mesh.facedata = serde_json::from_value(facedata.clone()).ok()?;
        }

        if let Some(edgedata) = data.get("edgedata").and_then(|v| v.as_object()) {
            for (edge_str, attrs) in edgedata {
                let Some((u_str, v_str)) = edge_str.split_once(',') else {
                    continue;
                };
                let (Ok(u), Ok(v)) = (u_str.parse::<usize>(), v_str.parse::<usize>()) else {
                    continue;
                };
                let Ok(map) = serde_json::from_value::<HashMap<String, f64>>(attrs.clone()) else {
                    continue;
                };
                mesh.edgedata.insert((u, v), map);
            }
        }

        if let Some(v) = data.get("default_vertex_attributes") {
            mesh.default_vertex_attributes = serde_json::from_value(v.clone()).ok()?;
        }

        if let Some(v) = data.get("default_face_attributes") {
            mesh.default_face_attributes = serde_json::from_value(v.clone()).ok()?;
        }

        if let Some(v) = data.get("default_edge_attributes") {
            mesh.default_edge_attributes = serde_json::from_value(v.clone()).ok()?;
        }

        if let Some(max_vertex) = data.get("max_vertex").and_then(|v| v.as_u64()) {
            mesh.max_vertex = max_vertex as usize;
        }

        if let Some(max_face) = data.get("max_face").and_then(|v| v.as_u64()) {
            mesh.max_face = max_face as usize;
        }

        if let Some(arr) = data.get("pointcolors") {
            mesh.pointcolors = Mesh::colors_from_json(arr);
        }

        if let Some(arr) = data.get("facecolors") {
            mesh.facecolors = Mesh::colors_from_json(arr);
        }

        if let Some(arr) = data.get("linecolors") {
            mesh.linecolors = Mesh::colors_from_json(arr);
        }

        if let Some(widths) = data.get("widths").and_then(|v| v.as_array()) {
            mesh.widths.clear();

            for w in widths {
                if let Some(x) = w.as_f64() {
                    mesh.widths.push(x);
                }
            }
        }

        if let Some(oc) = data.get("objectcolor") {
            if let Ok(color) = serde_json::from_value::<Color>(oc.clone()) {
                mesh.objectcolor = color;
            }
        }

        if let Some(cm) = data.get("color_mode").and_then(|v| v.as_str()) {
            mesh.color_mode = ColorMode::from_str(cm);
        }

        if let Some(tri_obj) = data.get("triangulation").and_then(|v| v.as_object()) {
            for (fk_str, tris_val) in tri_obj {
                let Ok(fk) = fk_str.parse::<usize>() else {
                    continue;
                };
                let Ok(tris) = serde_json::from_value::<Vec<[usize; 3]>>(tris_val.clone()) else {
                    continue;
                };
                mesh.triangulation.insert(fk, tris);
            }
        }

        Some(mesh)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        let sorted = crate::file_encoders::sort_json_keys(self.jsondump());

        serde_json::to_string_pretty(&sorted).unwrap_or_default()
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        let data: serde_json::Value = serde_json::from_str(json_string).unwrap_or_default();

        Self::jsonload(&data).unwrap_or_default()
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&self, filename: &str) -> std::io::Result<()> {
        let sorted = crate::file_encoders::sort_json_keys(self.jsondump());

        std::fs::write(filename, serde_json::to_string_pretty(&sorted)?)
    }

    /// Read from a JSON file.
    pub fn file_json_load(filename: &str) -> std::io::Result<Self> {
        let content = std::fs::read_to_string(filename)?;
        let data: serde_json::Value = serde_json::from_str(&content)?;

        Self::jsonload(&data).ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::InvalidData, "Invalid mesh data")
        })
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Colors as flat r, g, b, a floats.
    fn colors_to_rgba(colors: &[Color]) -> Vec<f32> {
        let mut rgba = Vec::with_capacity(colors.len() * 4);

        for c in colors {
            rgba.push(c.r);
            rgba.push(c.g);
            rgba.push(c.b);
            rgba.push(c.a);
        }

        rgba
    }

    /// Colors from flat r, g, b, a floats.
    fn colors_from_rgba(rgba: &[f32]) -> Vec<Color> {
        let mut colors = Vec::with_capacity(rgba.len() / 4);
        let mut i = 0;

        while i + 3 < rgba.len() {
            colors.push(Color::new(rgba[i], rgba[i + 1], rgba[i + 2], rgba[i + 3]));
            i += 4;
        }

        colors
    }

    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::Mesh {
        let mut vertices: HashMap<u64, crate::proto::VertexData> =
            HashMap::with_capacity(self.vertex.len());

        for (&vkey, vdata) in &self.vertex {
            let mut attrs: BTreeMap<String, f64> = BTreeMap::new();

            for (k, v) in &vdata.attributes {
                attrs.insert(k.clone(), *v);
            }

            vertices.insert(
                vkey as u64,
                crate::proto::VertexData {
                    x: vdata.x,
                    y: vdata.y,
                    z: vdata.z,
                    attributes: attrs,
                },
            );
        }

        let mut faces: HashMap<u64, crate::proto::FaceData> =
            HashMap::with_capacity(self.face.len());

        for (&fkey, fverts) in &self.face {
            let mut attrs: BTreeMap<String, f64> = BTreeMap::new();

            if let Some(fdata) = self.facedata.get(&fkey) {
                for (k, v) in fdata {
                    attrs.insert(k.clone(), *v);
                }
            }

            let mut holes: Vec<crate::proto::HoleRing> = Vec::new();

            if let Some(rings) = self.face_holes.get(&fkey) {
                for ring in rings {
                    holes.push(crate::proto::HoleRing {
                        vertices: ring.iter().map(|&v| v as u64).collect(),
                    });
                }
            }

            faces.insert(
                fkey as u64,
                crate::proto::FaceData {
                    vertices: fverts.iter().map(|&v| v as u64).collect(),
                    attributes: attrs,
                    holes,
                },
            );
        }

        let mut triangulation: HashMap<u64, crate::proto::TriList> =
            HashMap::with_capacity(self.triangulation.len());

        for (&fkey, tris) in &self.triangulation {
            let mut tri_list = crate::proto::TriList {
                vertices: Vec::with_capacity(tris.len() * 3),
            };

            for t in tris {
                tri_list.vertices.push(t[0] as u64);
                tri_list.vertices.push(t[1] as u64);
                tri_list.vertices.push(t[2] as u64);
            }

            triangulation.insert(fkey as u64, tri_list);
        }

        let mut edge_data: Vec<crate::proto::EdgeData> = Vec::with_capacity(self.edgedata.len());
        let mut ekeys: Vec<(usize, usize)> = self.edgedata.keys().copied().collect();
        ekeys.sort_unstable();

        for ek in ekeys {
            let mut attrs: BTreeMap<String, f64> = BTreeMap::new();

            for (k, v) in &self.edgedata[&ek] {
                attrs.insert(k.clone(), *v);
            }

            edge_data.push(crate::proto::EdgeData {
                vertex1: ek.0 as u64,
                vertex2: ek.1 as u64,
                attributes: attrs,
            });
        }

        crate::proto::Mesh {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            vertices,
            faces,
            edge_data,
            default_vertex_attributes: self
                .default_vertex_attributes
                .iter()
                .map(|(k, v)| (k.clone(), *v))
                .collect(),
            default_face_attributes: self
                .default_face_attributes
                .iter()
                .map(|(k, v)| (k.clone(), *v))
                .collect(),
            default_edge_attributes: self
                .default_edge_attributes
                .iter()
                .map(|(k, v)| (k.clone(), *v))
                .collect(),
            pointcolors_rgba: Mesh::colors_to_rgba(&self.pointcolors),
            facecolors_rgba: Mesh::colors_to_rgba(&self.facecolors),
            linecolors_rgba: Mesh::colors_to_rgba(&self.linecolors),
            widths: self.widths.clone(),
            objectcolor: Some(self.objectcolor.to_proto()),
            color_mode: self.color_mode.to_i32(),
            triangulation,
        }
    }

    /// Construct from the protobuf message.
    pub fn from_proto(proto: crate::proto::Mesh) -> Self {
        let mut mesh = Self::new();

        if !proto.guid.is_empty() {
            mesh.set_guid(proto.guid.clone());
        }

        mesh.name = proto.name;
        mesh.vertex.reserve(proto.vertices.len());
        mesh.face.reserve(proto.faces.len());

        for (vkey, vdata) in proto.vertices {
            mesh.vertex.insert(
                vkey as usize,
                VertexData {
                    x: vdata.x,
                    y: vdata.y,
                    z: vdata.z,
                    attributes: vdata.attributes.into_iter().collect(),
                },
            );
        }

        for (fkey, fdata) in proto.faces {
            mesh.face.insert(
                fkey as usize,
                fdata.vertices.iter().map(|&v| v as usize).collect(),
            );

            if !fdata.attributes.is_empty() {
                mesh.facedata
                    .insert(fkey as usize, fdata.attributes.into_iter().collect());
            }

            if !fdata.holes.is_empty() {
                let mut rings: Vec<Vec<usize>> = Vec::with_capacity(fdata.holes.len());

                for h in &fdata.holes {
                    rings.push(h.vertices.iter().map(|&v| v as usize).collect());
                }

                mesh.face_holes.insert(fkey as usize, rings);
            }
        }

        for (fkey, tri_list) in proto.triangulation {
            let vlist = &tri_list.vertices;
            let mut tris: Vec<[usize; 3]> = Vec::with_capacity(vlist.len() / 3);
            let mut i = 0;

            while i + 2 < vlist.len() {
                tris.push([
                    vlist[i] as usize,
                    vlist[i + 1] as usize,
                    vlist[i + 2] as usize,
                ]);
                i += 3;
            }

            mesh.triangulation.insert(fkey as usize, tris);
        }

        for edata in proto.edge_data {
            mesh.edgedata.insert(
                (edata.vertex1 as usize, edata.vertex2 as usize),
                edata.attributes.into_iter().collect(),
            );
        }

        mesh.default_vertex_attributes = proto.default_vertex_attributes.into_iter().collect();
        mesh.default_face_attributes = proto.default_face_attributes.into_iter().collect();
        mesh.default_edge_attributes = proto.default_edge_attributes.into_iter().collect();
        mesh.pointcolors = Mesh::colors_from_rgba(&proto.pointcolors_rgba);
        mesh.facecolors = Mesh::colors_from_rgba(&proto.facecolors_rgba);
        mesh.linecolors = Mesh::colors_from_rgba(&proto.linecolors_rgba);
        mesh.widths = proto.widths;

        if let Some(color) = proto.objectcolor {
            mesh.objectcolor = Color::from_proto(color);
        }

        mesh.color_mode = ColorMode::from_i32(proto.color_mode);

        if let Some(&max_v) = mesh.vertex.keys().max() {
            mesh.max_vertex = max_v + 1;
        }

        if let Some(&max_f) = mesh.face.keys().max() {
            mesh.max_face = max_f + 1;
        }

        mesh.orient_faces();
        mesh.triangulate_faces();

        mesh
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Ok(Self::from_proto(crate::proto::Mesh::decode(data)?))
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    /// Read from a protobuf file.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");

        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the "Mesh(name=..., vertices=..., faces=...)" form.
    pub fn str(&self) -> String {
        format!(
            "Mesh(name={}, vertices={}, faces={})",
            self.name,
            self.number_of_vertices(),
            self.number_of_faces()
        )
    }

    /// Return the multi-line form with name, vertices, faces and edges.
    pub fn repr(&self) -> String {
        format!(
            "Mesh(\n  name={},\n  vertices={},\n  faces={},\n  edges={}\n)",
            self.name,
            self.number_of_vertices(),
            self.number_of_faces(),
            self.number_of_edges()
        )
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Private helpers
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return every directed edge (u, v) some face ring walks.
    fn directed_face_edges(&self) -> HashSet<(usize, usize)> {
        let mut s = HashSet::with_capacity(self.face.len() * 4);

        for verts in self.face.values() {
            let n = verts.len();

            for i in 0..n {
                s.insert((verts[i], verts[(i + 1) % n]));
            }
        }

        s
    }

    /// Return the face-derived halfedge connectivity, computed without mutating.
    fn compute_halfedges(&self) -> HashMap<usize, HashMap<usize, Option<usize>>> {
        let mut he: HashMap<usize, HashMap<usize, Option<usize>>> =
            HashMap::with_capacity(self.vertex.len());

        for vkey in self.vertex.keys() {
            he.insert(*vkey, HashMap::new());
        }

        for fkey in self.faces() {
            let verts = &self.face[&fkey];
            let n = verts.len();

            for i in 0..n {
                let u = verts[i];
                let v = verts[(i + 1) % n];
                he.entry(u).or_default().insert(v, Some(fkey));
                he.entry(v).or_default().entry(u).or_insert(None);
            }
        }

        he
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SESSION_VIEWER
    // ═══════════════════════════════════════════════════════════════════════════
    /// Make windings consistent and, on a closed mesh, outward, so the viewer can trust a face normal.
    fn orient_faces(&mut self) {
        if self.face.len() < 2 {
            return;
        }

        let mut walked: HashSet<(usize, usize)> = HashSet::with_capacity(self.face.len() * 4);
        let mut consistent = true;

        for verts in self.face.values() {
            let n = verts.len();

            for i in 0..n {
                if !walked.insert((verts[i], verts[(i + 1) % n])) {
                    consistent = false;
                }
            }
        }

        if !consistent {
            self.unify_winding();

            return;
        }

        if walked.iter().any(|&(u, v)| !walked.contains(&(v, u))) {
            return;
        }

        if self.signed_volume() >= 0.0 {
            return;
        }

        for verts in self.face.values_mut() {
            verts.reverse();
        }

        for rings in self.face_holes.values_mut() {
            for ring in rings.iter_mut() {
                ring.reverse();
            }
        }

        self.halfedge.clear();
        self.triangulation.clear();
    }

    /// Six times the volume the winding encloses, positive when the faces wind outward.
    fn signed_volume(&self) -> f64 {
        let mut total = 0.0;

        for verts in self.face.values() {
            if verts.len() < 3 {
                continue;
            }

            let Some(p0) = self.vertex_point(verts[0]) else {
                continue;
            };

            for i in 1..verts.len() - 1 {
                let (Some(p1), Some(p2)) =
                    (self.vertex_point(verts[i]), self.vertex_point(verts[i + 1]))
                else {
                    continue;
                };
                total += p0[0] * (p1[1] * p2[2] - p1[2] * p2[1])
                    + p0[1] * (p1[2] * p2[0] - p1[0] * p2[2])
                    + p0[2] * (p1[0] * p2[1] - p1[1] * p2[0]);
            }
        }

        total
    }

    /// Fill triangulation for every n-gon a fan from vertex 0 would render wrong.
    fn triangulate_faces(&mut self) {
        for face_key in self.faces() {
            if self.triangulation.contains_key(&face_key) {
                continue;
            }

            let Some(keys) = self.face_vertices(face_key).cloned() else {
                continue;
            };

            if keys.len() < 4 {
                continue;
            }

            let Some(points) = self.face_points(face_key) else {
                continue;
            };
            let normal = newell_normal(&points);
            let (nx, ny, nz) = (normal[0], normal[1], normal[2]);

            if nx * nx + ny * ny + nz * nz < 0.5 {
                continue;
            }

            let mut fan_is_wrong = false;

            for i in 1..keys.len() - 1 {
                let (p0, p1, p2) = (&points[0], &points[i], &points[i + 1]);
                let (e1x, e1y, e1z) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
                let (e2x, e2y, e2z) = (p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);
                let cross = [
                    e1y * e2z - e1z * e2y,
                    e1z * e2x - e1x * e2z,
                    e1x * e2y - e1y * e2x,
                ];

                if cross[0] * nx + cross[1] * ny + cross[2] * nz < 0.0 {
                    fan_is_wrong = true;
                }
            }

            if !fan_is_wrong {
                continue;
            }

            let tris = planar_cdt(&points);

            if tris.is_empty() {
                continue;
            }

            let mut oriented: Vec<[usize; 3]> = Vec::with_capacity(tris.len());

            for &(a, b, c) in &tris {
                let (p0, p1, p2) = (&points[a], &points[b], &points[c]);
                let (e1x, e1y, e1z) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
                let (e2x, e2y, e2z) = (p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);
                let cross = [
                    e1y * e2z - e1z * e2y,
                    e1z * e2x - e1x * e2z,
                    e1x * e2y - e1y * e2x,
                ];

                if cross[0] * nx + cross[1] * ny + cross[2] * nz < 0.0 {
                    oriented.push([keys[a], keys[c], keys[b]]);
                } else {
                    oriented.push([keys[a], keys[b], keys[c]]);
                }
            }

            self.triangulation.insert(face_key, oriented);
        }
    }

    /// True when the triangle BVH cache is built and usable by ray_cast_bvh_ready.
    pub fn has_triangle_bvh(&self) -> bool {
        self.tri_bvh.is_some() && !self.tri_tris.is_empty() && !self.tri_vertices.is_empty()
    }

    /// Return the nearest hit along a ray, building the triangle BVH first.
    pub fn ray_cast_bvh(&mut self, ray: &Line, epsilon: f64) -> Option<Point> {
        self.build_triangle_bvh(false);

        self.ray_cast_bvh_ready(ray, epsilon)
    }

    /// Nearest hit along a ray against an already-built triangle BVH.
    pub fn ray_cast_bvh_ready(&self, ray: &Line, epsilon: f64) -> Option<Point> {
        let bvh = self.tri_bvh.as_ref()?;
        let origin = ray.start();
        let dir = ray.to_vector();
        let len = dir.magnitude();

        if len <= Tolerance::ZERO_TOLERANCE {
            return None;
        }

        let dir_unit = Vector::new(dir[0] / len, dir[1] / len, dir[2] / len);
        let mut candidate_ids: Vec<usize> = Vec::new();
        bvh.ray_cast(&origin, &dir_unit, &mut candidate_ids, true);
        let mut best_t = f64::INFINITY;
        let mut best_p: Option<Point> = None;

        for idx in candidate_ids {
            if idx >= self.tri_tris.len() {
                continue;
            }

            let tri = self.tri_tris[idx];
            let v0 = &self.tri_vertices[tri[0]];
            let v1 = &self.tri_vertices[tri[1]];
            let v2 = &self.tri_vertices[tri[2]];
            let Some(p) = crate::intersection::ray_triangle(ray, v0, v1, v2, epsilon) else {
                continue;
            };
            let t = (p[0] - origin[0]) * dir_unit[0]
                + (p[1] - origin[1]) * dir_unit[1]
                + (p[2] - origin[2]) * dir_unit[2];

            if t >= 0.0 && t < best_t {
                best_t = t;
                best_p = Some(p);
            }
        }

        best_p
    }

    /// Drop halfedges, colors and widths once the GPU buffers hold them.
    pub fn strip_render_data(&mut self) {
        self.halfedge.clear();
        self.pointcolors.clear();
        self.facecolors.clear();
        self.linecolors.clear();
        self.widths.clear();
    }

    /// Edges paired with their stored line color, in the order add_face seeded linecolors.
    pub fn edges_with_colors(&self) -> Vec<(usize, usize, Color)> {
        let mut seen: HashSet<(usize, usize)> = HashSet::new();
        let mut out: Vec<(usize, usize, Color)> = Vec::new();
        let mut ci = 0usize;

        for fk in self.faces() {
            let vs = &self.face[&fk];

            for i in 0..vs.len() {
                let u = vs[i];
                let v = vs[(i + 1) % vs.len()];
                let e = (u.min(v), u.max(v));

                if seen.insert(e) {
                    let c = self
                        .linecolors
                        .get(ci)
                        .cloned()
                        .unwrap_or_else(Color::black);

                    out.push((e.0, e.1, c));
                    ci += 1;
                }
            }
        }

        out
    }

    /// Return the edge widths.
    pub fn widths(&self) -> &[f64] {
        &self.widths
    }

    /// Return the object color.
    pub fn objectcolor(&self) -> &Color {
        &self.objectcolor
    }
}

impl std::fmt::Display for Mesh {
    /// Write the str() form to a formatter.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.str())
    }
}

impl std::fmt::Debug for Mesh {
    /// Write the repr() form to a formatter.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.repr())
    }
}

impl PartialEq for Mesh {
    /// Compare name, vertices and faces; guid ignored.
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name && self.vertex == other.vertex && self.face == other.face
    }
}
