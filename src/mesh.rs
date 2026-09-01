use crate::{OBB, Color, Line, Point, Tolerance, Vector, Xform, SpatialBVH};
use crate::polyline::Polyline;
use crate::remesh_cdt;
use crate::tolerance::PI;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

/// Active color mode for mesh rendering
#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ColorMode {
    #[default] OBJECTCOLOR,
    POINTCOLORS,
    FACECOLORS,
    NONE,
}

impl ColorMode {
    fn to_i32(&self) -> i32 {
        match self { Self::OBJECTCOLOR => 0, Self::POINTCOLORS => 1, Self::FACECOLORS => 2, Self::NONE => 3 }
    }
    fn from_i32(v: i32) -> Self {
        match v { 1 => Self::POINTCOLORS, 2 => Self::FACECOLORS, 3 => Self::NONE, _ => Self::OBJECTCOLOR }
    }
    fn to_str(&self) -> &'static str {
        match self { Self::OBJECTCOLOR => "objectcolor", Self::POINTCOLORS => "pointcolors", Self::FACECOLORS => "facecolors", Self::NONE => "none" }
    }
    fn from_str(s: &str) -> Self {
        match s { "pointcolors" => Self::POINTCOLORS, "facecolors" => Self::FACECOLORS, "none" => Self::NONE, _ => Self::OBJECTCOLOR }
    }
}

/// Weighting scheme for vertex normal computation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NormalWeighting {
    Area,
    Angle,
    Uniform,
}

/// A halfedge mesh data structure for representing polygonal surfaces
#[derive(Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Mesh")]
pub struct Mesh {
    pub halfedge: HashMap<usize, HashMap<usize, Option<usize>>>, // Halfedge connectivity
    pub vertex: HashMap<usize, VertexData>,                      // Vertex data
    pub face: HashMap<usize, Vec<usize>>,                        // Face vertex lists
    pub facedata: HashMap<usize, HashMap<String, f64>>,          // Face attributes
    pub edgedata: HashMap<(usize, usize), HashMap<String, f64>>, // Edge attributes
    pub default_vertex_attributes: HashMap<String, f64>,         // Default vertex attrs
    pub default_face_attributes: HashMap<String, f64>,           // Default face attrs
    pub default_edge_attributes: HashMap<String, f64>,           // Default edge attrs
    #[serde(skip)]
    pub triangulation: HashMap<usize, Vec<[usize; 3]>>, // Cached triangulations
    #[serde(skip)]
    pub face_holes: HashMap<usize, Vec<Vec<usize>>>,    // Face hole rings
    max_vertex: usize,                                           // Next vertex key
    max_face: usize,                                             // Next face key
    #[serde(serialize_with = "crate::guid_serde::serialize", deserialize_with = "crate::guid_serde::deserialize")]
    guid: std::sync::OnceLock<String>,                          // Unique identifier
    pub name: String,                                            // Mesh name
    #[serde(skip)]
    pointcolors: Vec<Color>,                   // Vertex colors
    #[serde(skip)]
    facecolors: Vec<Color>,                    // Face colors
    #[serde(skip)]
    linecolors: Vec<Color>,                    // Edge colors
    #[serde(skip)]
    widths: Vec<f64>,                          // Edge widths
    #[serde(skip)]
    objectcolor: Color,                        // Object color
    #[serde(skip)]
    pub color_mode: ColorMode,                 // Active color mode
    // Cached triangle BVH for ray queries (not serialized)
    #[serde(skip)]
    pub tri_bvh: Option<SpatialBVH>,
    #[serde(skip)]
    pub tri_tris: Vec<[usize; 3]>,
    #[serde(skip)]
    pub tri_vertices: Vec<Point>,
    #[serde(skip)]
    pub crease_angle_deg: f64,
    // Cached GPU buffers (f32, render-side). Built once by `gpu_mesh()`, dropped on
    // any geometry/color change. Never serialized; resets to empty on clone.
    #[serde(skip)]
    pub(crate) gpu_cache: crate::render_mesh::GpuCache, // accessed by render_mesh::{gpu_mesh, invalidate_gpu}
}

/// A vertex's attribute map, paid for ONLY when the vertex has attributes.
///
/// This is a `HashMap<String, f64>` in every way that matters at a call site - `get`, `insert`,
/// `len`, iteration, `collect()` - but it costs 8 bytes inside `VertexData` instead of 48, and it
/// touches the heap only once something is actually stored in it.
///
/// The reason is the shape of real data. A mesh from a PDF sheet or a scan carries positions and
/// nothing else: 362,581 vertices, ZERO attribute entries, and 48 bytes per vertex of empty map
/// header regardless - 17 MB of one sheet's 52 MB. Attributes are a NURBS-tessellation and
/// vertex-colour feature (u/v, r/g/b, nx/ny/nz); those meshes are thousands of vertices, not
/// millions, and one `Box` each is nothing to them.
///
/// `BTreeMap` inside, not `HashMap`: the order is then alphabetical instead of seeded-random,
/// which is what the JSON dumps want and what `std::map` gives the C++ side.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Attributes(Option<Box<std::collections::BTreeMap<String, f64>>>);

impl Attributes {
    pub fn new() -> Self { Self(None) }
    pub fn get(&self, key: &str) -> Option<&f64> { self.0.as_ref().and_then(|m| m.get(key)) }
    pub fn contains_key(&self, key: &str) -> bool { self.get(key).is_some() }
    pub fn len(&self) -> usize { self.0.as_ref().map_or(0, |m| m.len()) }
    pub fn is_empty(&self) -> bool { self.len() == 0 }
    pub fn clear(&mut self) { self.0 = None }
    pub fn insert(&mut self, key: String, value: f64) -> Option<f64> {
        self.0.get_or_insert_with(Default::default).insert(key, value)
    }
    pub fn remove(&mut self, key: &str) -> Option<f64> {
        let out = self.0.as_mut().and_then(|m| m.remove(key));
        // Back to the un-allocated state, so a map that empties out stops costing anything.
        if self.is_empty() { self.0 = None }
        out
    }
    pub fn iter(&self) -> std::collections::btree_map::Iter<'_, String, f64> {
        // An empty map's iterator, so callers never special-case the None arm. `Box::leak` is not
        // needed: a fresh empty BTreeMap has no allocation, but it also has no lifetime here -
        // hence the static empty below.
        static EMPTY: std::sync::OnceLock<std::collections::BTreeMap<String, f64>> = std::sync::OnceLock::new();
        match &self.0 {
            Some(m) => m.iter(),
            None => EMPTY.get_or_init(Default::default).iter(),
        }
    }
    pub fn keys(&self) -> impl Iterator<Item = &String> { self.iter().map(|(k, _)| k) }
    pub fn values(&self) -> impl Iterator<Item = &f64> { self.iter().map(|(_, v)| v) }
}

impl<'a> IntoIterator for &'a Attributes {
    type Item = (&'a String, &'a f64);
    type IntoIter = std::collections::btree_map::Iter<'a, String, f64>;
    fn into_iter(self) -> Self::IntoIter { self.iter() }
}

impl IntoIterator for Attributes {
    type Item = (String, f64);
    type IntoIter = std::collections::btree_map::IntoIter<String, f64>;
    fn into_iter(self) -> Self::IntoIter {
        self.0.map_or_else(Default::default, |m| *m).into_iter()
    }
}

impl FromIterator<(String, f64)> for Attributes {
    fn from_iter<T: IntoIterator<Item = (String, f64)>>(it: T) -> Self {
        let m: std::collections::BTreeMap<String, f64> = it.into_iter().collect();
        if m.is_empty() { Self(None) } else { Self(Some(Box::new(m))) }
    }
}

impl Extend<(String, f64)> for Attributes {
    fn extend<T: IntoIterator<Item = (String, f64)>>(&mut self, it: T) {
        for (k, v) in it { self.insert(k, v); }
    }
}

impl Serialize for Attributes {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeMap;
        let mut m = s.serialize_map(Some(self.len()))?;
        for (k, v) in self.iter() { m.serialize_entry(k, v)?; }
        m.end()
    }
}

impl<'de> Deserialize<'de> for Attributes {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let m = std::collections::BTreeMap::<String, f64>::deserialize(d)?;
        Ok(if m.is_empty() { Self(None) } else { Self(Some(Box::new(m))) })
    }
}

/// Vertex data containing position and attributes
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VertexData {
    pub x: f64,                           // X coordinate
    pub y: f64,                           // Y coordinate
    pub z: f64,                           // Z coordinate
    pub attributes: Attributes,           // Vertex attributes, 8 B when there are none
}

impl VertexData {
    pub fn new(point: Point) -> Self {
        Self {
            x: point[0],
            y: point[1],
            z: point[2],
            attributes: Attributes::new(),
        }
    }

    pub fn position(&self) -> Point {
        Point::new(self.x, self.y, self.z)
    }

    pub fn set_position(&mut self, point: Point) {
        self.x = point[0];
        self.y = point[1];
        self.z = point[2];
    }

    pub fn color(&self) -> [f64; 3] {
        [
            self.attributes.get("r").copied().unwrap_or(0.5),
            self.attributes.get("g").copied().unwrap_or(0.5),
            self.attributes.get("b").copied().unwrap_or(0.5),
        ]
    }

    pub fn set_color(&mut self, r: f64, g: f64, b: f64) {
        self.attributes.insert("r".to_string(), r);
        self.attributes.insert("g".to_string(), g);
        self.attributes.insert("b".to_string(), b);
    }

    pub fn normal(&self) -> Option<[f64; 3]> {
        let nx = self.attributes.get("nx")?;
        let ny = self.attributes.get("ny")?;
        let nz = self.attributes.get("nz")?;
        Some([*nx, *ny, *nz])
    }

    pub fn set_normal(&mut self, nx: f64, ny: f64, nz: f64) {
        self.attributes.insert("nx".to_string(), nx);
        self.attributes.insert("ny".to_string(), ny);
        self.attributes.insert("nz".to_string(), nz);
    }
}

pub struct LoftWallFace {
    pub face_key: usize,
    pub face_index: usize,
    pub is_quad: bool,
    pub top_v0: usize,
    pub top_v1: usize,
    pub bot_v0: usize,
    pub bot_v1: usize,
}

pub struct LoftAdjPair {
    pub pi: usize,
    pub wi: usize,
    pub pj: usize,
    pub wj: usize,
}

pub struct LoftPanel {
    pub mesh: Mesh,
    pub top_face_key: usize,
    pub bot_face_key: usize,
    pub wall_faces: Vec<LoftWallFace>,
    pub orig_top_to_local: HashMap<usize, usize>,
    pub orig_bot_to_local: HashMap<usize, usize>,
    pub top_vertices: Vec<usize>,
    pub bot_vertices: Vec<usize>,
    pub face_roles: HashMap<usize, &'static str>,
}

impl Default for Mesh {
    fn default() -> Self {
        Self::new()
    }
}

fn lp_newell_normal(pts: &[Point]) -> (f64, f64, f64) {
    let (mut nx, mut ny, mut nz) = (0.0f64, 0.0, 0.0);
    let n = pts.len();
    for i in 0..n {
        let a = &pts[i]; let b = &pts[(i+1)%n];
        nx += (a[1]-b[1]) * (a[2]+b[2]);
        ny += (a[2]-b[2]) * (a[0]+b[0]);
        nz += (a[0]-b[0]) * (a[1]+b[1]);
    }
    (nx, ny, nz)
}

fn lp_merge_collinear(pts: &mut Vec<Point>, vkeys: &mut Vec<usize>) {
    let tol = Tolerance::APPROXIMATION;
    let zt2 = Tolerance::ZERO_TOLERANCE * Tolerance::ZERO_TOLERANCE;
    let mut changed = true;
    while changed {
        changed = false;
        let m = pts.len();
        if m < 3 { break; }
        let mut np: Vec<Point> = Vec::new();
        let mut nk: Vec<usize> = Vec::new();
        for i in 0..m {
            let p = (i+m-1)%m; let nx = (i+1)%m;
            let (ax, ay, az) = (pts[i][0]-pts[p][0], pts[i][1]-pts[p][1], pts[i][2]-pts[p][2]);
            let (bx, by, bz) = (pts[nx][0]-pts[i][0], pts[nx][1]-pts[i][1], pts[nx][2]-pts[i][2]);
            let (cx, cy, cz) = (ay*bz-az*by, az*bx-ax*bz, ax*by-ay*bx);
            let (a2, b2) = (ax*ax+ay*ay+az*az, bx*bx+by*by+bz*bz);
            if a2 < zt2 || b2 < zt2 || cx*cx+cy*cy+cz*cz < tol*tol*a2*b2 {
                changed = true;
            } else {
                np.push(pts[i].clone()); nk.push(vkeys[i]);
            }
        }
        *pts = np; *vkeys = nk;
    }
}

fn lp_offset_toward(p: &Point, cx: f64, cy: f64, cz: f64, gap: f64) -> Point {
    let (mut dx, mut dy, mut dz) = (cx-p[0], cy-p[1], cz-p[2]);
    let len = (dx*dx+dy*dy+dz*dz).sqrt();
    if len > 1e-10 { dx *= gap/len; dy *= gap/len; dz *= gap/len; }
    Point::new(p[0]+dx, p[1]+dy, p[2]+dz)
}

fn lp_face_centroid(m: &Mesh, fk: usize) -> Point {
    let vkeys = m.face_vertices(fk).unwrap();
    let (mut cx, mut cy, mut cz) = (0.0f64, 0.0, 0.0);
    for &vk in vkeys { let p = m.vertex_point(vk).unwrap(); cx += p[0]; cy += p[1]; cz += p[2]; }
    let n = vkeys.len() as f64;
    Point::new(cx/n, cy/n, cz/n)
}

impl Mesh {
    /// Creates a new empty halfedge mesh
    pub fn new() -> Self {
        let mut default_vertex_attributes = HashMap::new();
        default_vertex_attributes.insert("x".to_string(), 0.0);
        default_vertex_attributes.insert("y".to_string(), 0.0);
        default_vertex_attributes.insert("z".to_string(), 0.0);

        Mesh {
            halfedge: HashMap::new(),
            vertex: HashMap::new(),
            face: HashMap::new(),
            facedata: HashMap::new(),
            edgedata: HashMap::new(),
            default_vertex_attributes,
            default_face_attributes: HashMap::new(),
            default_edge_attributes: HashMap::new(),
            triangulation: HashMap::new(),
            face_holes: HashMap::new(),
            max_vertex: 0,
            max_face: 0,
            guid: std::sync::OnceLock::new(),
            name: "my_mesh".to_string(),
            pointcolors: Vec::new(),
            facecolors: Vec::new(),
            linecolors: Vec::new(),
            widths: Vec::new(),
            objectcolor: Color::white(),
            color_mode: ColorMode::OBJECTCOLOR,
            tri_bvh: None,
            tri_tris: Vec::new(),
            tri_vertices: Vec::new(),
            crease_angle_deg: 0.0,
            gpu_cache: crate::render_mesh::GpuCache::default(),
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Construction
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn from_vertices_and_faces(vertices: Vec<Point>, faces: Vec<Vec<usize>>) -> Self {
        let mut mesh = Mesh::new();
        let mut vkeys: Vec<usize> = Vec::with_capacity(vertices.len());
        for pt in vertices {
            vkeys.push(mesh.add_vertex(pt, None));
        }
        for f in faces {
            let mapped: Vec<usize> = f.iter().map(|&i| vkeys[i]).collect();
            mesh.add_face(mapped, None);
        }
        mesh
    }

    pub fn from_polylines(polygons: Vec<Vec<Point>>, precision: Option<f64>) -> Self {
        let mut mesh = Mesh::new();
        let mut map_eps: HashMap<(i64, i64, i64), usize> = HashMap::new();
        let mut map_exact: HashMap<(u64, u64, u64), usize> = HashMap::new();
        let eps = precision.unwrap_or(0.0);
        let use_eps = eps > 0.0;

        let mut get_vkey = |p: &Point, mesh: &mut Mesh| -> usize {
            if use_eps {
                let kx = (p[0] / eps).round() as i64;
                let ky = (p[1] / eps).round() as i64;
                let kz = (p[2] / eps).round() as i64;
                let key = (kx, ky, kz);
                if let Some(&vk) = map_eps.get(&key) {
                    return vk;
                }
                let vk = mesh.add_vertex(p.clone(), None);
                map_eps.insert(key, vk);
                vk
            } else {
                let key = (p[0].to_bits(), p[1].to_bits(), p[2].to_bits());
                if let Some(&vk) = map_exact.get(&key) {
                    return vk;
                }
                let vk = mesh.add_vertex(p.clone(), None);
                map_exact.insert(key, vk);
                vk
            }
        };

        for poly in polygons.into_iter() {
            if poly.len() < 3 {
                continue;
            }
            let mut vkeys: Vec<usize> = Vec::with_capacity(poly.len());
            for p in &poly {
                let vk = get_vkey(p, &mut mesh);
                vkeys.push(vk);
            }
            if vkeys.len() > 1 && *vkeys.last().unwrap() == vkeys[0] {
                vkeys.pop();
            }
            if vkeys.len() < 3 {
                continue;
            }
            let np = vkeys.len();
        if np >= 4 {
            if let Some(fk) = mesh.add_face(vkeys.clone(), None) {
                let (mut nx, mut ny, mut nz) = (0.0f64, 0.0, 0.0);
                for i in 0..np {
                    let a = &poly[i];
                    let b = &poly[(i + 1) % np];
                    nx += (a[1] - b[1]) * (a[2] + b[2]);
                    ny += (a[2] - b[2]) * (a[0] + b[0]);
                    nz += (a[0] - b[0]) * (a[1] + b[1]);
                }
                let nlen = (nx*nx + ny*ny + nz*nz).sqrt();
                if nlen > 1e-12 {
                    nx /= nlen; ny /= nlen; nz /= nlen;
                    let (mut ux, mut uy, mut uz) = (1.0f64, 0.0, 0.0);
                    if nx.abs() > 0.9 { ux = 0.0; uy = 1.0; uz = 0.0; }
                    let dot = ux*nx + uy*ny + uz*nz;
                    ux -= dot*nx; uy -= dot*ny; uz -= dot*nz;
                    let um = (ux*ux + uy*uy + uz*uz).sqrt();
                    ux /= um; uy /= um; uz /= um;
                    let (vx, vy, vz) = (ny*uz - nz*uy, nz*ux - nx*uz, nx*uy - ny*ux);
                    let nk = vkeys.len();
                    let bpts: Vec<Point> = poly[..nk].iter().map(|p| Point::new(
                        p[0]*ux + p[1]*uy + p[2]*uz,
                        p[0]*vx + p[1]*vy + p[2]*vz, 0.0)).collect();
                    let tris = remesh_cdt::cdt_triangulate(&bpts, &[]);
                    let tri_list: Vec<[usize; 3]> = tris.iter().map(|&(a, b, c)| {
                        [vkeys[a], vkeys[b], vkeys[c]]
                    }).collect();
                    mesh.triangulation.insert(fk, tri_list);
                }
            }
        } else {
            let _ = mesh.add_face(vkeys, None);
        }
        }

        mesh
    }

    pub fn from_lines(lines: &[Line], delete_boundary_face: bool, precision: Option<f64>) -> Self {
        if lines.is_empty() {
            return Mesh::new();
        }

        let mut all_pts: Vec<Point> = Vec::with_capacity(lines.len() * 2);
        for ln in lines {
            all_pts.push(ln.start());
            all_pts.push(ln.end());
        }

        let mut eps = precision.unwrap_or(0.0);
        if eps <= 0.0 {
            let (mut minx, mut miny, mut minz) = (all_pts[0][0], all_pts[0][1], all_pts[0][2]);
            let (mut maxx, mut maxy, mut maxz) = (minx, miny, minz);
            for p in &all_pts {
                if p[0] < minx { minx = p[0]; } if p[0] > maxx { maxx = p[0]; }
                if p[1] < miny { miny = p[1]; } if p[1] > maxy { maxy = p[1]; }
                if p[2] < minz { minz = p[2]; } if p[2] > maxz { maxz = p[2]; }
            }
            let diag = ((maxx-minx).powi(2) + (maxy-miny).powi(2) + (maxz-minz).powi(2)).sqrt();
            eps = diag * 1e-6;
            if eps < 1e-12 { eps = 1e-12; }
        }

        let mut vmap: HashMap<(i64, i64, i64), usize> = HashMap::new();
        let mut verts: Vec<Point> = Vec::new();
        let mut get_vid = |p: &Point| -> usize {
            let kx = (p[0] / eps).round() as i64;
            let ky = (p[1] / eps).round() as i64;
            let kz = (p[2] / eps).round() as i64;
            let key = (kx, ky, kz);
            if let Some(&id) = vmap.get(&key) {
                return id;
            }
            let id = verts.len();
            verts.push(p.clone());
            vmap.insert(key, id);
            id
        };

        let mut adj: HashMap<usize, Vec<usize>> = HashMap::new();
        for ln in lines {
            let a = get_vid(&ln.start());
            let b = get_vid(&ln.end());
            if a == b { continue; }
            adj.entry(a).or_default().push(b);
            adj.entry(b).or_default().push(a);
        }

        let nv = verts.len();

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

        let mut visited: HashSet<(usize, usize)> = HashSet::new();
        let mut face_cycles: Vec<Vec<usize>> = Vec::new();

        let adj_keys: Vec<usize> = {
            let mut keys: Vec<usize> = adj.keys().copied().collect();
            keys.sort();
            keys
        };

        for &u in &adj_keys {
            let nbrs: Vec<usize> = adj.get(&u).cloned().unwrap_or_default();
            for &v in &nbrs {
                if visited.contains(&(u, v)) { continue; }
                let mut cycle: Vec<usize> = Vec::new();
                let (mut cu, mut cv) = (u, v);
                let mut valid = true;
                loop {
                    if visited.contains(&(cu, cv)) { break; }
                    visited.insert((cu, cv));
                    cycle.push(cu);
                    let cv_nbrs = match adj.get(&cv) {
                        Some(n) => n,
                        None => { valid = false; break; }
                    };
                    let idx = match cv_nbrs.iter().position(|&x| x == cu) {
                        Some(i) => i,
                        None => { valid = false; break; }
                    };
                    let prev_idx = if idx == 0 { cv_nbrs.len() - 1 } else { idx - 1 };
                    let nxt = cv_nbrs[prev_idx];
                    cu = cv;
                    cv = nxt;
                    if cycle.len() > nv * 2 { valid = false; break; }
                }
                if valid && cycle.len() >= 3 {
                    face_cycles.push(cycle);
                }
            }
        }

        if delete_boundary_face && !face_cycles.is_empty() {
            let mut min_idx = 0;
            let mut min_area = f64::MAX;
            for (i, cyc) in face_cycles.iter().enumerate() {
                let cn = cyc.len();
                let mut area = 0.0_f64;
                for j in 0..cn {
                    let a = cyc[j];
                    let b = cyc[(j+1)%cn];
                    area += verts[a][0] * verts[b][1] - verts[b][0] * verts[a][1];
                }
                area *= 0.5;
                if area < min_area { min_area = area; min_idx = i; }
            }
            face_cycles.remove(min_idx);
        }

        let mut mesh = Mesh::new();
        let mut vkeys: Vec<usize> = Vec::with_capacity(verts.len());
        for pt in &verts {
            vkeys.push(mesh.add_vertex(pt.clone(), None));
        }
        for cycle in &face_cycles {
            let fvkeys: Vec<usize> = cycle.iter().map(|&i| vkeys[i]).collect();
            if let Some(fkey) = mesh.add_face(fvkeys, None) {
                let mut ordered: Vec<usize> = cycle.clone();
                let mut bpts: Vec<Point> = ordered.iter().map(|&i| Point::new(verts[i][0], verts[i][1], 0.0)).collect();
                let area: f64 = (0..bpts.len()).map(|j| {
                    let k = (j + 1) % bpts.len();
                    bpts[j][0] * bpts[k][1] - bpts[k][0] * bpts[j][1]
                }).sum::<f64>() * 0.5;
                if area < 0.0 { bpts.reverse(); ordered.reverse(); }
                let tris = remesh_cdt::cdt_triangulate(&bpts, &[]);
                let tri_list: Vec<[usize; 3]> = tris.iter().map(|&(a, b, c)| {
                    [vkeys[ordered[a]], vkeys[ordered[b]], vkeys[ordered[c]]]
                }).collect();
                mesh.triangulation.insert(fkey, tri_list);
            }
        }
        mesh
    }

    pub fn from_polygon_with_holes(raw: &[Vec<Point>], sort_by_bbox: bool) -> Self {
        let polylines: Vec<Polyline> = raw.iter().map(|v| Polyline::new(v.clone())).collect();
        Mesh::_from_polygon_with_holes_pl(&polylines, sort_by_bbox)
    }

    fn _from_polygon_with_holes_pl(polylines: &[Polyline], sort_by_bbox: bool) -> Self {
        crate::remesh_cdt::RemeshCDT::from_polylines(polylines, false, !sort_by_bbox)
    }

    pub fn loft(polylines0: &[Polyline], polylines1: &[Polyline], cap: bool, fix_collinear: bool) -> Self {
        if polylines0.is_empty() || polylines1.is_empty() || polylines0.len() != polylines1.len() {
            return Mesh::new();
        }
        let mut border_idx = 0usize;
        let mut max_diag = 0.0_f64;
        for (i, pl) in polylines0.iter().enumerate() {
            let pts = pl.get_points();
            if pts.is_empty() { continue; }
            let (mut minx, mut miny, mut minz) = (pts[0][0], pts[0][1], pts[0][2]);
            let (mut maxx, mut maxy, mut maxz) = (minx, miny, minz);
            for p in &pts {
                if p[0] < minx { minx = p[0]; } if p[0] > maxx { maxx = p[0]; }
                if p[1] < miny { miny = p[1]; } if p[1] > maxy { maxy = p[1]; }
                if p[2] < minz { minz = p[2]; } if p[2] > maxz { maxz = p[2]; }
            }
            let (dx, dy, dz) = (maxx-minx, maxy-miny, maxz-minz);
            let diag = (dx*dx + dy*dy + dz*dz).sqrt();
            if diag > max_diag { max_diag = diag; border_idx = i; }
        }
        let get_open = |pl: &Polyline| -> Vec<Point> {
            let mut pts = pl.get_points();
            if pts.len() > 1 {
                let f = pts[0].clone(); let b = pts[pts.len()-1].clone();
                if (f[0]-b[0]).abs() < 1e-12 && (f[1]-b[1]).abs() < 1e-12 && (f[2]-b[2]).abs() < 1e-12 {
                    pts.pop();
                }
            }
            pts
        };
        let (origin, xaxis, mut yaxis, zaxis) = polylines0[border_idx].get_average_plane();
        {
            let c0 = polylines0[border_idx].center();
            let c1 = polylines1[border_idx].center();
            let btt = Vector::new(c1[0]-c0[0], c1[1]-c0[1], c1[2]-c0[2]);
            if zaxis.dot(&btt) < 0.0 {
                yaxis = Vector::new(-yaxis[0], -yaxis[1], -yaxis[2]);
            }
        }
        let proj = |p: &Point| -> Point {
            let dx = p[0]-origin[0]; let dy = p[1]-origin[1]; let dz = p[2]-origin[2];
            Point::new(dx*xaxis[0]+dy*xaxis[1]+dz*xaxis[2], dx*yaxis[0]+dy*yaxis[1]+dz*yaxis[2], 0.0)
        };
        let sarea = |pts: &[Point]| -> f64 {
            let n = pts.len();
            let mut a = 0.0;
            for i in 0..n {
                let j = (i+1) % n;
                let pi = proj(&pts[i]); let pj = proj(&pts[j]);
                a += pi[0]*pj[1] - pj[0]*pi[1];
            }
            a * 0.5
        };
        let mut order: Vec<usize> = vec![border_idx];
        for i in 0..polylines0.len() { if i != border_idx { order.push(i); } }
        let mut poly_infos: Vec<(usize, usize, usize, usize)> = Vec::new(); // (bot_off, bot_n, top_off, top_n)
        let mut all_bot: Vec<Point> = Vec::new();
        let mut all_top: Vec<Point> = Vec::new();
        let strip_shared_collinear = |bot: &mut Vec<Point>, top: &mut Vec<Point>| {
            let cdt_scale = 1.0e6_f64;
            let cross_q = |a: &Point, b: &Point, c: &Point| -> i64 {
                let pa = proj(a); let pb = proj(b); let pc = proj(c);
                let iax = (pa[0] * cdt_scale).round() as i64;
                let iay = (pa[1] * cdt_scale).round() as i64;
                let ibx = (pb[0] * cdt_scale).round() as i64;
                let iby = (pb[1] * cdt_scale).round() as i64;
                let icx = (pc[0] * cdt_scale).round() as i64;
                let icy = (pc[1] * cdt_scale).round() as i64;
                (ibx - iax) * (icy - iay) - (iby - iay) * (icx - iax)
            };
            let mut changed = true;
            while changed && bot.len() > 3 {
                changed = false;
                let n = bot.len();
                for i in 0..n {
                    let prev = (i + n - 1) % n;
                    let next = (i + 1) % n;
                    if cross_q(&bot[prev], &bot[i], &bot[next]) == 0
                        && cross_q(&top[prev], &top[i], &top[next]) == 0
                    {
                        bot.remove(i);
                        top.remove(i);
                        changed = true;
                        break;
                    }
                }
            }
        };
        for (oi, &idx) in order.iter().enumerate() {
            let mut bot = get_open(&polylines0[idx]);
            let mut top = get_open(&polylines1[idx]);
            let area = sarea(&bot);
            if (oi == 0 && area < 0.0) || (oi != 0 && area > 0.0) {
                bot.reverse(); top.reverse();
            }
            if bot.len() == top.len() { strip_shared_collinear(&mut bot, &mut top); }
            poly_infos.push((all_bot.len(), bot.len(), all_top.len(), top.len()));
            all_bot.extend(bot.into_iter());
            all_top.extend(top.into_iter());
        }
        let mut mesh = Mesh::new();
        let bvk: Vec<usize> = all_bot.iter().map(|p| mesh.add_vertex(p.clone(), None)).collect();
        let tvk: Vec<usize> = all_top.iter().map(|p| mesh.add_vertex(p.clone(), None)).collect();
        if cap {
            let (_, bot_n0, _, top_n0) = poly_infos[0];
            // Bottom cap CDT
            let b2d: Vec<Point> = (0..bot_n0).map(|i| proj(&all_bot[i])).collect();
            let bh2d: Vec<Vec<Point>> = poly_infos[1..].iter().map(|&(off,cnt,_,_)| {
                (off..off+cnt).map(|i| proj(&all_bot[i])).collect()
            }).collect();
            let b_tris = remesh_cdt::cdt_triangulate(&b2d, &bh2d);
            let bot_fvkeys: Vec<usize> = (0..bot_n0).rev().map(|i| bvk[i]).collect();
            if let Some(fk_bot) = mesh.add_face(bot_fvkeys, None) {
                if !bh2d.is_empty() {
                    let hole_rings: Vec<Vec<usize>> = poly_infos[1..].iter()
                        .map(|&(off,cnt,_,_)| (off..off+cnt).map(|i| bvk[i]).collect())
                        .collect();
                    mesh.face_holes.insert(fk_bot, hole_rings);
                }
                let mut tri_list: Vec<[usize;3]> = b_tris.iter().map(|&(a,b,c)| [bvk[a], bvk[c], bvk[b]]).collect();
                if fix_collinear {
                    let vk2d: std::collections::HashMap<usize,(f64,f64)> = (0..bot_n0).map(|i| { let p = proj(&all_bot[i]); (bvk[i], (p[0],p[1])) }).collect();
                    let fv: Vec<usize> = (0..bot_n0).rev().map(|i| bvk[i]).collect();
                    let mut chg = true;
                    while chg {
                        chg = false;
                        let tv: std::collections::HashSet<usize> = tri_list.iter().flat_map(|t| t.iter().copied()).collect();
                        let n = fv.len();
                        'outer: for k in 0..n {
                            let b_vk = fv[k];
                            if tv.contains(&b_vk) { continue; }
                            let a_vk = fv[(k+n-1)%n]; let c_vk = fv[(k+1)%n];
                            for j in 0..tri_list.len() {
                                let ha = tri_list[j].contains(&a_vk); let hc = tri_list[j].contains(&c_vk);
                                if !ha || !hc { continue; }
                                let ft = tri_list[j];
                                let (t1, t2) = if (ft[0]==a_vk||ft[0]==c_vk) && (ft[1]==a_vk||ft[1]==c_vk) {
                                    ([ft[0],b_vk,ft[2]], [b_vk,ft[1],ft[2]])
                                } else if (ft[1]==a_vk||ft[1]==c_vk) && (ft[2]==a_vk||ft[2]==c_vk) {
                                    ([ft[0],ft[1],b_vk], [ft[0],b_vk,ft[2]])
                                } else {
                                    ([ft[0],ft[1],b_vk], [b_vk,ft[1],ft[2]])
                                };
                                tri_list[j] = t1; tri_list.push(t2); chg = true; break 'outer;
                            }
                        }
                    }
                    let sc = 1e6_f64;
                    tri_list.retain(|t| {
                        let (u0,v0) = vk2d.get(&t[0]).copied().unwrap_or((0.0,0.0));
                        let (u1,v1) = vk2d.get(&t[1]).copied().unwrap_or((0.0,0.0));
                        let (u2,v2) = vk2d.get(&t[2]).copied().unwrap_or((0.0,0.0));
                        let iu0=(u0*sc).round() as i64; let iv0=(v0*sc).round() as i64;
                        let iu1=(u1*sc).round() as i64; let iv1=(v1*sc).round() as i64;
                        let iu2=(u2*sc).round() as i64; let iv2=(v2*sc).round() as i64;
                        (iu1-iu0)*(iv2-iv0)-(iv1-iv0)*(iu2-iu0) != 0
                    });
                }
                mesh.triangulation.insert(fk_bot, tri_list);
            }
            // Top cap CDT
            let t2d: Vec<Point> = (0..top_n0).map(|i| proj(&all_top[i])).collect();
            let th2d: Vec<Vec<Point>> = poly_infos[1..].iter().map(|&(_,_,off,cnt)| {
                (off..off+cnt).map(|i| proj(&all_top[i])).collect()
            }).collect();
            let t_tris = remesh_cdt::cdt_triangulate(&t2d, &th2d);
            let top_fvkeys: Vec<usize> = (0..top_n0).map(|i| tvk[i]).collect();
            if let Some(fk_top) = mesh.add_face(top_fvkeys, None) {
                if !th2d.is_empty() {
                    let hole_rings: Vec<Vec<usize>> = poly_infos[1..].iter()
                        .map(|&(_,_,off,cnt)| (off..off+cnt).map(|i| tvk[i]).collect())
                        .collect();
                    mesh.face_holes.insert(fk_top, hole_rings);
                }
                let mut tri_list: Vec<[usize;3]> = t_tris.iter().map(|&(a,b,c)| [tvk[a], tvk[b], tvk[c]]).collect();
                if fix_collinear {
                    let vk2d: std::collections::HashMap<usize,(f64,f64)> = (0..top_n0).map(|i| { let p = proj(&all_top[i]); (tvk[i], (p[0],p[1])) }).collect();
                    let fv: Vec<usize> = (0..top_n0).map(|i| tvk[i]).collect();
                    let mut chg = true;
                    while chg {
                        chg = false;
                        let tv: std::collections::HashSet<usize> = tri_list.iter().flat_map(|t| t.iter().copied()).collect();
                        let n = fv.len();
                        'outer: for k in 0..n {
                            let b_vk = fv[k];
                            if tv.contains(&b_vk) { continue; }
                            let a_vk = fv[(k+n-1)%n]; let c_vk = fv[(k+1)%n];
                            for j in 0..tri_list.len() {
                                let ha = tri_list[j].contains(&a_vk); let hc = tri_list[j].contains(&c_vk);
                                if !ha || !hc { continue; }
                                let ft = tri_list[j];
                                let (t1, t2) = if (ft[0]==a_vk||ft[0]==c_vk) && (ft[1]==a_vk||ft[1]==c_vk) {
                                    ([ft[0],b_vk,ft[2]], [b_vk,ft[1],ft[2]])
                                } else if (ft[1]==a_vk||ft[1]==c_vk) && (ft[2]==a_vk||ft[2]==c_vk) {
                                    ([ft[0],ft[1],b_vk], [ft[0],b_vk,ft[2]])
                                } else {
                                    ([ft[0],ft[1],b_vk], [b_vk,ft[1],ft[2]])
                                };
                                tri_list[j] = t1; tri_list.push(t2); chg = true; break 'outer;
                            }
                        }
                    }
                    let sc = 1e6_f64;
                    tri_list.retain(|t| {
                        let (u0,v0) = vk2d.get(&t[0]).copied().unwrap_or((0.0,0.0));
                        let (u1,v1) = vk2d.get(&t[1]).copied().unwrap_or((0.0,0.0));
                        let (u2,v2) = vk2d.get(&t[2]).copied().unwrap_or((0.0,0.0));
                        let iu0=(u0*sc).round() as i64; let iv0=(v0*sc).round() as i64;
                        let iu1=(u1*sc).round() as i64; let iv1=(v1*sc).round() as i64;
                        let iu2=(u2*sc).round() as i64; let iv2=(v2*sc).round() as i64;
                        (iu1-iu0)*(iv2-iv0)-(iv1-iv0)*(iu2-iu0) != 0
                    });
                }
                mesh.triangulation.insert(fk_top, tri_list);
            }
        }
        // Side faces: align by longest edge, quads for equal counts, zipper+triangles otherwise
        let edsq = |pts: &[Point], i: usize| -> f64 {
            let j = (i + 1) % pts.len();
            let (dx, dy, dz) = (pts[j][0]-pts[i][0], pts[j][1]-pts[i][1], pts[j][2]-pts[i][2]);
            dx*dx + dy*dy + dz*dz
        };
        for &(bot_off, bot_n, top_off, top_n) in &poly_infos {
            let bpts = &all_bot[bot_off..bot_off+bot_n];
            let tpts = &all_top[top_off..top_off+top_n];
            let ia = (0..bot_n).max_by(|&a, &b| edsq(bpts, a).partial_cmp(&edsq(bpts, b)).unwrap()).unwrap_or(0);
            let ib = if bot_n == top_n {
                let align_cost = |cand: usize| -> f64 {
                    (0..bot_n).map(|k| {
                        let pb = proj(&bpts[(ia+k)%bot_n]);
                        let pt = proj(&tpts[(cand+k)%top_n]);
                        (pt[0]-pb[0])*(pt[0]-pb[0]) + (pt[1]-pb[1])*(pt[1]-pb[1])
                    }).sum::<f64>()
                };
                (0..top_n).min_by(|&a, &b| align_cost(a).partial_cmp(&align_cost(b)).unwrap()).unwrap_or(0)
            } else { 0 };
            if bot_n == top_n {
                for k in 0..bot_n {
                    let (cb, ct) = (bot_off+(ia+k)%bot_n, top_off+(ib+k)%top_n);
                    let (nb, nt) = (bot_off+(ia+k+1)%bot_n, top_off+(ib+k+1)%top_n);
                    mesh.add_face(vec![bvk[cb], bvk[nb], tvk[nt], tvk[ct]], None);
                }
                continue;
            }
            let mut b_arcs = vec![0.0_f64; bot_n+1];
            for k in 0..bot_n {
                let (i, j) = ((ia+k)%bot_n, (ia+k+1)%bot_n);
                let (dx, dy, dz) = (bpts[j][0]-bpts[i][0], bpts[j][1]-bpts[i][1], bpts[j][2]-bpts[i][2]);
                b_arcs[k+1] = b_arcs[k] + (dx*dx+dy*dy+dz*dz).sqrt();
            }
            let mut t_arcs = vec![0.0_f64; top_n+1];
            for k in 0..top_n {
                let (i, j) = ((ib+k)%top_n, (ib+k+1)%top_n);
                let (dx, dy, dz) = (tpts[j][0]-tpts[i][0], tpts[j][1]-tpts[i][1], tpts[j][2]-tpts[i][2]);
                t_arcs[k+1] = t_arcs[k] + (dx*dx+dy*dy+dz*dz).sqrt();
            }
            let inv_b = if b_arcs[bot_n] > 0.0 { 1.0/b_arcs[bot_n] } else { 1.0 };
            let inv_t = if t_arcs[top_n] > 0.0 { 1.0/t_arcs[top_n] } else { 1.0 };
            let (mut bi, mut ti) = (0usize, 0usize);
            while bi < bot_n || ti < top_n {
                let (cb, ct) = (bot_off+(ia+bi)%bot_n, top_off+(ib+ti)%top_n);
                let (nb, nt) = (bot_off+(ia+bi+1)%bot_n, top_off+(ib+ti+1)%top_n);
                if bi >= bot_n {
                    mesh.add_face(vec![bvk[cb], tvk[ct], tvk[nt]], None); ti += 1;
                } else if ti >= top_n {
                    mesh.add_face(vec![bvk[cb], bvk[nb], tvk[ct]], None); bi += 1;
                } else {
                    let (bp, tp) = (b_arcs[bi+1]*inv_b, t_arcs[ti+1]*inv_t);
                    if (bp-tp).abs() < 1e-9 {
                        mesh.add_face(vec![bvk[cb], bvk[nb], tvk[nt], tvk[ct]], None); bi += 1; ti += 1;
                    } else if bp < tp {
                        mesh.add_face(vec![bvk[cb], bvk[nb], tvk[ct]], None); bi += 1;
                    } else {
                        mesh.add_face(vec![bvk[cb], tvk[ct], tvk[nt]], None); ti += 1;
                    }
                }
            }
        }
        mesh
    }

    pub fn from_polygon_with_holes_many(inputs: Vec<Vec<Vec<Point>>>, sort_by_bbox: bool, parallel: bool) -> Vec<Self> {
        if parallel && inputs.len() > 1 {
            use rayon::prelude::*;
            inputs.into_par_iter().map(|input| Mesh::from_polygon_with_holes(&input, sort_by_bbox)).collect()
        } else {
            inputs.iter().map(|input| Mesh::from_polygon_with_holes(input, sort_by_bbox)).collect()
        }
    }

    pub fn loft_many(pairs: Vec<(Vec<Polyline>, Vec<Polyline>)>, cap: bool, parallel: bool, fix_collinear: bool) -> Vec<Self> {
        if parallel && pairs.len() > 1 {
            use rayon::prelude::*;
            pairs.into_par_iter().map(|(p0, p1)| Mesh::loft(&p0, &p1, cap, fix_collinear)).collect()
        } else {
            pairs.iter().map(|(p0, p1)| Mesh::loft(p0, p1, cap, fix_collinear)).collect()
        }
    }

    pub fn loft_panels(
        top_polygons: Vec<Vec<Point>>,
        bot_polygons: Vec<Vec<Point>>,
        merge_precision: f64,
        edge_gap: f64,
        edge_match_threshold: f64,
        add_caps: bool,
        skip_triangles: bool,
    ) -> (Vec<LoftPanel>, Vec<LoftAdjPair>, Mesh, Mesh) {
        let top_mesh = Mesh::from_polylines(top_polygons, Some(merge_precision));
        let bot_mesh = Mesh::from_polylines(bot_polygons, Some(merge_precision));
        let tfks: Vec<usize> = top_mesh.face.keys().cloned().collect();
        let bfks: Vec<usize> = bot_mesh.face.keys().cloned().collect();
        let mut dists: Vec<(f64, usize, usize)> = Vec::with_capacity(tfks.len() * bfks.len());
        for ti in 0..tfks.len() {
            for bi in 0..bfks.len() {
                let d = lp_face_centroid(&top_mesh, tfks[ti]).distance(&lp_face_centroid(&bot_mesh, bfks[bi]), None);
                dists.push((d, ti, bi));
            }
        }
        dists.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let mut top_used = vec![false; tfks.len()];
        let mut bot_used = vec![false; bfks.len()];
        let mut face_match: Vec<(usize, usize)> = Vec::new();
        for &(_, ti, bi) in &dists {
            if top_used[ti] || bot_used[bi] { continue; }
            face_match.push((tfks[ti], bfks[bi]));
            top_used[ti] = true; bot_used[bi] = true;
        }
        face_match.sort();
        let mut panels: Vec<LoftPanel> = Vec::with_capacity(face_match.len());
        for (tfk, bfk) in face_match {
            let mut panel = LoftPanel {
                mesh: Mesh::new(), top_face_key: 0, bot_face_key: 0,
                wall_faces: Vec::new(), orig_top_to_local: HashMap::new(),
                orig_bot_to_local: HashMap::new(), top_vertices: Vec::new(), bot_vertices: Vec::new(),
                face_roles: HashMap::new(),
            };
            let mut top_vkeys: Vec<usize> = top_mesh.face_vertices(tfk).unwrap().clone();
            let mut bot_vkeys: Vec<usize> = bot_mesh.face_vertices(bfk).unwrap().clone();
            let mut top_pts: Vec<Point> = top_vkeys.iter().map(|&vk| top_mesh.vertex_point(vk).unwrap()).collect();
            let mut bot_pts: Vec<Point> = bot_vkeys.iter().map(|&vk| bot_mesh.vertex_point(vk).unwrap()).collect();
            lp_merge_collinear(&mut top_pts, &mut top_vkeys);
            lp_merge_collinear(&mut bot_pts, &mut bot_vkeys);
            {
                let sz = top_pts.len();
                let mut max_te = 0.0f64;
                for i in 0..sz { max_te = max_te.max(top_pts[i].distance(&top_pts[(i+1)%sz], None)); }
                let stol = max_te * 0.001;
                let mut tp: Vec<Point> = Vec::new(); let mut tk: Vec<usize> = Vec::new();
                for i in 0..top_pts.len() {
                    if tp.is_empty() || tp.last().unwrap().distance(&top_pts[i], None) > stol {
                        tp.push(top_pts[i].clone()); tk.push(top_vkeys[i]);
                    }
                }
                while tp.len() >= 3 && tp.last().unwrap().distance(&tp[0], None) <= stol { tp.pop(); tk.pop(); }
                if tp.len() >= 3 { top_pts = tp; top_vkeys = tk; }
            }
            let n = top_pts.len(); let m = bot_pts.len();
            let (mut tcx, mut tcy, mut tcz) = (0.0f64, 0.0, 0.0);
            let (mut bcx, mut bcy, mut bcz) = (0.0f64, 0.0, 0.0);
            for p in &top_pts { tcx += p[0]; tcy += p[1]; tcz += p[2]; }
            for p in &bot_pts { bcx += p[0]; bcy += p[1]; bcz += p[2]; }
            tcx /= n as f64; tcy /= n as f64; tcz /= n as f64;
            bcx /= m as f64; bcy /= m as f64; bcz /= m as f64;
            let (mut ax, mut ay, mut az) = (tcx-bcx, tcy-bcy, tcz-bcz);
            let alen = (ax*ax+ay*ay+az*az).sqrt();
            if alen > 1e-12 { ax /= alen; ay /= alen; az /= alen; }
            let (tnx, tny, tnz) = lp_newell_normal(&top_pts);
            if tnx*ax+tny*ay+tnz*az < 0.0 { top_pts.reverse(); top_vkeys.reverse(); }
            let (bnx, bny, bnz) = lp_newell_normal(&bot_pts);
            if bnx*ax+bny*ay+bnz*az > 0.0 { bot_pts.reverse(); bot_vkeys.reverse(); }
            for i in 0..n {
                let lk = panel.mesh.add_vertex(top_pts[i].clone(), None);
                panel.orig_top_to_local.insert(top_vkeys[i], lk);
                panel.top_vertices.push(lk);
            }
            for j in 0..m {
                let lk = panel.mesh.add_vertex(bot_pts[j].clone(), None);
                panel.orig_bot_to_local.insert(bot_vkeys[j], lk);
                panel.bot_vertices.push(lk);
            }
            if add_caps {
                let top_cap: Vec<usize> = top_vkeys.iter().map(|&vk| panel.orig_top_to_local[&vk]).collect();
                let top_cap_fk = panel.mesh.add_face(top_cap.clone(), None);
                if let Some(fk) = top_cap_fk { panel.top_face_key = fk; }
                if let Some(fk) = top_cap_fk {
                    if top_cap.len() >= 3 {
                        let (mut nx, mut ny, mut nz) = lp_newell_normal(&top_pts);
                        let mag = (nx*nx+ny*ny+nz*nz).sqrt();
                        if mag > 1e-12 {
                            nx /= mag; ny /= mag; nz /= mag;
                            let (mut ux, mut uy, mut uz) = if nx.abs() > 0.9 { (0.0f64, 1.0, 0.0) } else { (1.0f64, 0.0, 0.0) };
                            let dot = ux*nx+uy*ny+uz*nz;
                            ux -= dot*nx; uy -= dot*ny; uz -= dot*nz;
                            let um = (ux*ux+uy*uy+uz*uz).sqrt(); ux /= um; uy /= um; uz /= um;
                            let (vx, vy, vz) = (ny*uz-nz*uy, nz*ux-nx*uz, nx*uy-ny*ux);
                            let bpts: Vec<Point> = top_pts.iter().map(|p| Point::new(p[0]*ux+p[1]*uy+p[2]*uz, p[0]*vx+p[1]*vy+p[2]*vz, 0.0)).collect();
                            let tris = remesh_cdt::cdt_triangulate(&bpts, &[]);
                            if !tris.is_empty() {
                                let tri_list: Vec<[usize;3]> = tris.iter().map(|&(a,b,c)| [top_cap[a], top_cap[b], top_cap[c]]).collect();
                                panel.mesh.triangulation.insert(fk, tri_list);
                            }
                        }
                    }
                }
            }
            let top_mids: Vec<Point> = (0..n).map(|i| Point::new(
                (top_pts[i][0]+top_pts[(i+1)%n][0])*0.5,
                (top_pts[i][1]+top_pts[(i+1)%n][1])*0.5,
                (top_pts[i][2]+top_pts[(i+1)%n][2])*0.5)).collect();
            let bot_mids: Vec<Point> = (0..m).map(|j| Point::new(
                (bot_pts[j][0]+bot_pts[(j+1)%m][0])*0.5,
                (bot_pts[j][1]+bot_pts[(j+1)%m][1])*0.5,
                (bot_pts[j][2]+bot_pts[(j+1)%m][2])*0.5)).collect();
            let mut bot_to_top = vec![-1i64; m]; let mut bot_dist = vec![f64::INFINITY; m];
            for j in 0..m {
                for i in 0..n {
                    let d = bot_mids[j].distance(&top_mids[i], None);
                    if d < bot_dist[j] { bot_dist[j] = d; bot_to_top[j] = i as i64; }
                }
            }
            let mut top_to_bot = vec![-1i64; n]; let mut top_dist = vec![f64::INFINITY; n];
            for i in 0..n {
                for j in 0..m {
                    let d = top_mids[i].distance(&bot_mids[j], None);
                    if d < top_dist[i] { top_dist[i] = d; top_to_bot[i] = j as i64; }
                }
            }
            let avg: f64 = bot_dist.iter().sum::<f64>() / m as f64;
            let threshold = avg * edge_match_threshold;
            let mut top_used_edge = vec![false; n];
            for j in 0..m {
                let b0 = panel.orig_bot_to_local[&bot_vkeys[j]];
                let b1 = panel.orig_bot_to_local[&bot_vkeys[(j+1)%m]];
                let ti = bot_to_top[j];
                if ti >= 0 && bot_dist[j] <= threshold && top_to_bot[ti as usize] == j as i64 {
                    let ti = ti as usize;
                    let t0 = panel.orig_top_to_local[&top_vkeys[ti]];
                    let t1 = panel.orig_top_to_local[&top_vkeys[(ti+1)%n]];
                    let face_fk = if edge_gap > 0.0 {
                        let pb0 = panel.mesh.vertex_point(b0).unwrap();
                        let pb1 = panel.mesh.vertex_point(b1).unwrap();
                        let pt0 = panel.mesh.vertex_point(t0).unwrap();
                        let pt1 = panel.mesh.vertex_point(t1).unwrap();
                        let cx = (pb0[0]+pb1[0]+pt0[0]+pt1[0])*0.25;
                        let cy = (pb0[1]+pb1[1]+pt0[1]+pt1[1])*0.25;
                        let cz = (pb0[2]+pb1[2]+pt0[2]+pt1[2])*0.25;
                        let nb0 = panel.mesh.add_vertex(lp_offset_toward(&pb0, cx, cy, cz, edge_gap), None);
                        let nb1 = panel.mesh.add_vertex(lp_offset_toward(&pb1, cx, cy, cz, edge_gap), None);
                        panel.mesh.add_face(vec![nb0, t1, t0, nb1], None)
                    } else {
                        panel.mesh.add_face(vec![b0, t1, t0, b1], None)
                    };
                    if let Some(fk) = face_fk {
                        panel.wall_faces.push(LoftWallFace {
                            face_key: fk, face_index: 0, is_quad: true,
                            top_v0: top_vkeys[ti], top_v1: top_vkeys[(ti+1)%n],
                            bot_v0: bot_vkeys[(j+1)%m], bot_v1: bot_vkeys[j],
                        });
                    }
                    top_used_edge[ti] = true;
                } else if !skip_triangles {
                    let mut best_d = f64::INFINITY; let mut best_tv = 0usize;
                    for i in 0..n {
                        let d = bot_mids[j].distance(&top_pts[i], None);
                        if d < best_d { best_d = d; best_tv = i; }
                    }
                    let tv = panel.orig_top_to_local[&top_vkeys[best_tv]];
                    if let Some(fk) = panel.mesh.add_face(vec![b0, tv, b1], None) {
                        panel.wall_faces.push(LoftWallFace { face_key: fk, face_index: 0, is_quad: false, top_v0: 0, top_v1: 0, bot_v0: 0, bot_v1: 0 });
                    }
                }
            }
            if !skip_triangles {
                for i in 0..n {
                    if top_used_edge[i] { continue; }
                    let t0 = panel.orig_top_to_local[&top_vkeys[i]];
                    let t1 = panel.orig_top_to_local[&top_vkeys[(i+1)%n]];
                    let mut best_d = f64::INFINITY; let mut best_bv = 0usize;
                    for j in 0..m {
                        let d = top_mids[i].distance(&bot_pts[j], None);
                        if d < best_d { best_d = d; best_bv = j; }
                    }
                    let bv = panel.orig_bot_to_local[&bot_vkeys[best_bv]];
                    if let Some(fk) = panel.mesh.add_face(vec![t1, t0, bv], None) {
                        panel.wall_faces.push(LoftWallFace { face_key: fk, face_index: 0, is_quad: false, top_v0: 0, top_v1: 0, bot_v0: 0, bot_v1: 0 });
                    }
                }
            }
            if add_caps {
                let bot_cap: Vec<usize> = (0..m).map(|j| panel.orig_bot_to_local[&bot_vkeys[j]]).collect();
                let bot_cap_fk = panel.mesh.add_face(bot_cap.clone(), None);
                if let Some(fk) = bot_cap_fk { panel.bot_face_key = fk; }
                if let Some(fk) = bot_cap_fk {
                    if bot_cap.len() >= 3 {
                        let (mut bcnx, mut bcny, mut bcnz) = lp_newell_normal(&bot_pts);
                        let bcmag = (bcnx*bcnx+bcny*bcny+bcnz*bcnz).sqrt();
                        if bcmag > 1e-12 {
                            bcnx /= bcmag; bcny /= bcmag; bcnz /= bcmag;
                            let (mut bcux, mut bcuy, mut bcuz) = if bcnx.abs() > 0.9 { (0.0f64, 1.0, 0.0) } else { (1.0f64, 0.0, 0.0) };
                            let bcdot = bcux*bcnx+bcuy*bcny+bcuz*bcnz;
                            bcux -= bcdot*bcnx; bcuy -= bcdot*bcny; bcuz -= bcdot*bcnz;
                            let bcum = (bcux*bcux+bcuy*bcuy+bcuz*bcuz).sqrt();
                            bcux /= bcum; bcuy /= bcum; bcuz /= bcum;
                            let (bcvx, bcvy, bcvz) = (bcny*bcuz-bcnz*bcuy, bcnz*bcux-bcnx*bcuz, bcnx*bcuy-bcny*bcux);
                            let bpts2: Vec<Point> = bot_pts.iter().map(|p| Point::new(p[0]*bcux+p[1]*bcuy+p[2]*bcuz, p[0]*bcvx+p[1]*bcvy+p[2]*bcvz, 0.0)).collect();
                            let btris = remesh_cdt::cdt_triangulate(&bpts2, &[]);
                            if !btris.is_empty() {
                                let tri_list: Vec<[usize;3]> = btris.iter().map(|&(a,b,c)| [bot_cap[a], bot_cap[b], bot_cap[c]]).collect();
                                panel.mesh.triangulation.insert(fk, tri_list);
                            }
                        }
                    }
                }
            }
            panels.push(panel);
        }
        for pi in 0..panels.len() {
            let mut fkey_to_idx: HashMap<usize, usize> = HashMap::new();
            for (fi, (&fk, _)) in panels[pi].mesh.face.iter().enumerate() {
                fkey_to_idx.insert(fk, fi);
            }
            for wi in 0..panels[pi].wall_faces.len() {
                let fk = panels[pi].wall_faces[wi].face_key;
                panels[pi].wall_faces[wi].face_index = *fkey_to_idx.get(&fk).unwrap();
                let role = if panels[pi].wall_faces[wi].is_quad { "QuadWall" } else { "TriWall" };
                panels[pi].face_roles.insert(fk, role);
            }
            let tfk = panels[pi].top_face_key;
            if tfk != 0 { panels[pi].face_roles.insert(tfk, "TopCap"); }
            let bfk = panels[pi].bot_face_key;
            if bfk != 0 { panels[pi].face_roles.insert(bfk, "BotCap"); }
        }
        let mut edge_to_wall: HashMap<(usize, usize), (usize, usize)> = HashMap::new();
        for pi in 0..panels.len() {
            for wi in 0..panels[pi].wall_faces.len() {
                if !panels[pi].wall_faces[wi].is_quad { continue; }
                let v0 = panels[pi].wall_faces[wi].top_v0;
                let v1 = panels[pi].wall_faces[wi].top_v1;
                edge_to_wall.insert((v0, v1), (pi, wi));
            }
        }
        let mut adjacency: Vec<LoftAdjPair> = Vec::new();
        for pi in 0..panels.len() {
            for wi in 0..panels[pi].wall_faces.len() {
                if !panels[pi].wall_faces[wi].is_quad { continue; }
                let v0 = panels[pi].wall_faces[wi].top_v0;
                let v1 = panels[pi].wall_faces[wi].top_v1;
                if let Some(&(pj, wj)) = edge_to_wall.get(&(v1, v0)) {
                    if pj > pi { adjacency.push(LoftAdjPair { pi, wi, pj, wj }); }
                }
            }
        }
        let mut top_ordered = Mesh::new();
        let mut bot_ordered = Mesh::new();
        for i in 0..panels.len() {
            let top_pts: Vec<Point> = panels[i].top_vertices.iter().map(|&lk| panels[i].mesh.vertex_point(lk).unwrap()).collect();
            let bot_pts: Vec<Point> = panels[i].bot_vertices.iter().map(|&lk| panels[i].mesh.vertex_point(lk).unwrap()).collect();
            let tvks: Vec<usize> = top_pts.into_iter().map(|pt| top_ordered.add_vertex(pt, None)).collect();
            let bvks: Vec<usize> = bot_pts.into_iter().map(|pt| bot_ordered.add_vertex(pt, None)).collect();
            top_ordered.add_face(tvks, Some(i));
            bot_ordered.add_face(bvks, Some(i));
        }
        (panels, adjacency, top_ordered, bot_ordered)
    }

    pub fn create_box(x: f64, y: f64, z: f64) -> Self {
        let (hx, hy, hz) = (x * 0.5, y * 0.5, z * 0.5);
        let vertices = vec![
            Point::new(-hx, -hy, -hz),
            Point::new( hx, -hy, -hz),
            Point::new( hx,  hy, -hz),
            Point::new(-hx,  hy, -hz),
            Point::new(-hx, -hy,  hz),
            Point::new( hx, -hy,  hz),
            Point::new( hx,  hy,  hz),
            Point::new(-hx,  hy,  hz),
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

    pub fn create_dodecahedron(edge: f64) -> Self {
        let phi = (1.0 + 5.0_f64.sqrt()) / 2.0;
        let ip = 1.0 / phi;
        let s = edge / (2.0 * ip);
        let verts = vec![
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
            [0, 8,10, 2,16], [0,16,17, 1,12], [0,12,14, 4, 8],
            [1,17, 3,11, 9], [1, 9, 5,14,12], [2,10, 6,15,13],
            [2,13, 3,17,16], [3,13,15, 7,11], [4,14, 5,19,18],
            [4,18, 6,10, 8], [5, 9,11, 7,19], [6,18,19, 7,15],
        ];
        let mut faces: Vec<Vec<Point>> = Vec::new();
        for f in &idx {
            faces.push(vec![verts[f[0]].clone(), verts[f[1]].clone(), verts[f[2]].clone(), verts[f[3]].clone(), verts[f[4]].clone()]);
        }
        Mesh::from_polylines(faces, None)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Boolean Queries
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn is_empty(&self) -> bool {
        self.vertex.is_empty()
    }

    pub fn is_valid(&self) -> bool {
        if self.vertex.is_empty() || self.face.is_empty() {
            return false;
        }
        for (_fkey, vkeys) in &self.face {
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

    pub fn is_closed(&self) -> bool {
        let mut hole_edges: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();
        for (_, rings) in &self.face_holes {
            for ring in rings {
                let n = ring.len();
                for i in 0..n {
                    let a = ring[i]; let b = ring[(i + 1) % n];
                    hole_edges.insert((a, b));
                    hole_edges.insert((b, a));
                }
            }
        }
        let dfe = self.directed_face_edges();
        for &(u, v) in &dfe {
            // a face edge whose reverse no face walks is a border - unless a declared hole
            // ring owns it (hole_edges holds both directions)
            if !dfe.contains(&(v, u)) && !hole_edges.contains(&(v, u)) { return false; }
        }
        !self.vertex.is_empty()
    }

    pub fn is_vertex_on_boundary(&self, vertex_key: usize) -> bool {
        let dfe = self.directed_face_edges();
        for &(u, v) in &dfe {
            if !dfe.contains(&(v, u)) && (u == vertex_key || v == vertex_key) {
                return true;
            }
        }
        false
    }

    pub fn is_edge_on_boundary(&self, u: usize, v: usize) -> bool {
        let dfe = self.directed_face_edges();
        !(dfe.contains(&(u, v)) && dfe.contains(&(v, u)))
    }

    pub fn is_face_on_boundary(&self, face_key: usize) -> bool {
        match self.face_edges(face_key) {
            Some(fe) => fe.into_iter().any(|(u, v)| self.is_edge_on_boundary(u, v)),
            None => false,
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Basic Queries
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn strip_render_data(&mut self) {
        self.halfedge.clear();
        self.pointcolors.clear();
        self.facecolors.clear();
        self.linecolors.clear();
        self.widths.clear();
    }

    pub fn number_of_vertices(&self) -> usize {
        self.vertex.len()
    }

    pub fn number_of_faces(&self) -> usize {
        self.face.len()
    }

    pub fn vertices(&self) -> Vec<usize> {
        let mut keys: Vec<usize> = self.vertex.keys().cloned().collect();
        keys.sort();
        keys
    }

    pub fn faces(&self) -> Vec<usize> {
        let mut keys: Vec<usize> = self.face.keys().cloned().collect();
        keys.sort();
        keys
    }

    pub fn number_of_edges(&self) -> usize {
        let dfe = self.directed_face_edges();
        dfe.iter().filter(|&&(u, v)| u < v || !dfe.contains(&(v, u))).count()
    }

    pub fn edges(&self) -> Vec<(usize, usize)> {
        let dfe = self.directed_face_edges();
        let mut result: Vec<(usize, usize)> = dfe.iter()
            .map(|&(u, v)| if u < v { (u, v) } else { (v, u) })
            .collect::<HashSet<_>>()
            .into_iter()
            .collect();
        result.sort();
        result
    }

    pub fn naked_edges(&self, boundary: bool) -> Vec<(usize, usize)> {
        // one directed set for the whole call - never one per edge (that walk is quadratic)
        let dfe = self.directed_face_edges();
        let mut result: Vec<(usize, usize)> = dfe.iter()
            .map(|&(u, v)| if u < v { (u, v) } else { (v, u) })
            .collect::<HashSet<_>>()
            .into_iter()
            .filter(|&(u, v)| (!(dfe.contains(&(u, v)) && dfe.contains(&(v, u)))) == boundary)
            .collect();
        result.sort();
        result
    }

    /// Edges paired with their stored line color, walked in the SAME order `add_face`
    /// seeded `linecolors` (first-discovery during face traversal) — so color N belongs
    /// to edge N. `edges()` sorts `(u < v)` and therefore does NOT align with `linecolors`;
    /// this walk does. Robust for meshes built face-by-face; note that `remove_face`
    /// truncates `linecolors` from the end, so a mesh edited by face removal can desync
    /// (rebuild the colors after structural edits).
    pub fn edges_with_colors(&self) -> Vec<(usize, usize, Color)> {
        let mut seen: HashSet<(usize, usize)> = HashSet::new();
        let mut out: Vec<(usize, usize, Color)> = Vec::new();
        let mut ci = 0usize;
        let mut fkeys: Vec<usize> = self.face.keys().copied().collect();
        fkeys.sort_unstable();
        for fk in fkeys {
            let vs = &self.face[&fk];
            for i in 0..vs.len() {
                let u = vs[i];
                let v = vs[(i + 1) % vs.len()];
                let e = if u < v { (u, v) } else { (v, u) };
                if seen.insert(e) {
                    let c = self.linecolors.get(ci).cloned().unwrap_or_else(Color::black);
                    out.push((e.0, e.1, c));
                    ci += 1;
                }
            }
        }
        out
    }

    pub fn naked_vertices(&self, boundary: bool) -> Vec<usize> {
        let mut keys: Vec<usize> = self.vertex.keys().cloned().collect();
        keys.sort();
        let mut result = Vec::new();
        for vk in keys {
            if self.is_vertex_on_boundary(vk) == boundary {
                result.push(vk);
            }
        }
        result
    }

    pub fn naked_faces(&self, boundary: bool) -> Vec<usize> {
        let mut keys: Vec<usize> = self.face.keys().cloned().collect();
        keys.sort();
        let mut result = Vec::new();
        for fk in keys {
            if self.is_face_on_boundary(fk) == boundary {
                result.push(fk);
            }
        }
        result
    }

    pub fn euler(&self) -> i32 {
        let v = self.number_of_vertices() as i32;
        let e = self.number_of_edges() as i32;
        let f = self.number_of_faces() as i32;
        v - e + f
    }

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
        self.objectcolor = Color::white();
        self.color_mode = ColorMode::OBJECTCOLOR;
        self.invalidate_triangle_bvh();
    }

    pub fn set_pointcolors(&mut self, v: Vec<Color>) { self.pointcolors = v; self.color_mode = ColorMode::POINTCOLORS; self.gpu_cache.0 = None; }
    pub fn set_facecolors(&mut self, v: Vec<Color>) { self.facecolors = v; self.color_mode = ColorMode::FACECOLORS; }
    pub fn set_linecolors(&mut self, v: Vec<Color>, w: Vec<f64>) { self.linecolors = v; if !w.is_empty() { self.widths = w; } }
    pub fn set_objectcolor(&mut self, c: Color) { self.objectcolor = c; self.gpu_cache.0 = None; }

    pub fn get_pointcolors(&self) -> &[Color]      { &self.pointcolors }
    pub fn get_facecolors(&self) -> &[Color]       { &self.facecolors }
    pub fn get_linecolors(&self) -> &[Color]       { &self.linecolors }
    pub fn widths(&self) -> &[f64]                 { &self.widths }
    pub fn objectcolor(&self) -> &Color            { &self.objectcolor }
    pub fn pointcolors_mut(&mut self) -> &mut [Color] { &mut self.pointcolors }
    pub fn facecolors_mut(&mut self) -> &mut [Color]  { &mut self.facecolors }
    pub fn linecolors_mut(&mut self) -> &mut [Color]  { &mut self.linecolors }
    pub fn widths_mut(&mut self) -> &mut [f64]        { &mut self.widths }

    pub fn clear_pointcolors(&mut self) { self.pointcolors.clear(); if self.color_mode == ColorMode::POINTCOLORS { self.color_mode = ColorMode::OBJECTCOLOR; } }
    pub fn clear_facecolors(&mut self) { self.facecolors.clear(); if self.color_mode == ColorMode::FACECOLORS { self.color_mode = ColorMode::OBJECTCOLOR; } }
    pub fn clear_linecolors(&mut self) { self.linecolors.clear(); self.widths.clear(); }

    pub fn unify_winding(&mut self) -> bool {
        if self.face.len() < 2 {
            return false;
        }

        let mut edge_faces: HashMap<(usize, usize), Vec<(usize, usize, usize)>> = HashMap::new();
        for (&fkey, verts) in &self.face {
            let n = verts.len();
            for i in 0..n {
                let u = verts[i];
                let v = verts[(i + 1) % n];
                let edge = if u < v { (u, v) } else { (v, u) };
                edge_faces.entry(edge).or_default().push((fkey, u, v));
            }
        }

        let mut visited: HashSet<usize> = HashSet::new();
        let mut flipped: HashSet<usize> = HashSet::new();
        let face_keys: Vec<usize> = self.face.keys().copied().collect();
        for seed in face_keys {
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
                    let (eff_u, eff_v) = if is_flipped { (v_orig, u_orig) } else { (u_orig, v_orig) };
                    let edge = if u_orig < v_orig { (u_orig, v_orig) } else { (v_orig, u_orig) };
                    if let Some(adj_list) = edge_faces.get(&edge) {
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
        }

        if flipped.is_empty() {
            return false;
        }

        for &fkey in &flipped {
            self.face.get_mut(&fkey).unwrap().reverse();
        }

        for neighbors in self.halfedge.values_mut() {
            neighbors.clear();
        }
        let face_items: Vec<(usize, Vec<usize>)> = self.face.iter().map(|(&k, v)| (k, v.clone())).collect();
        for (fkey, verts) in face_items {
            let n = verts.len();
            for i in 0..n {
                let u = verts[i];
                let v = verts[(i + 1) % n];
                self.halfedge.entry(u).or_default().insert(v, Some(fkey));
                self.halfedge.entry(v).or_default().entry(u).or_insert(None);
            }
        }

        self.orient_outward();
        true
    }

    pub fn orient_outward(&mut self) -> bool {
        self.ensure_halfedges();
        if self.face.is_empty() || !self.naked_edges(true).is_empty() {
            return false;
        }
        let face_items: Vec<(usize, Vec<usize>)> = self.face.iter().map(|(&k, v)| (k, v.clone())).collect();
        let mut vol = 0.0f64;
        for (_fk, verts) in &face_items {
            let n = verts.len();
            let p0 = self.vertex_point(verts[0]).unwrap();
            for i in 1..n - 1 {
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
        for neighbors in self.halfedge.values_mut() {
            neighbors.clear();
        }
        let face_items2: Vec<(usize, Vec<usize>)> = self.face.iter().map(|(&k, v)| (k, v.clone())).collect();
        for (fk, verts) in face_items2 {
            let n = verts.len();
            for i in 0..n {
                let u = verts[i];
                let v = verts[(i + 1) % n];
                self.halfedge.entry(u).or_default().insert(v, Some(fk));
                self.halfedge.entry(v).or_default().entry(u).or_insert(None);
            }
        }
        true
    }

    pub fn unweld(&self) -> Mesh {
        let mut m = Mesh::new();
        let mut fkeys: Vec<usize> = self.face.keys().copied().collect();
        fkeys.sort();
        for fkey in &fkeys {
            let vkeys = &self.face[fkey];
            let mut new_vkeys = Vec::new();
            for &vk in vkeys {
                let vd = &self.vertex[&vk];
                new_vkeys.push(m.add_vertex(Point::new(vd.x, vd.y, vd.z), None));
            }
            m.add_face(new_vkeys, None);
        }
        m
    }

    pub fn weld(&self, tolerance: f64) -> Mesh {
        if self.vertex.is_empty() { return Mesh::new(); }

        let mut vkeys: Vec<usize> = self.vertex.keys().copied().collect();
        vkeys.sort();
        let positions: Vec<Point> = vkeys.iter().map(|k| {
            let v = &self.vertex[k];
            Point::new(v.x, v.y, v.z)
        }).collect();
        let n = vkeys.len();

        let mut parent: Vec<usize> = (0..n).collect();
        fn find(parent: &mut Vec<usize>, mut x: usize) -> usize {
            while parent[x] != x { parent[x] = parent[parent[x]]; x = parent[x]; }
            x
        }

        if tolerance > 0.0 {
            let boxes: Vec<OBB> = positions.iter().map(|p| OBB::from_point(p.clone(), tolerance)).collect();
            let ws = SpatialBVH::compute_world_size(&boxes);
            let bvh = SpatialBVH::from_boxes(&boxes, ws);
            let (pairs, _, _) = bvh.check_all_collisions(&boxes);
            for (i, j) in pairs {
                if positions[i].distance(&positions[j], None) <= tolerance {
                    let ri = find(&mut parent, i);
                    let rj = find(&mut parent, j);
                    if ri != rj { parent[ri] = rj; }
                }
            }
        }

        let mut root_to_rep: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
        for i in 0..n {
            let root = find(&mut parent, i);
            let entry = root_to_rep.entry(root).or_insert(vkeys[i]);
            if vkeys[i] < *entry { *entry = vkeys[i]; }
        }
        let vkey_to_rep: std::collections::HashMap<usize, usize> = (0..n).map(|i| {
            let root = find(&mut parent, i);
            (vkeys[i], root_to_rep[&root])
        }).collect();

        let mut m = Mesh::new();
        let mut added: std::collections::HashSet<usize> = std::collections::HashSet::new();
        for i in 0..n {
            let rep = vkey_to_rep[&vkeys[i]];
            if added.insert(rep) {
                let v = &self.vertex[&rep];
                m.add_vertex(Point::new(v.x, v.y, v.z), Some(rep));
            }
        }
        for (fk, fvkeys) in &self.face {
            let new_vkeys: Vec<usize> = fvkeys.iter().map(|vk| vkey_to_rep[vk]).collect();
            m.add_face(new_vkeys, Some(*fk));
        }
        m
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Vertex and Face Operations
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_vertex(&mut self, position: Point, key: Option<usize>) -> usize {
        self.ensure_halfedges();
        let vertex_key = match key {
            Some(k) => {
                if k >= self.max_vertex {
                    self.max_vertex = k + 1;
                }
                k
            }
            None => {
                let k = self.max_vertex;
                self.max_vertex += 1;
                k
            }
        };

        let vertex_data = VertexData::new(position);
        self.vertex.insert(vertex_key, vertex_data);
        self.halfedge.entry(vertex_key).or_default();
        self.pointcolors.push(Color::white());
        self.invalidate_triangle_bvh();

        vertex_key
    }

    pub fn add_face(&mut self, vertices: Vec<usize>, fkey: Option<usize>) -> Option<usize> {
        self.ensure_halfedges();
        if vertices.len() < 3 {
            return None;
        }

        if !vertices.iter().all(|v| self.vertex.contains_key(v)) {
            return None;
        }

        let mut unique_vertices = HashSet::new();
        for vertex in &vertices {
            if !unique_vertices.insert(*vertex) {
                return None;
            }
        }

        let face_key = match fkey {
            Some(k) => {
                if k >= self.max_face {
                    self.max_face = k + 1;
                }
                k
            }
            None => {
                let k = self.max_face;
                self.max_face += 1;
                k
            }
        };

        self.face.insert(face_key, vertices.clone());
        self.triangulation.remove(&face_key);
        self.facecolors.push(Color::white());
        self.invalidate_triangle_bvh();

        for i in 0..vertices.len() {
            let u = vertices[i];
            let v = vertices[(i + 1) % vertices.len()];

            self.halfedge.entry(u).or_default();
            self.halfedge.entry(v).or_default();

            let is_new_edge = !self.halfedge.get(&v).unwrap().contains_key(&u);

            self.halfedge.get_mut(&u).unwrap().insert(v, Some(face_key));

            if is_new_edge {
                self.halfedge.get_mut(&v).unwrap().insert(u, None);
                self.linecolors.push(Color::black());
                self.widths.push(1.0);
            }
        }

        Some(face_key)
    }

    /// Recreate `halfedge` from `vertex` + `face` alone.
    ///
    /// Every write to the halfedge map lives inside a face-creation loop, so the map carries no
    /// information the faces do not already have - which is why `pb_dumps` stops serializing it and
    /// calls this on the way back in. The rules are `add_face`'s: the directed edge u->v belongs to
    /// its face, and the twin v->u is seeded to `None` unless a face already claimed it. Order is
    /// irrelevant - a shared edge ends up owned from both sides either way.
    ///
    /// Colors and widths are NOT touched here: `add_face` grows those vectors as it discovers
    /// edges, but on a load path they are restored from the wire.
    /// Every (u, v) some face ring walks, as a flat set. The halfedge-free backbone of the
    /// pure readers (is_closed, edges, boundaries): one transient allocation per call instead
    /// of a persistent nested-map structure per mesh.
    fn directed_face_edges(&self) -> std::collections::HashSet<(usize, usize)> {
        let mut s = std::collections::HashSet::with_capacity(self.face.len() * 4);
        for verts in self.face.values() {
            let n = verts.len();
            for i in 0..n {
                s.insert((verts[i], verts[(i + 1) % n]));
            }
        }
        s
    }

    /// Face-derived halfedge connectivity, computed WITHOUT mutating - `to_proto` borrows it
    /// when the lazy map was never built, so the wire format never changes.
    fn compute_halfedges(&self) -> HashMap<usize, HashMap<usize, Option<usize>>> {
        let mut he: HashMap<usize, HashMap<usize, Option<usize>>> =
            HashMap::with_capacity(self.vertex.len());
        for vkey in self.vertex.keys() {
            he.insert(*vkey, HashMap::new());
        }
        // SORTED face keys, and the FIRST face to walk a directed edge owns it - the same rule
        // as `edge_face_map`. Walking `self.face` in HashMap order let the LAST face win, so on a
        // mesh where two faces walk the same directed edge the halfedge map came out different
        // run to run, and `jsondump`/`file_json_dump` of that mesh was not reproducible.
        let mut fkeys: Vec<usize> = self.face.keys().copied().collect();
        fkeys.sort_unstable();
        for fkey in fkeys {
            let verts = &self.face[&fkey];
            for i in 0..verts.len() {
                let u = verts[i];
                let v = verts[(i + 1) % verts.len()];
                let slot = he.entry(u).or_default().entry(v).or_insert(None);
                if slot.is_none() { *slot = Some(fkey); }
                he.entry(v).or_default().entry(u).or_insert(None);
            }
        }
        he
    }

    pub fn rebuild_halfedges(&mut self) {
        self.halfedge = self.compute_halfedges();
    }

    /// Topology is LAZY: a freshly DECODED mesh carries no halfedge map - measured as the
    /// dominant decode-time and memory cost on dense scenes (a nested HashMap per vertex,
    /// per mesh). Every EDIT entry point calls this first; the pure readers are face-based
    /// and never build it. Constructed meshes (add_vertex/add_face from empty) maintain the
    /// map incrementally exactly as before, so this fires only on decoded-then-edited meshes.
    pub fn ensure_halfedges(&mut self) {
        if self.halfedge.is_empty() && !self.face.is_empty() {
            self.rebuild_halfedges();
        }
    }

    pub fn remove_face(&mut self, fkey: usize) {
        self.ensure_halfedges();
        let verts = match self.face.get(&fkey) {
            Some(v) => v.clone(),
            None => return,
        };
        let n = verts.len();
        for i in 0..n {
            let u = verts[i];
            let v = verts[(i + 1) % n];
            if let Some(nbrs) = self.halfedge.get_mut(&u) {
                if nbrs.contains_key(&v) {
                    nbrs.insert(v, None);
                }
            }
            let v_to_u_none = self.halfedge.get(&v).and_then(|m| m.get(&u)).map(|f| f.is_none()).unwrap_or(false);
            if v_to_u_none {
                if let Some(nbrs) = self.halfedge.get_mut(&u) { nbrs.remove(&v); }
                if let Some(nbrs) = self.halfedge.get_mut(&v) { nbrs.remove(&u); }
            }
        }
        self.face.remove(&fkey);
        self.triangulation.remove(&fkey);
        self.facedata.remove(&fkey);
        self.face_holes.remove(&fkey);
        let n_edges = self.number_of_edges();
        if self.linecolors.len() > n_edges { self.linecolors.truncate(n_edges); }
        if self.widths.len() > n_edges { self.widths.truncate(n_edges); }
        let n_faces = self.face.len();
        if self.facecolors.len() > n_faces { self.facecolors.truncate(n_faces); }
        self.invalidate_triangle_bvh();
    }

    pub fn remove_vertex(&mut self, vkey: usize) {
        self.ensure_halfedges();
        if !self.vertex.contains_key(&vkey) { return; }
        let faces_to_remove: Vec<usize> = self.face.iter()
            .filter(|(_, verts)| verts.contains(&vkey))
            .map(|(&fk, _)| fk)
            .collect();
        for fk in faces_to_remove {
            self.remove_face(fk);
        }
        if let Some(nbrs) = self.halfedge.remove(&vkey) {
            for (v, _) in nbrs {
                if let Some(m) = self.halfedge.get_mut(&v) { m.remove(&vkey); }
            }
        }
        self.edgedata.retain(|k, _| k.0 != vkey && k.1 != vkey);
        self.vertex.remove(&vkey);
        let n_vertices = self.vertex.len();
        if self.pointcolors.len() > n_vertices { self.pointcolors.truncate(n_vertices); }
        self.invalidate_triangle_bvh();
    }

    pub fn remove_edge(&mut self, u: usize, v: usize) {
        self.ensure_halfedges();
        let mut faces_to_remove = Vec::new();
        if let Some(f) = self.halfedge.get(&u).and_then(|m| m.get(&v)).and_then(|&f| f) {
            faces_to_remove.push(f);
        }
        if let Some(f) = self.halfedge.get(&v).and_then(|m| m.get(&u)).and_then(|&f| f) {
            if !faces_to_remove.contains(&f) { faces_to_remove.push(f); }
        }
        for fk in faces_to_remove {
            self.remove_face(fk);
        }
        if let Some(nbrs) = self.halfedge.get_mut(&u) { nbrs.remove(&v); }
        if let Some(nbrs) = self.halfedge.get_mut(&v) { nbrs.remove(&u); }
        self.edgedata.remove(&(u, v));
        self.edgedata.remove(&(v, u));
        let n_edges = self.number_of_edges();
        if self.linecolors.len() > n_edges { self.linecolors.truncate(n_edges); }
        if self.widths.len() > n_edges { self.widths.truncate(n_edges); }
        self.invalidate_triangle_bvh();
    }

    pub fn flip_face(&mut self, fkey: usize) {
        self.ensure_halfedges();
        let fv = match self.face.get(&fkey) {
            Some(v) => v.clone(),
            None => return,
        };
        self.remove_face(fkey);
        let mut rev = fv;
        rev.reverse();
        self.add_face(rev, Some(fkey));
    }

    pub fn flip(&mut self) {
        for verts in self.face.values_mut() {
            verts.reverse();
        }
        for neighbors in self.halfedge.values_mut() {
            neighbors.clear();
        }
        let face_items: Vec<(usize, Vec<usize>)> = self.face.iter().map(|(&k, v)| (k, v.clone())).collect();
        for (fkey, verts) in face_items {
            let n = verts.len();
            for i in 0..n {
                let u = verts[i];
                let v = verts[(i + 1) % n];
                self.halfedge.entry(u).or_default().insert(v, Some(fkey));
                self.halfedge.entry(v).or_default().entry(u).or_insert(None);
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Connectivity Queries
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn edge_edges(&self, u: usize, v: usize) -> Option<Vec<(usize, usize)>> {
        let dfe = self.directed_face_edges();
        if !dfe.contains(&(u, v)) && !dfe.contains(&(v, u)) { return None; }
        let ends = |x: usize| -> Vec<usize> {
            let mut keys: Vec<usize> = dfe.iter()
                .filter_map(|&(a, b)| if a == x { Some(b) } else if b == x { Some(a) } else { None })
                .collect::<std::collections::HashSet<_>>()
                .into_iter()
                .collect();
            keys.sort();
            keys
        };
        let mut edges = Vec::new();
        for w in ends(u) { if w != v { edges.push((u, w)); } }
        for w in ends(v) { if w != u { edges.push((v, w)); } }
        Some(edges)
    }

    /// Every directed face edge -> its face key, in ONE face walk. The bulk form of
    /// `edge_faces` for per-edge loops (a per-call `edge_faces` is O(E) face-based, so a
    /// loop over all edges would be quadratic - build this once instead).
    pub fn edge_face_map(&self) -> HashMap<(usize, usize), usize> {
        let mut m: HashMap<(usize, usize), usize> = HashMap::with_capacity(self.face.len() * 4);
        // SORTED keys, and the FIRST face to walk a directed edge keeps it. Iterating `self.face`
        // directly walked the faces in HashMap order and `insert` let the LAST one win, so on a
        // mesh where two faces walk the same directed edge - an inconsistently wound or
        // non-manifold patch - the winner changed run to run. Rust seeds every HashMap
        // differently, so two loads of the SAME file in one process disagreed: floor_model.pb
        // came back with different packed `facing` words on 2 of its 15,095 wireframe edges, and
        // a golden-image test on such a mesh flakes. Same walk order as `edges_with_colors`.
        let mut fkeys: Vec<usize> = self.face.keys().copied().collect();
        fkeys.sort_unstable();
        for fkey in fkeys {
            let verts = &self.face[&fkey];
            let n = verts.len();
            for i in 0..n {
                m.entry((verts[i], verts[(i + 1) % n])).or_insert(fkey);
            }
        }
        m
    }

    pub fn edge_faces(&self, u: usize, v: usize) -> Option<Vec<usize>> {
        // ORDER IS THE CONTRACT: the face walking (u, v) first, then the one walking (v, u) -
        // the two sides of the directed edge, exactly what the halfedge map used to answer.
        // Collecting by iterating `self.face` instead would hand back HashMap order, i.e. a
        // different answer run to run.
        let efm = self.edge_face_map();
        let out: Vec<usize> = [efm.get(&(u, v)), efm.get(&(v, u))]
            .into_iter().flatten().copied().collect();
        if out.is_empty() { None } else { Some(out) }
    }

    pub fn edge_line(&self, u: usize, v: usize) -> Option<Line> {
        let dfe = self.directed_face_edges();
        if !dfe.contains(&(u, v)) && !dfe.contains(&(v, u)) { return None; }
        Some(Line::from_points(&self.vertex_point(u)?, &self.vertex_point(v)?))
    }

    pub fn face_edges(&self, face_key: usize) -> Option<Vec<(usize, usize)>> {
        let verts = self.face.get(&face_key)?;
        let n = verts.len();
        Some((0..n).map(|i| (verts[i], verts[(i + 1) % n])).collect())
    }

    pub fn face_faces(&self, face_key: usize) -> Option<Vec<usize>> {
        let fe = self.face_edges(face_key)?;
        let efm = self.edge_face_map();
        Some(fe.into_iter()
            .filter_map(|(u, v)| efm.get(&(v, u)).copied())
            .collect())
    }

    pub fn face_points(&self, face_key: usize) -> Option<Vec<Point>> {
        let fv = self.face_vertices(face_key)?;
        fv.iter().map(|&vk| self.vertex_point(vk)).collect()
    }

    pub fn face_polyline(&self, face_key: usize) -> Option<Polyline> {
        Some(Polyline::new(self.face_points(face_key)?))
    }

    pub fn face_vertices(&self, face_key: usize) -> Option<&Vec<usize>> {
        self.face.get(&face_key)
    }

    pub fn vertex_edges(&self, vertex_key: usize) -> Option<Vec<(usize, usize)>> {
        if !self.vertex.contains_key(&vertex_key) { return None; }
        let mut keys = self.vertex_vertices(vertex_key)?;
        keys.sort();
        Some(keys.into_iter().map(|u| (vertex_key, u)).collect())
    }

    pub fn vertex_faces(&self, vertex_key: usize) -> Option<Vec<usize>> {
        if !self.vertex.contains_key(&vertex_key) { return None; }
        // same order as the halfedge version: neighbors sorted, each mapped to the face of
        // the DIRECTED edge (v, u) - one entry per incident face, no duplicates
        let efm = self.edge_face_map();
        let mut keys = self.vertex_vertices(vertex_key)?;
        keys.sort();
        Some(keys.into_iter().filter_map(|u| efm.get(&(vertex_key, u)).copied()).collect())
    }

    pub fn vertex_point(&self, vertex_key: usize) -> Option<Point> {
        self.vertex.get(&vertex_key).map(|v| v.position())
    }

    pub fn vertex_vertices(&self, vertex_key: usize) -> Option<Vec<usize>> {
        if !self.vertex.contains_key(&vertex_key) { return None; }
        let dfe = self.directed_face_edges();
        let mut keys: Vec<usize> = dfe.iter()
            .filter_map(|&(u, v)| if u == vertex_key { Some(v) } else if v == vertex_key { Some(u) } else { None })
            .collect::<std::collections::HashSet<_>>()
            .into_iter()
            .collect();
        keys.sort();
        Some(keys)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Geometric Properties
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn area(&self) -> f64 {
        let mut total = 0.0;
        // SORTED, because floating-point addition is not associative: summing in HashMap order
        // gave a result that differed in the last bits between two loads of the same file, and a
        // test comparing areas exactly then flakes.
        let mut fkeys: Vec<usize> = self.face.keys().copied().collect();
        fkeys.sort_unstable();
        for fk in fkeys {
            if let Some(a) = self.face_area(fk) {
                total += a;
            }
        }
        total
    }

    pub fn centroid(&self) -> Point {
        let mut x = 0.0_f64; let mut y = 0.0_f64; let mut z = 0.0_f64;
        // Sorted for the same reason as `area`: a reproducible summation order.
        let mut vkeys: Vec<usize> = self.vertex.keys().copied().collect();
        vkeys.sort_unstable();
        for vk in vkeys {
            let p = self.vertex_point(vk).unwrap();
            x += p[0]; y += p[1]; z += p[2];
        }
        let n = if self.vertex.is_empty() { 1.0 } else { self.vertex.len() as f64 };
        Point::new(x / n, y / n, z / n)
    }

    pub fn dihedral_angle(&self, u: usize, v: usize) -> Option<f64> {
        let ef = self.edge_faces(u, v)?;
        if ef.len() < 2 { return None; }
        let n0 = self.face_normal(ef[0])?;
        let n1 = self.face_normal(ef[1])?;
        let dot = n0.dot(&n1).clamp(-1.0, 1.0);
        Some((PI - dot.acos()) * 180.0 / PI)
    }

    pub fn dihedral_angles(&self, scale: f64)
        -> (std::collections::BTreeMap<(usize, usize), f64>, Vec<Polyline>, Vec<Point>)
    {
        let mut angles: std::collections::BTreeMap<(usize, usize), f64> = std::collections::BTreeMap::new();
        let mut arcs: Vec<Polyline> = Vec::new();
        let mut points: Vec<Point> = Vec::new();
        let arc_n: usize = 12;
        for (u, v) in self.edges() {
            let da = match self.dihedral_angle(u, v) { Some(a) => a, None => continue };
            angles.insert((u, v), da);
            let deg = da;
            let ep0 = match self.vertex_point(u) { Some(p) => p, None => continue };
            let ep1 = match self.vertex_point(v) { Some(p) => p, None => continue };
            let mx = (ep0[0]+ep1[0])*0.5;
            let my = (ep0[1]+ep1[1])*0.5;
            let mz = (ep0[2]+ep1[2])*0.5;
            if scale == 0.0 {
                let mut pt = Point::new(mx, my, mz);
                pt.name = deg.to_string();
                pt.pointcolor = Color::new(240.0/255.0, 220.0/255.0, 0.0, 1.0);
                points.push(pt);
                continue;
            }
            let ef = match self.edge_faces(u, v) { Some(f) => f, None => continue };
            if ef.len() < 2 { continue; }
            let ex = ep1[0]-ep0[0]; let ey = ep1[1]-ep0[1]; let ez = ep1[2]-ep0[2];
            let elen = (ex*ex+ey*ey+ez*ez).sqrt();
            if elen < 1e-10 { continue; }
            let ex = ex/elen; let ey = ey/elen; let ez = ez/elen;
            let fc0 = match self.face_centroid(ef[0]) { Some(p) => p, None => continue };
            let fc1 = match self.face_centroid(ef[1]) { Some(p) => p, None => continue };
            let mut d0x = fc0[0]-mx; let mut d0y = fc0[1]-my; let mut d0z = fc0[2]-mz;
            let dot0 = d0x*ex+d0y*ey+d0z*ez;
            d0x -= dot0*ex; d0y -= dot0*ey; d0z -= dot0*ez;
            let d0len = (d0x*d0x+d0y*d0y+d0z*d0z).sqrt();
            if d0len < 1e-10 { continue; }
            let d0x = d0x/d0len; let d0y = d0y/d0len; let d0z = d0z/d0len;
            let mut d1x = fc1[0]-mx; let mut d1y = fc1[1]-my; let mut d1z = fc1[2]-mz;
            let dot1 = d1x*ex+d1y*ey+d1z*ez;
            d1x -= dot1*ex; d1y -= dot1*ey; d1z -= dot1*ez;
            let d1len = (d1x*d1x+d1y*d1y+d1z*d1z).sqrt();
            if d1len < 1e-10 { continue; }
            let d1x = d1x/d1len; let d1y = d1y/d1len; let d1z = d1z/d1len;
            let theta = (d0x*d1x+d0y*d1y+d0z*d1z).clamp(-1.0, 1.0).acos();
            if theta.sin().abs() < 1e-10 { continue; }
            let mut arc_pts: Vec<Point> = Vec::new();
            for j in 0..=arc_n {
                let t = j as f64 / arc_n as f64;
                let w1 = ((1.0-t)*theta).sin() / theta.sin();
                let w2 = (t*theta).sin() / theta.sin();
                arc_pts.push(Point::new(
                    mx+(w1*d0x+w2*d1x)*scale,
                    my+(w1*d0y+w2*d1y)*scale,
                    mz+(w1*d0z+w2*d1z)*scale));
            }
            let mut arc = Polyline::new(arc_pts.clone());
            arc.name = format!("dihedral_e{}_{}={}", u, v, deg);
            arc.linecolor = Color::new(240.0/255.0, 220.0/255.0, 0.0, 1.0);
            arcs.push(arc);
            let mid = &arc_pts[arc_n/2];
            let mut pt = Point::new(mid[0], mid[1], mid[2]);
            pt.name = deg.to_string();
            pt.pointcolor = Color::new(240.0/255.0, 220.0/255.0, 0.0, 1.0);
            points.push(pt);
        }
        (angles, arcs, points)
    }

    pub fn face_area(&self, face_key: usize) -> Option<f64> {
        let vertices = self.face.get(&face_key)?;
        if vertices.len() < 3 {
            return Some(0.0);
        }

        let mut area = 0.0;
        let p0 = self.vertex_point(vertices[0])?;

        for i in 1..(vertices.len() - 1) {
            let p1 = self.vertex_point(vertices[i])?;
            let p2 = self.vertex_point(vertices[i + 1])?;

            let u = Vector::new(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
            let v = Vector::new(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);

            area += u.cross(&v).magnitude() * 0.5;
        }

        Some(area)
    }

    pub fn face_centroid(&self, face_key: usize) -> Option<Point> {
        let verts = self.face.get(&face_key)?;
        if verts.is_empty() { return None; }
        let mut x = 0.0_f64; let mut y = 0.0_f64; let mut z = 0.0_f64;
        for vk in verts {
            let p = self.vertex_point(*vk)?;
            x += p[0]; y += p[1]; z += p[2];
        }
        let n = verts.len() as f64;
        Some(Point::new(x / n, y / n, z / n))
    }

    pub fn face_normal(&self, face_key: usize) -> Option<Vector> {
        let vertices = self.face.get(&face_key)?;
        if vertices.len() < 3 {
            return None;
        }

        let p0 = self.vertex_point(vertices[0])?;
        let p1 = self.vertex_point(vertices[1])?;
        let p2 = self.vertex_point(vertices[2])?;

        let u = Vector::new(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
        let v = Vector::new(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);

        let normal = u.cross(&v);
        let len = normal.magnitude();
        if len > Tolerance::ZERO_TOLERANCE {
            Some(Vector::new(
                normal[0] / len,
                normal[1] / len,
                normal[2] / len,
            ))
        } else {
            None
        }
    }

    pub fn face_normals(&self) -> HashMap<usize, Vector> {
        let mut normals = HashMap::new();
        for face_key in self.face.keys() {
            if let Some(normal) = self.face_normal(*face_key) {
                normals.insert(*face_key, normal);
            }
        }
        normals
    }

    pub fn vertex_angle_in_face(&self, vertex_key: usize, face_key: usize) -> Option<f64> {
        let vertices = self.face.get(&face_key)?;
        let vertex_index = vertices.iter().position(|&v| v == vertex_key)?;

        let n = vertices.len();
        let prev_vertex = vertices[(vertex_index + n - 1) % n];
        let next_vertex = vertices[(vertex_index + 1) % n];

        let center = self.vertex_point(vertex_key)?;
        let prev_pos = self.vertex_point(prev_vertex)?;
        let next_pos = self.vertex_point(next_vertex)?;

        let u = Vector::new(
            prev_pos[0] - center[0],
            prev_pos[1] - center[1],
            prev_pos[2] - center[2],
        );
        let v = Vector::new(
            next_pos[0] - center[0],
            next_pos[1] - center[1],
            next_pos[2] - center[2],
        );

        let u_len = u.magnitude();
        let v_len = v.magnitude();

        if u_len < Tolerance::ZERO_TOLERANCE || v_len < Tolerance::ZERO_TOLERANCE {
            return Some(0.0);
        }

        let cos_angle = u.dot(&v) / (u_len * v_len);
        let cos_angle = cos_angle.clamp(-1.0, 1.0);
        Some(cos_angle.acos())
    }

    pub fn vertex_normal(&self, vertex_key: usize) -> Option<Vector> {
        self.vertex_normal_weighted(vertex_key, NormalWeighting::Area)
    }

    pub fn vertex_normal_weighted(
        &self,
        vertex_key: usize,
        weighting: NormalWeighting,
    ) -> Option<Vector> {
        let faces = match self.vertex_faces(vertex_key) {
            Some(f) if !f.is_empty() => f,
            _ => return None,
        };

        let mut normal_acc = Vector::new(0.0, 0.0, 0.0);

        for face_key in faces {
            if let Some(face_normal) = self.face_normal(face_key) {
                let weight = match weighting {
                    NormalWeighting::Area => self.face_area(face_key).unwrap_or(1.0),
                    NormalWeighting::Angle => self
                        .vertex_angle_in_face(vertex_key, face_key)
                        .unwrap_or(1.0),
                    NormalWeighting::Uniform => 1.0,
                };

                normal_acc[0] = normal_acc[0] + face_normal[0] * weight;
                normal_acc[1] = normal_acc[1] + face_normal[1] * weight;
                normal_acc[2] = normal_acc[2] + face_normal[2] * weight;
            }
        }

        let len = normal_acc.magnitude();
        if len > Tolerance::ZERO_TOLERANCE {
            Some(Vector::new(
                normal_acc[0] / len,
                normal_acc[1] / len,
                normal_acc[2] / len,
            ))
        } else {
            None
        }
    }

    pub fn vertex_normals(&self) -> HashMap<usize, Vector> {
        self.vertex_normals_weighted(NormalWeighting::Area)
    }

    pub fn vertex_normals_weighted(&self, weighting: NormalWeighting) -> HashMap<usize, Vector> {
        let mut acc: HashMap<usize, [f64; 3]> = HashMap::new();
        for (_, vkeys) in &self.face {
            let n = vkeys.len();
            if n < 3 { continue; }
            let mut pts: Vec<[f64; 3]> = Vec::with_capacity(n);
            let mut ok = true;
            for &vk in vkeys {
                match self.vertex.get(&vk) {
                    Some(vd) => pts.push([vd.x, vd.y, vd.z]),
                    None => { ok = false; break; }
                }
            }
            if !ok { continue; }
            let ex = pts[1][0]-pts[0][0]; let ey = pts[1][1]-pts[0][1]; let ez = pts[1][2]-pts[0][2];
            let fx = pts[2][0]-pts[0][0]; let fy = pts[2][1]-pts[0][1]; let fz = pts[2][2]-pts[0][2];
            let cnx = ey*fz-ez*fy; let cny = ez*fx-ex*fz; let cnz = ex*fy-ey*fx;
            let len = (cnx*cnx + cny*cny + cnz*cnz).sqrt();
            if len < Tolerance::ZERO_TOLERANCE { continue; }
            let ux = cnx/len; let uy = cny/len; let uz = cnz/len;
            let area = match weighting {
                NormalWeighting::Area => {
                    let mut a = 0.0_f64;
                    for i in 1..(n-1) {
                        let ax = pts[i][0]-pts[0][0]; let ay = pts[i][1]-pts[0][1]; let az = pts[i][2]-pts[0][2];
                        let bx = pts[i+1][0]-pts[0][0]; let by = pts[i+1][1]-pts[0][1]; let bz = pts[i+1][2]-pts[0][2];
                        let cx = ay*bz-az*by; let cy = az*bx-ax*bz; let cz = ax*by-ay*bx;
                        a += (cx*cx + cy*cy + cz*cz).sqrt() * 0.5;
                    }
                    a
                }
                _ => 0.0,
            };
            for i in 0..n {
                let weight = match weighting {
                    NormalWeighting::Uniform => 1.0,
                    NormalWeighting::Area => area,
                    NormalWeighting::Angle => {
                        let prev = (i + n - 1) % n; let next = (i + 1) % n;
                        let ax = pts[prev][0]-pts[i][0]; let ay = pts[prev][1]-pts[i][1]; let az = pts[prev][2]-pts[i][2];
                        let bx = pts[next][0]-pts[i][0]; let by = pts[next][1]-pts[i][1]; let bz = pts[next][2]-pts[i][2];
                        let a_len = (ax*ax + ay*ay + az*az).sqrt();
                        let b_len = (bx*bx + by*by + bz*bz).sqrt();
                        if a_len < Tolerance::ZERO_TOLERANCE || b_len < Tolerance::ZERO_TOLERANCE { 0.0 }
                        else { ((ax*bx + ay*by + az*bz) / (a_len * b_len)).clamp(-1.0, 1.0).acos() }
                    }
                };
                let v = acc.entry(vkeys[i]).or_insert([0.0, 0.0, 0.0]);
                v[0] += ux * weight;
                v[1] += uy * weight;
                v[2] += uz * weight;
            }
        }
        let mut normals = HashMap::new();
        for (&vk, v) in &acc {
            let len = (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt();
            if len > Tolerance::ZERO_TOLERANCE {
                normals.insert(vk, Vector::new(v[0]/len, v[1]/len, v[2]/len));
            }
        }
        normals
    }

    pub fn compute_vertex_normals(&mut self) {
        let normals = self.vertex_normals();
        for (key, n) in &normals {
            if let Some(v) = self.vertex.get_mut(key) {
                v.set_normal(n[0], n[1], n[2]);
            }
        }
        self.invalidate_gpu();
    }

    pub fn volume(&self) -> f64 {
        let mut total = 0.0;
        // Sorted for the same reason as `area`: a reproducible summation order.
        let mut fkeys: Vec<usize> = self.face.keys().copied().collect();
        fkeys.sort_unstable();
        for fk in fkeys {
            let vkeys = &self.face[&fk];
            if vkeys.len() < 3 {
                continue;
            }
            let p0 = match self.vertex_point(vkeys[0]) { Some(p) => p, None => continue };
            for i in 1..(vkeys.len() - 1) {
                let p1 = match self.vertex_point(vkeys[i]) { Some(p) => p, None => continue };
                let p2 = match self.vertex_point(vkeys[i + 1]) { Some(p) => p, None => continue };
                total += p0[0] * (p1[1] * p2[2] - p1[2] * p2[1])
                       + p0[1] * (p1[2] * p2[0] - p1[0] * p2[2])
                       + p0[2] * (p1[0] * p2[1] - p1[1] * p2[0]);
            }
        }
        total.abs() / 6.0
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Basic Queries
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn vertex_index(&self) -> HashMap<usize, usize> {
        let mut keys: Vec<usize> = self.vertex.keys().copied().collect();
        keys.sort();
        keys.iter()
            .enumerate()
            .map(|(index, &key)| (key, index))
            .collect()
    }

    pub fn to_vertices_and_faces(&self) -> (Vec<Point>, Vec<Vec<usize>>) {
        let vertex_index = self.vertex_index();
        let mut vertices: Vec<Point> = vec![Point::default(); self.vertex.len()];

        for (&key, data) in &self.vertex {
            let idx = vertex_index[&key];
            vertices[idx] = data.position();
        }

        // Sort face keys to ensure consistent ordering
        let mut face_keys: Vec<usize> = self.face.keys().copied().collect();
        face_keys.sort();

        let mut faces = Vec::new();
        for face_key in face_keys {
            let face_vertices = &self.face[&face_key];
            let remapped: Vec<usize> = face_vertices.iter().map(|v| vertex_index[v]).collect();
            faces.push(remapped);
        }

        (vertices, faces)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn transform(&mut self, xform: &Xform) {
        for v in self.vertex.values_mut() {
            let mut pt = Point::new(v.x, v.y, v.z);
            pt.transform(xform);
            v.x = pt[0];
            v.y = pt[1];
            v.z = pt[2];
        }
        self.invalidate_triangle_bvh();
    }

    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut result = self.clone();
        result.transform(xform);
        result
    }

    pub fn duplicate(&self) -> Self {
        self.clone_with_new_guid()
    }

    pub fn clone_with_new_guid(&self) -> Self {
        let mut m = self.clone();
        m.guid = std::sync::OnceLock::new();
        m
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Clear the guid so a FRESH one mints lazily on next read — the duplicate/copy enabler.
    pub fn refresh_guid(&mut self) {
        self.guid = std::sync::OnceLock::new();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serializes the Mesh to JSON data
    pub fn jsondump(&self) -> serde_json::Value {
        let pointcolors_flat: Vec<u8> = self
            .pointcolors
            .iter()
            .flat_map(|c| vec![(c.r * 255.0) as u8, (c.g * 255.0) as u8, (c.b * 255.0) as u8, (c.a * 255.0) as u8])
            .collect();

        let facecolors_flat: Vec<u8> = self
            .facecolors
            .iter()
            .flat_map(|c| vec![(c.r * 255.0) as u8, (c.g * 255.0) as u8, (c.b * 255.0) as u8, (c.a * 255.0) as u8])
            .collect();

        let linecolors_flat: Vec<u8> = self
            .linecolors
            .iter()
            .flat_map(|c| vec![(c.r * 255.0) as u8, (c.g * 255.0) as u8, (c.b * 255.0) as u8, (c.a * 255.0) as u8])
            .collect();

        let mut tri_json = serde_json::Map::new();
        for (&fk, tris) in &self.triangulation {
            let tri_arr: Vec<serde_json::Value> = tris.iter()
                .map(|t| serde_json::json!([t[0], t[1], t[2]]))
                .collect();
            tri_json.insert(fk.to_string(), serde_json::Value::Array(tri_arr));
        }

        let mut face_holes_json = serde_json::Map::new();
        for (&fk, rings) in &self.face_holes {
            let rings_arr: Vec<serde_json::Value> = rings.iter()
                .map(|ring| serde_json::json!(ring))
                .collect();
            face_holes_json.insert(fk.to_string(), serde_json::Value::Array(rings_arr));
        }

        serde_json::json!({
            "type": "Mesh",
            "guid": self.guid(),
            "name": self.name,
            "vertex": self.vertex,
            "face": self.face,
            "face_holes": serde_json::Value::Object(face_holes_json),
            "halfedge": if self.halfedge.is_empty() && !self.face.is_empty() {
                self.compute_halfedges()
            } else {
                self.halfedge.clone()
            },
            "facedata": self.facedata,
            "edgedata": self.edgedata,
            "default_vertex_attributes": self.default_vertex_attributes,
            "default_face_attributes": self.default_face_attributes,
            "default_edge_attributes": self.default_edge_attributes,
            "max_vertex": self.max_vertex,
            "max_face": self.max_face,
            "pointcolors": pointcolors_flat,
            "triangulation": serde_json::Value::Object(tri_json),
            "facecolors": facecolors_flat,
            "linecolors": linecolors_flat,
            "objectcolor": serde_json::to_value(&self.objectcolor).unwrap_or(serde_json::Value::Null),
            "color_mode": self.color_mode.to_str(),
            "widths": self.widths
        })
    }

    pub fn jsonload(data: &serde_json::Value) -> Option<Self> {
        let mut mesh = Mesh::new();

        if let Some(guid) = data.get("guid").and_then(|v| v.as_str()) {
            mesh.set_guid(guid.to_string());
        }
        if let Some(name) = data.get("name").and_then(|v| v.as_str()) {
            mesh.name = name.to_string();
        }

        if let Some(vertex_data) = data.get("vertex") {
            mesh.vertex = serde_json::from_value(vertex_data.clone()).ok()?;
        }
        if let Some(face_data) = data.get("face") {
            mesh.face = serde_json::from_value(face_data.clone()).ok()?;
        }
        if let Some(fh_obj) = data.get("face_holes").and_then(|v| v.as_object()) {
            for (fk_str, rings_val) in fh_obj {
                if let Ok(fk) = fk_str.parse::<usize>() {
                    if let Some(rings_arr) = rings_val.as_array() {
                        let rings: Vec<Vec<usize>> = rings_arr.iter()
                            .filter_map(|r| r.as_array().map(|a| a.iter()
                                .filter_map(|v| v.as_u64().map(|n| n as usize)).collect()))
                            .collect();
                        mesh.face_holes.insert(fk, rings);
                    }
                }
            }
        }
        if let Some(halfedge_data) = data.get("halfedge") {
            mesh.halfedge = serde_json::from_value(halfedge_data.clone()).ok()?;
        }
        if let Some(facedata) = data.get("facedata") {
            mesh.facedata = serde_json::from_value(facedata.clone()).ok()?;
        }
        if let Some(edgedata) = data.get("edgedata") {
            mesh.edgedata = serde_json::from_value(edgedata.clone()).ok()?;
        }
        if let Some(max_vertex) = data.get("max_vertex").and_then(|v| v.as_u64()) {
            mesh.max_vertex = max_vertex as usize;
        }
        if let Some(max_face) = data.get("max_face").and_then(|v| v.as_u64()) {
            mesh.max_face = max_face as usize;
        }

        // Deserialize flat color arrays
        if let Some(pointcolors_flat) = data.get("pointcolors").and_then(|v| v.as_array()) {
            let rgba_values: Vec<u8> = pointcolors_flat
                .iter()
                .filter_map(|v| v.as_u64().map(|n| n as u8))
                .collect();
            mesh.pointcolors = rgba_values
                .chunks(4)
                .filter(|c| c.len() == 4)
                .map(|c| Color::new(c[0] as f32 / 255.0, c[1] as f32 / 255.0, c[2] as f32 / 255.0, c[3] as f32 / 255.0))
                .collect();
        }

        if let Some(facecolors_flat) = data.get("facecolors").and_then(|v| v.as_array()) {
            let rgba_values: Vec<u8> = facecolors_flat
                .iter()
                .filter_map(|v| v.as_u64().map(|n| n as u8))
                .collect();
            mesh.facecolors = rgba_values
                .chunks(4)
                .filter(|c| c.len() == 4)
                .map(|c| Color::new(c[0] as f32 / 255.0, c[1] as f32 / 255.0, c[2] as f32 / 255.0, c[3] as f32 / 255.0))
                .collect();
        }

        if let Some(linecolors_flat) = data.get("linecolors").and_then(|v| v.as_array()) {
            let rgba_values: Vec<u8> = linecolors_flat
                .iter()
                .filter_map(|v| v.as_u64().map(|n| n as u8))
                .collect();
            mesh.linecolors = rgba_values
                .chunks(4)
                .filter(|c| c.len() == 4)
                .map(|c| Color::new(c[0] as f32 / 255.0, c[1] as f32 / 255.0, c[2] as f32 / 255.0, c[3] as f32 / 255.0))
                .collect();
        }

        if let Some(widths) = data.get("widths").and_then(|v| v.as_array()) {
            mesh.widths = widths.iter().filter_map(|v| v.as_f64().map(|x| x as f64)).collect();
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
                if let Ok(fk) = fk_str.parse::<usize>() {
                    if let Some(tris_arr) = tris_val.as_array() {
                        let tris: Vec<[usize; 3]> = tris_arr.iter()
                            .filter_map(|t| {
                                let a = t.as_array()?;
                                if a.len() >= 3 {
                                    Some([a[0].as_u64()? as usize, a[1].as_u64()? as usize, a[2].as_u64()? as usize])
                                } else {
                                    None
                                }
                            })
                            .collect();
                        mesh.triangulation.insert(fk, tris);
                    }
                }
            }
        }

        Some(mesh)
    }

    pub fn file_json_dumps(&self) -> String {
        let sorted = crate::file_encoders::sort_json_keys(self.jsondump());
        serde_json::to_string_pretty(&sorted).unwrap_or_default()
    }

    pub fn file_json_loads(json_string: &str) -> Self {
        let data: serde_json::Value = serde_json::from_str(json_string).unwrap_or_default();
        Self::jsonload(&data).unwrap_or_else(|| Self::new())
    }

    pub fn file_json_dump(&self, filename: &str) -> std::io::Result<()> {
        let sorted = crate::file_encoders::sort_json_keys(self.jsondump());
        std::fs::write(filename, serde_json::to_string_pretty(&sorted)?)
    }

    pub fn file_json_load(filename: &str) -> std::io::Result<Self> {
        let content = std::fs::read_to_string(filename)?;
        let data: serde_json::Value = serde_json::from_str(&content)?;
        Self::jsonload(&data).ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::InvalidData, "Invalid mesh data")
        })
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        self.to_proto().encode_to_vec()
    }

    /// The proto struct itself — pb_dumps encodes it; Session embeds it directly.
    pub fn to_proto(&self) -> crate::proto::Mesh {
        use std::collections::HashMap;

        let mut vertices: HashMap<u64, crate::proto::VertexData> = HashMap::new();
        for (&vkey, vdata) in &self.vertex {
            let mut attrs: std::collections::BTreeMap<String, f64> = std::collections::BTreeMap::new();
            for (k, v) in &vdata.attributes {
                attrs.insert(k.clone(), *v as f64);
            }
            vertices.insert(vkey as u64, crate::proto::VertexData {
                x: vdata.x as f64,
                y: vdata.y as f64,
                z: vdata.z as f64,
                attributes: attrs,
            });
        }

        let mut faces: HashMap<u64, crate::proto::FaceData> = HashMap::new();
        for (&fkey, fverts) in &self.face {
            let mut attrs: std::collections::BTreeMap<String, f64> = std::collections::BTreeMap::new();
            if let Some(fdata) = self.facedata.get(&fkey) {
                for (k, v) in fdata {
                    attrs.insert(k.clone(), *v as f64);
                }
            }
            let holes: Vec<crate::proto::HoleRing> = self.face_holes.get(&fkey)
                .map(|rings| rings.iter().map(|ring| crate::proto::HoleRing {
                    vertices: ring.iter().map(|&v| v as u64).collect(),
                }).collect())
                .unwrap_or_default();
            faces.insert(fkey as u64, crate::proto::FaceData {
                vertices: fverts.iter().map(|&v| v as u64).collect(),
                attributes: attrs,
                holes,
            });
        }

        let mut edge_data_vec: Vec<crate::proto::EdgeData> = Vec::new();
        // SORTED: `edge_data` is a repeated field, so its order IS the bytes. Walking the
        // `edgedata` HashMap put the entries in a different order on every run.
        let mut ekeys: Vec<(usize, usize)> = self.edgedata.keys().copied().collect();
        ekeys.sort_unstable();
        for ek in ekeys {
            let (v1, v2) = (&ek.0, &ek.1);
            let attrs = &self.edgedata[&ek];
            let mut attr_map: std::collections::BTreeMap<String, f64> = std::collections::BTreeMap::new();
            for (k, v) in attrs {
                attr_map.insert(k.clone(), *v as f64);
            }
            edge_data_vec.push(crate::proto::EdgeData {
                vertex1: *v1 as u64,
                vertex2: *v2 as u64,
                attributes: attr_map,
            });
        }

        // P6: bulk colours go out as packed floats, 4 per colour. A list holding a NAMED
        // colour keeps the old sub-message shape, because the packed form has nowhere to
        // put a name and Color equality compares it.
        let pointcolors_rgba = Color::pack(&self.pointcolors);
        let facecolors_rgba = Color::pack(&self.facecolors);
        let linecolors_rgba = Color::pack(&self.linecolors);

        let mut triangulation_map: HashMap<u64, crate::proto::TriList> = HashMap::new();
        for (&fkey, tris) in &self.triangulation {
            let mut tri_list = crate::proto::TriList { vertices: Vec::new() };
            for t in tris {
                tri_list.vertices.push(t[0] as u64);
                tri_list.vertices.push(t[1] as u64);
                tri_list.vertices.push(t[2] as u64);
            }
            triangulation_map.insert(fkey as u64, tri_list);
        }

        crate::proto::Mesh {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            vertices,
            faces,
            edge_data: edge_data_vec,
            default_vertex_attributes: self.default_vertex_attributes.iter().map(|(k, v)| (k.clone(), *v as f64)).collect(),
            default_face_attributes: self.default_face_attributes.iter().map(|(k, v)| (k.clone(), *v as f64)).collect(),
            default_edge_attributes: self.default_edge_attributes.iter().map(|(k, v)| (k.clone(), *v as f64)).collect(),
            pointcolors_rgba,
            facecolors_rgba,
            linecolors_rgba,
            widths: self.widths.iter().map(|&v| v as f64).collect(),
            objectcolor: Some(crate::proto::Color {
                guid: self.objectcolor.guid().to_string(),
                name: self.objectcolor.name.clone(),
                r: self.objectcolor.r,
                g: self.objectcolor.g,
                b: self.objectcolor.b,
                a: self.objectcolor.a,
            }),
            color_mode: self.color_mode.to_i32(),
            triangulation: triangulation_map,
        }
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        Ok(Self::from_proto(crate::proto::Mesh::decode(data)?))
    }

    /// Build from an already-decoded proto — pb_loads decodes then calls this.
    pub fn from_proto(proto: crate::proto::Mesh) -> Self {
        let mut mesh = Self::new();
        mesh.set_guid(proto.guid.clone());
        mesh.name = proto.name;

        // Sized up front: a 360k-vertex sheet otherwise rehashes the whole table a dozen times
        // on the way up, and the growth copies dominate the insert.
        mesh.vertex.reserve(proto.vertices.len());
        mesh.face.reserve(proto.faces.len());
        for (vkey, vdata) in proto.vertices {
            mesh.vertex.insert(vkey as usize, VertexData {
                x: vdata.x,
                y: vdata.y,
                z: vdata.z,
                // MOVED, not rebuilt: prost already decoded this map, and the old copy loop
                // re-hashed every key into a second allocation for an identical result.
                attributes: vdata.attributes.into_iter().collect(),
            });
        }

        for (fkey, fdata) in proto.faces {
            let verts: Vec<usize> = fdata.vertices.iter().map(|&v| v as usize).collect();
            mesh.face.insert(fkey as usize, verts);
            if !fdata.attributes.is_empty() {
                mesh.facedata.insert(fkey as usize, fdata.attributes.into_iter().collect());
            }
            if !fdata.holes.is_empty() {
                let rings: Vec<Vec<usize>> = fdata.holes.iter()
                    .map(|h| h.vertices.iter().map(|&v| v as usize).collect())
                    .collect();
                mesh.face_holes.insert(fkey as usize, rings);
            }
        }

        for (fkey, tri_list) in proto.triangulation {
            let vlist = &tri_list.vertices;
            let mut tris: Vec<[usize; 3]> = Vec::new();
            let mut i = 0;
            while i + 2 < vlist.len() {
                tris.push([vlist[i] as usize, vlist[i+1] as usize, vlist[i+2] as usize]);
                i += 3;
            }
            mesh.triangulation.insert(fkey as usize, tris);
        }

        // Topology is LAZY: the wire may carry a halfedges map (older writers), but decoding
        // it into a nested HashMap per vertex was the single biggest load cost for dense
        // scenes - and the viewer never reads it. `ensure_halfedges` rebuilds from faces the
        // first time an EDIT needs it; `to_proto` computes it transiently so the wire format
        // is unchanged. The pure readers (is_closed, edges, boundaries) are face-based.

        for edata in proto.edge_data {
            let key = (edata.vertex1 as usize, edata.vertex2 as usize);
            mesh.edgedata.insert(key, edata.attributes.into_iter().collect());
        }

        mesh.default_vertex_attributes = proto.default_vertex_attributes.into_iter().collect();
        mesh.default_face_attributes = proto.default_face_attributes.into_iter().collect();
        mesh.default_edge_attributes = proto.default_edge_attributes.into_iter().collect();

        mesh.pointcolors = Color::unpack(&proto.pointcolors_rgba);
        mesh.facecolors = Color::unpack(&proto.facecolors_rgba);
        mesh.linecolors = Color::unpack(&proto.linecolors_rgba);

        mesh.widths = proto.widths.into_iter().map(|v| v as f64).collect();

        if let Some(oc) = proto.objectcolor {
            mesh.objectcolor = Color::new(oc.r, oc.g, oc.b, oc.a);
            mesh.objectcolor.set_guid(oc.guid.clone());
            mesh.objectcolor.name = oc.name;
        }
        mesh.color_mode = ColorMode::from_i32(proto.color_mode);

        // Update max_vertex and max_face
        if let Some(&max_v) = mesh.vertex.keys().max() {
            mesh.max_vertex = max_v + 1;
        }
        if let Some(&max_f) = mesh.face.keys().max() {
            mesh.max_face = max_f + 1;
        }

        mesh
    }

    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Triangle BVH Cache
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn build_triangle_bvh(&mut self) {
        self.ensure_triangle_bvh();
    }

    pub fn invalidate_triangle_bvh(&mut self) {
        self.tri_bvh = None;
        self.tri_tris.clear();
        self.tri_vertices.clear();
        self.gpu_cache.0 = None; // geometry changed → cached GPU buffers are stale
    }

    pub fn ensure_triangle_bvh(&mut self) {
        if self.tri_bvh.is_some() && !self.tri_tris.is_empty() && !self.tri_vertices.is_empty() {
            return;
        }

        let (vertices, faces) = self.to_vertices_and_faces();
        let mut vertex_keys: Vec<usize> = self.vertex.keys().cloned().collect();
        vertex_keys.sort();
        let vkey_to_idx: HashMap<usize, usize> = vertex_keys.iter().enumerate().map(|(i, &k)| (k, i)).collect();
        let mut face_keys: Vec<usize> = self.face.keys().cloned().collect();
        face_keys.sort();
        let mut tris: Vec<[usize; 3]> = Vec::new();
        let mut tri_boxes: Vec<OBB> = Vec::new();

        for (fi, face) in faces.iter().enumerate() {
            if face.len() < 3 {
                continue;
            }
            if face.len() >= 5 && fi < face_keys.len() {
                let fk = face_keys[fi];
                if let Some(stored) = self.triangulation.get(&fk) {
                    for t in stored {
                        let i0 = vkey_to_idx[&t[0]];
                        let i1 = vkey_to_idx[&t[1]];
                        let i2 = vkey_to_idx[&t[2]];
                        tris.push([i0, i1, i2]);
                        let pts = [vertices[i0].clone(), vertices[i1].clone(), vertices[i2].clone()];
                        tri_boxes.push(OBB::from_points(&pts, 0.0));
                    }
                    continue;
                }
            }
            let v0 = face[0];
            for i in 1..(face.len() - 1) {
                let t = [v0, face[i], face[i + 1]];
                tris.push(t);
                let pts = [vertices[t[0]].clone(), vertices[t[1]].clone(), vertices[t[2]].clone()];
                tri_boxes.push(OBB::from_points(&pts, 0.0));
            }
        }

        if tris.is_empty() {
            self.tri_bvh = None;
            self.tri_tris.clear();
            self.tri_vertices = vertices; // keep for consistency
            return;
        }

        let world_size = SpatialBVH::compute_world_size(&tri_boxes);
        let bvh = SpatialBVH::from_boxes(&tri_boxes, world_size);
        self.tri_vertices = vertices;
        self.tri_tris = tris;
        self.tri_bvh = Some(bvh);
    }

    pub fn triangle_bvh_ray_cast(&mut self, ray: &Line, epsilon: f64) -> Option<Point> {
        self.ray_cast_bvh(ray, epsilon)
    }

    /// True when the triangle BVH cache is built and usable by `ray_cast_bvh_ready`.
    pub fn has_triangle_bvh(&self) -> bool {
        self.tri_bvh.is_some() && !self.tri_tris.is_empty() && !self.tri_vertices.is_empty()
    }

    pub fn ray_cast_bvh(&mut self, ray: &Line, epsilon: f64) -> Option<Point> {
        self.ensure_triangle_bvh();
        self.ray_cast_bvh_ready(ray, epsilon)
    }

    /// Read-only cast against an ALREADY-BUILT triangle BVH (see `has_triangle_bvh` /
    /// `build_triangle_bvh`) — lets shared (Rc) meshes cast without a COW split.
    pub fn ray_cast_bvh_ready(&self, ray: &Line, epsilon: f64) -> Option<Point> {
        let bvh = match &self.tri_bvh {
            Some(b) => b,
            None => return None,
        };

        let origin = ray.start();
        let dir = ray.to_vector();
        let len = dir.magnitude();
        if len <= Tolerance::ZERO_TOLERANCE {
            return None;
        }
        let dir_unit = Vector::new(dir[0] / len, dir[1] / len, dir[2] / len);

        let mut candidate_ids: Vec<usize> = Vec::new();
        bvh.ray_cast(&origin, &dir_unit, &mut candidate_ids, true);
        if candidate_ids.is_empty() {
            return None;
        }

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
            if let Some(p) = crate::intersection::ray_triangle(ray, v0, v1, v2, epsilon) {
                let dx = p[0] - origin[0];
                let dy = p[1] - origin[1];
                let dz = p[2] - origin[2];
                let t = dx * dir_unit[0] + dy * dir_unit[1] + dz * dir_unit[2];
                if t >= 0.0 && t < best_t {
                    best_t = t;
                    best_p = Some(p);
                }
            }
        }

        best_p
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Color and Width Management
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn set_vertex_color(&mut self, index: usize, color: Color) {
        if index < self.pointcolors.len() {
            self.pointcolors[index] = color;
        }
    }

    pub fn set_face_color(&mut self, index: usize, color: Color) {
        if index < self.facecolors.len() {
            self.facecolors[index] = color;
        }
    }

    pub fn set_edge_color(&mut self, index: usize, color: Color) {
        if index < self.linecolors.len() {
            self.linecolors[index] = color;
        }
    }

    pub fn set_edge_width(&mut self, index: usize, width: f64) {
        if index < self.widths.len() {
            self.widths[index] = width;
        }
    }

    pub fn set_face_triangulation(&mut self, fk: usize, tris: Vec<[usize; 3]>) {
        self.triangulation.insert(fk, tris);
    }

    pub fn str(&self) -> String {
        format!("Mesh(name={}, vertices={}, faces={})",
            self.name, self.number_of_vertices(), self.number_of_faces())
    }

    pub fn repr(&self) -> String {
        format!("Mesh(\n  name={},\n  vertices={},\n  faces={},\n  edges={}\n)",
            self.name, self.number_of_vertices(), self.number_of_faces(), self.number_of_edges())
    }
}

impl std::fmt::Display for Mesh {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Mesh(name={}, vertices={}, faces={})",
            self.name, self.number_of_vertices(), self.number_of_faces())
    }
}

impl std::fmt::Debug for Mesh {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Mesh(\n  name={},\n  vertices={},\n  faces={},\n  edges={}\n)",
            self.name, self.number_of_vertices(), self.number_of_faces(), self.number_of_edges())
    }
}

impl PartialEq for Mesh {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name && self.vertex == other.vertex
            && self.face == other.face
    }
}

#[cfg(test)]
#[path = "mesh_test.rs"]
mod mesh_test;