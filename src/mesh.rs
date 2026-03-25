use crate::{Obb, Color, Line, Point, Tolerance, Vector, Xform, BVH};
use crate::polyline::Polyline;
use crate::remesh_cdt;
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
    pub guid: String,                                            // Unique identifier
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
    #[serde(default = "Xform::identity")]
    pub xform: Xform,   // Transformation matrix
    // Cached triangle BVH for ray queries (not serialized)
    #[serde(skip)]
    pub tri_bvh: Option<BVH>,
    #[serde(skip)]
    pub tri_tris: Vec<[usize; 3]>,
    #[serde(skip)]
    pub tri_vertices: Vec<Point>,
}

/// Vertex data containing position and attributes
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VertexData {
    pub x: f64,                           // X coordinate
    pub y: f64,                           // Y coordinate
    pub z: f64,                           // Z coordinate
    pub attributes: HashMap<String, f64>, // Vertex attributes
}

impl VertexData {
    pub fn new(point: Point) -> Self {
        Self {
            x: point[0],
            y: point[1],
            z: point[2],
            attributes: HashMap::new(),
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
            guid: uuid::Uuid::new_v4().to_string(),
            name: "my_mesh".to_string(),
            pointcolors: Vec::new(),
            facecolors: Vec::new(),
            linecolors: Vec::new(),
            widths: Vec::new(),
            objectcolor: Color::white(),
            color_mode: ColorMode::OBJECTCOLOR,
            xform: Xform::identity(),
            tri_bvh: None,
            tri_tris: Vec::new(),
            tri_vertices: Vec::new(),
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
                    let bpts: Vec<(f64, f64)> = poly[..nk].iter().map(|p| {
                        (p[0]*ux + p[1]*uy + p[2]*uz,
                         p[0]*vx + p[1]*vy + p[2]*vz)
                    }).collect();
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
                let mut bpts: Vec<(f64, f64)> = ordered.iter().map(|&i| (verts[i][0], verts[i][1])).collect();
                let area: f64 = (0..bpts.len()).map(|j| {
                    let k = (j + 1) % bpts.len();
                    bpts[j].0 * bpts[k].1 - bpts[k].0 * bpts[j].1
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

    pub fn from_polygon_with_holes(polylines: &[Vec<Point>], sort_by_bbox: bool) -> Self {
        if polylines.is_empty() { return Mesh::new(); }
        let mut border_idx = 0usize;
        if sort_by_bbox && polylines.len() > 1 {
            let mut max_diag = 0.0_f64;
            for (i, poly) in polylines.iter().enumerate() {
                if poly.len() < 3 { continue; }
                let (mut minx, mut miny, mut minz) = (poly[0][0], poly[0][1], poly[0][2]);
                let (mut maxx, mut maxy, mut maxz) = (minx, miny, minz);
                for p in poly {
                    if p[0] < minx { minx = p[0]; } if p[0] > maxx { maxx = p[0]; }
                    if p[1] < miny { miny = p[1]; } if p[1] > maxy { maxy = p[1]; }
                    if p[2] < minz { minz = p[2]; } if p[2] > maxz { maxz = p[2]; }
                }
                let (dx, dy, dz) = (maxx - minx, maxy - miny, maxz - minz);
                let diag = (dx*dx + dy*dy + dz*dz).sqrt();
                if diag > max_diag { max_diag = diag; border_idx = i; }
            }
        }
        let strip_close = |pts: &[Point]| -> Vec<Point> {
            if pts.len() > 1 {
                let f = &pts[0]; let b = &pts[pts.len()-1];
                if (f[0]-b[0]).abs() < 1e-12 && (f[1]-b[1]).abs() < 1e-12 && (f[2]-b[2]).abs() < 1e-12 {
                    return pts[..pts.len()-1].to_vec();
                }
            }
            pts.to_vec()
        };
        let mut border = strip_close(&polylines[border_idx]);
        if border.len() < 3 { return Mesh::new(); }
        let border_pl = Polyline::new(border.clone());
        let (origin, xaxis, yaxis, _zaxis) = border_pl.get_average_plane();
        let project_2d = |p: &Point| -> Point {
            let dx = p[0] - origin[0]; let dy = p[1] - origin[1]; let dz = p[2] - origin[2];
            let u = dx * xaxis[0] + dy * xaxis[1] + dz * xaxis[2];
            let v = dx * yaxis[0] + dy * yaxis[1] + dz * yaxis[2];
            Point::new(u, v, 0.0)
        };
        let mut boundary_2d: Vec<Point> = border.iter().map(|p| project_2d(p)).collect();
        let signed_area = |pts: &[Point]| -> f64 {
            let n = pts.len();
            let mut area = 0.0;
            for i in 0..n {
                let j = (i + 1) % n;
                area += pts[i][0] * pts[j][1] - pts[j][0] * pts[i][1];
            }
            area * 0.5
        };
        if signed_area(&boundary_2d) < 0.0 {
            border.reverse();
            boundary_2d.reverse();
        }
        let mut holes_2d: Vec<Vec<Point>> = Vec::new();
        let mut hole_pts_3d: Vec<Vec<Point>> = Vec::new();
        for (i, poly) in polylines.iter().enumerate() {
            if i == border_idx { continue; }
            let mut hole = strip_close(poly);
            if hole.len() < 3 { continue; }
            let mut hole_2d: Vec<Point> = hole.iter().map(|p| project_2d(p)).collect();
            if signed_area(&hole_2d) > 0.0 {
                hole.reverse();
                hole_2d.reverse();
            }
            holes_2d.push(hole_2d);
            hole_pts_3d.push(hole);
        }
        let b2d: Vec<(f64,f64)> = boundary_2d.iter().map(|p| (p[0], p[1])).collect();
        let h2d: Vec<Vec<(f64,f64)>> = holes_2d.iter().map(|h| h.iter().map(|p| (p[0], p[1])).collect()).collect();
        let tris = remesh_cdt::cdt_triangulate(&b2d, &h2d);
        let mut all_pts = border.clone();
        for h in &hole_pts_3d { all_pts.extend(h.iter().cloned()); }
        let mut mesh = Mesh::new();
        let mut vkeys: Vec<usize> = Vec::with_capacity(all_pts.len());
        for p in &all_pts { vkeys.push(mesh.add_vertex(p.clone(), None)); }
        if hole_pts_3d.is_empty() {
            let fvkeys: Vec<usize> = (0..border.len()).map(|i| vkeys[i]).collect();
            if let Some(fkey) = mesh.add_face(fvkeys, None) {
                let mut tri_list: Vec<[usize; 3]> = Vec::new();
                for &(a, b, c) in &tris {
                    if vkeys[a] == vkeys[b] || vkeys[b] == vkeys[c] || vkeys[c] == vkeys[a] { continue; }
                    tri_list.push([vkeys[a], vkeys[b], vkeys[c]]);
                }
                let n_vk = border.len();
                let covered: std::collections::HashSet<usize> = tri_list.iter().flat_map(|t| t.iter().copied()).collect();
                for m in 0..n_vk {
                    if !covered.contains(&vkeys[m]) {
                        tri_list.push([vkeys[(m + n_vk - 1) % n_vk], vkeys[m], vkeys[(m + 1) % n_vk]]);
                    }
                }
                mesh.triangulation.insert(fkey, tri_list);
            }
        } else {
            let fvkeys: Vec<usize> = (0..border.len()).map(|i| vkeys[i]).collect();
            if let Some(fkey) = mesh.add_face(fvkeys, None) {
                let mut hole_rings: Vec<Vec<usize>> = Vec::new();
                let mut off = border.len();
                for h in &hole_pts_3d {
                    let ring: Vec<usize> = (off..off+h.len()).map(|i| vkeys[i]).collect();
                    hole_rings.push(ring); off += h.len();
                }
                mesh.face_holes.insert(fkey, hole_rings);
                let tri_list: Vec<[usize; 3]> = tris.iter()
                    .filter(|&&(a, b, c)| vkeys[a] != vkeys[b] && vkeys[b] != vkeys[c] && vkeys[c] != vkeys[a])
                    .map(|&(a, b, c)| [vkeys[a], vkeys[b], vkeys[c]])
                    .collect();
                mesh.triangulation.insert(fkey, tri_list);
            }
        }
        mesh
    }

    pub fn loft(polylines0: &[Polyline], polylines1: &[Polyline], cap: bool) -> Self {
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
        let proj = |p: &Point| -> (f64, f64) {
            let dx = p[0]-origin[0]; let dy = p[1]-origin[1]; let dz = p[2]-origin[2];
            (dx*xaxis[0]+dy*xaxis[1]+dz*xaxis[2], dx*yaxis[0]+dy*yaxis[1]+dz*yaxis[2])
        };
        let sarea = |pts: &[Point]| -> f64 {
            let n = pts.len();
            let mut a = 0.0;
            for i in 0..n {
                let j = (i+1) % n;
                let (xi, yi) = proj(&pts[i]); let (xj, yj) = proj(&pts[j]);
                a += xi*yj - xj*yi;
            }
            a * 0.5
        };
        let mut order: Vec<usize> = vec![border_idx];
        for i in 0..polylines0.len() { if i != border_idx { order.push(i); } }
        let mut poly_infos: Vec<(usize, usize, usize, usize)> = Vec::new(); // (bot_off, bot_n, top_off, top_n)
        let mut all_bot: Vec<Point> = Vec::new();
        let mut all_top: Vec<Point> = Vec::new();
        for (oi, &idx) in order.iter().enumerate() {
            let mut bot = get_open(&polylines0[idx]);
            let mut top = get_open(&polylines1[idx]);
            let area = sarea(&bot);
            if (oi == 0 && area < 0.0) || (oi != 0 && area > 0.0) {
                bot.reverse(); top.reverse();
            }
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
            let b2d: Vec<(f64,f64)> = (0..bot_n0).map(|i| proj(&all_bot[i])).collect();
            let bh2d: Vec<Vec<(f64,f64)>> = poly_infos[1..].iter().map(|&(off,cnt,_,_)| {
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
                let tri_list: Vec<[usize;3]> = b_tris.iter().map(|&(a,b,c)| [bvk[a], bvk[c], bvk[b]]).collect();
                mesh.triangulation.insert(fk_bot, tri_list);
            }
            // Top cap CDT
            let t2d: Vec<(f64,f64)> = (0..top_n0).map(|i| proj(&all_top[i])).collect();
            let th2d: Vec<Vec<(f64,f64)>> = poly_infos[1..].iter().map(|&(_,_,off,cnt)| {
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
                let tri_list: Vec<[usize;3]> = t_tris.iter().map(|&(a,b,c)| [tvk[a], tvk[b], tvk[c]]).collect();
                mesh.triangulation.insert(fk_top, tri_list);
            }
            for &(b_off, b_n, t_off, t_n) in &poly_infos[1..] {
                let bh: Vec<usize> = (0..b_n).rev().map(|j| bvk[b_off + j]).collect();
                mesh.add_face(bh, None);
                let th: Vec<usize> = (0..t_n).map(|j| tvk[t_off + j]).collect();
                mesh.add_face(th, None);
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
            let ib = (0..top_n).max_by(|&a, &b| edsq(tpts, a).partial_cmp(&edsq(tpts, b)).unwrap()).unwrap_or(0);
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

    pub fn loft_many(pairs: Vec<(Vec<Polyline>, Vec<Polyline>)>, cap: bool, parallel: bool) -> Vec<Self> {
        if parallel && pairs.len() > 1 {
            use rayon::prelude::*;
            pairs.into_par_iter().map(|(p0, p1)| Mesh::loft(&p0, &p1, cap)).collect()
        } else {
            pairs.iter().map(|(p0, p1)| Mesh::loft(p0, p1, cap)).collect()
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
                            let bpts: Vec<(f64,f64)> = top_pts.iter().map(|p| (p[0]*ux+p[1]*uy+p[2]*uz, p[0]*vx+p[1]*vy+p[2]*vz)).collect();
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
            let mut bot_to_top = vec![-1i64; m]; let mut bot_dist = vec![1e300f64; m];
            for j in 0..m {
                for i in 0..n {
                    let d = bot_mids[j].distance(&top_mids[i], None);
                    if d < bot_dist[j] { bot_dist[j] = d; bot_to_top[j] = i as i64; }
                }
            }
            let mut top_to_bot = vec![-1i64; n]; let mut top_dist = vec![1e300f64; n];
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
                    let mut best_d = 1e300f64; let mut best_tv = 0usize;
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
                    let mut best_d = 1e300f64; let mut best_bv = 0usize;
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
                            let bpts2: Vec<(f64,f64)> = bot_pts.iter().map(|p| (p[0]*bcux+p[1]*bcuy+p[2]*bcuz, p[0]*bcvx+p[1]*bcvy+p[2]*bcvz)).collect();
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
        Mesh::from_polylines(faces, Some(1e-10))
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
        for (_, nbrs) in &self.halfedge {
            for (_, fkey) in nbrs {
                if fkey.is_none() { return false; }
            }
        }
        !self.halfedge.is_empty()
    }

    pub fn is_vertex_on_boundary(&self, vertex_key: usize) -> bool {
        if let Some(neigh) = self.halfedge.get(&vertex_key) {
            for (_v, face_opt) in neigh.iter() {
                if face_opt.is_none() {
                    return true;
                }
            }
        }

        for (_u, neigh) in self.halfedge.iter() {
            if let Some(face_opt) = neigh.get(&vertex_key) {
                if face_opt.is_none() {
                    return true;
                }
            }
        }
        false
    }

    pub fn is_edge_on_boundary(&self, u: usize, v: usize) -> bool {
        let f0 = self.halfedge.get(&u).and_then(|m| m.get(&v));
        let f1 = self.halfedge.get(&v).and_then(|m| m.get(&u));
        f0.map_or(true, |f| f.is_none()) || f1.map_or(true, |f| f.is_none())
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
        let mut seen = HashSet::new();
        let mut count = 0;

        for u in self.halfedge.keys() {
            if let Some(neighbors) = self.halfedge.get(u) {
                for v in neighbors.keys() {
                    let edge = if u < v { (*u, *v) } else { (*v, *u) };
                    if seen.insert(edge) {
                        count += 1;
                    }
                }
            }
        }

        count
    }

    pub fn edges(&self) -> Vec<(usize, usize)> {
        let mut seen = HashSet::new();
        let mut outer: Vec<usize> = self.halfedge.keys().cloned().collect();
        outer.sort();
        let mut result = Vec::new();
        for u in outer {
            let mut inner: Vec<usize> = self.halfedge[&u].keys().cloned().collect();
            inner.sort();
            for v in inner {
                let edge = if u < v { (u, v) } else { (v, u) };
                if seen.insert(edge) {
                    result.push(edge);
                }
            }
        }
        result
    }

    pub fn naked_edges(&self, boundary: bool) -> Vec<(usize, usize)> {
        let mut seen = HashSet::new();
        let mut outer: Vec<usize> = self.halfedge.keys().cloned().collect();
        outer.sort();
        let mut result = Vec::new();
        for u in outer {
            let mut inner: Vec<usize> = self.halfedge[&u].keys().cloned().collect();
            inner.sort();
            for v in inner {
                let edge = if u < v { (u, v) } else { (v, u) };
                if seen.insert(edge) {
                    if self.is_edge_on_boundary(edge.0, edge.1) == boundary {
                        result.push(edge);
                    }
                }
            }
        }
        result
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

    pub fn set_pointcolors(&mut self, v: Vec<Color>) { self.pointcolors = v; self.color_mode = ColorMode::POINTCOLORS; }
    pub fn set_facecolors(&mut self, v: Vec<Color>) { self.facecolors = v; self.color_mode = ColorMode::FACECOLORS; }
    pub fn set_linecolors(&mut self, v: Vec<Color>, w: Vec<f64>) { self.linecolors = v; if !w.is_empty() { self.widths = w; } }
    pub fn set_objectcolor(&mut self, c: Color) { self.objectcolor = c; }

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
            let boxes: Vec<Obb> = positions.iter().map(|p| Obb::from_point(p.clone(), tolerance)).collect();
            let ws = BVH::compute_world_size(&boxes);
            let bvh = BVH::from_boxes(&boxes, ws);
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
                self.linecolors.push(Color::white());
                self.widths.push(1.0);
            }
        }

        Some(face_key)
    }

    pub fn remove_face(&mut self, fkey: usize) {
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
        let uv = self.halfedge.get(&u).map_or(false, |m| m.contains_key(&v));
        let vu = self.halfedge.get(&v).map_or(false, |m| m.contains_key(&u));
        if !uv && !vu { return None; }
        let mut edges = Vec::new();
        if let Some(neighbors) = self.halfedge.get(&u) {
            let mut keys: Vec<usize> = neighbors.keys().copied().collect();
            keys.sort();
            for w in keys { if w != v { edges.push((u, w)); } }
        }
        if let Some(neighbors) = self.halfedge.get(&v) {
            let mut keys: Vec<usize> = neighbors.keys().copied().collect();
            keys.sort();
            for w in keys { if w != u { edges.push((v, w)); } }
        }
        Some(edges)
    }

    pub fn edge_faces(&self, u: usize, v: usize) -> Option<Vec<usize>> {
        let f0 = self.halfedge.get(&u).and_then(|m| m.get(&v)).copied().flatten();
        let f1 = self.halfedge.get(&v).and_then(|m| m.get(&u)).copied().flatten();
        if f0.is_none() && f1.is_none() { return None; }
        Some([f0, f1].iter().filter_map(|f| *f).collect())
    }

    pub fn edge_line(&self, u: usize, v: usize) -> Option<Line> {
        let uv = self.halfedge.get(&u).map_or(false, |m| m.contains_key(&v));
        let vu = self.halfedge.get(&v).map_or(false, |m| m.contains_key(&u));
        if !uv && !vu { return None; }
        Some(Line::from_points(&self.vertex_point(u)?, &self.vertex_point(v)?))
    }

    pub fn face_edges(&self, face_key: usize) -> Option<Vec<(usize, usize)>> {
        let verts = self.face.get(&face_key)?;
        let n = verts.len();
        Some((0..n).map(|i| (verts[i], verts[(i + 1) % n])).collect())
    }

    pub fn face_faces(&self, face_key: usize) -> Option<Vec<usize>> {
        let fe = self.face_edges(face_key)?;
        Some(fe.into_iter()
            .filter_map(|(u, v)| self.halfedge.get(&v)?.get(&u).copied().flatten())
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
        let neighbors = self.halfedge.get(&vertex_key)?;
        let mut keys: Vec<usize> = neighbors.keys().copied().collect();
        keys.sort();
        Some(keys.into_iter().map(|u| (vertex_key, u)).collect())
    }

    pub fn vertex_faces(&self, vertex_key: usize) -> Option<Vec<usize>> {
        let neighbors = self.halfedge.get(&vertex_key)?;
        let mut keys: Vec<usize> = neighbors.keys().copied().collect();
        keys.sort();
        Some(keys.into_iter().filter_map(|k| neighbors[&k]).collect())
    }

    pub fn vertex_point(&self, vertex_key: usize) -> Option<Point> {
        self.vertex.get(&vertex_key).map(|v| v.position())
    }

    pub fn vertex_vertices(&self, vertex_key: usize) -> Option<Vec<usize>> {
        let neighbors = self.halfedge.get(&vertex_key)?;
        let mut keys: Vec<usize> = neighbors.keys().copied().collect();
        keys.sort();
        Some(keys)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Geometric Properties
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn area(&self) -> f64 {
        let mut total = 0.0;
        for fk in self.face.keys() {
            if let Some(a) = self.face_area(*fk) {
                total += a;
            }
        }
        total
    }

    pub fn centroid(&self) -> Point {
        let mut x = 0.0_f64; let mut y = 0.0_f64; let mut z = 0.0_f64;
        for vk in self.vertex.keys() {
            let p = self.vertex_point(*vk).unwrap();
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
        Some((std::f64::consts::PI - dot.acos()) * 180.0 / std::f64::consts::PI)
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
                pt.pointcolor = Color::new(240, 220, 0, 255);
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
            arc.linecolor = Color::new(240, 220, 0, 255);
            arcs.push(arc);
            let mid = &arc_pts[arc_n/2];
            let mut pt = Point::new(mid[0], mid[1], mid[2]);
            pt.name = deg.to_string();
            pt.pointcolor = Color::new(240, 220, 0, 255);
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

    pub fn volume(&self) -> f64 {
        let mut total = 0.0;
        for (_, vkeys) in &self.face {
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

    pub fn transform(&mut self, xf: Option<&Xform>) {
        let xform = match xf {
            Some(x) => x.clone(),
            None => self.xform.clone(),
        };
        for v in self.vertex.values_mut() {
            let mut pt = Point::new(v.x, v.y, v.z);
            xform.transform_point(&mut pt);
            v.x = pt[0];
            v.y = pt[1];
            v.z = pt[2];
        }
        self.invalidate_triangle_bvh();
    }

    pub fn transformed(&self, xf: Option<&Xform>) -> Self {
        let mut result = self.clone();
        result.transform(xf);
        result
    }

    pub fn duplicate(&self) -> Self {
        self.clone_with_new_guid()
    }

    pub fn clone_with_new_guid(&self) -> Self {
        let mut m = self.clone();
        m.guid = uuid::Uuid::new_v4().to_string();
        m
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serializes the Mesh to JSON data
    pub fn jsondump(&self) -> serde_json::Value {
        let pointcolors_flat: Vec<u8> = self
            .pointcolors
            .iter()
            .flat_map(|c| vec![c.r, c.g, c.b, c.a])
            .collect();

        let facecolors_flat: Vec<u8> = self
            .facecolors
            .iter()
            .flat_map(|c| vec![c.r, c.g, c.b, c.a])
            .collect();

        let linecolors_flat: Vec<u8> = self
            .linecolors
            .iter()
            .flat_map(|c| vec![c.r, c.g, c.b, c.a])
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
            "guid": self.guid,
            "name": self.name,
            "vertex": self.vertex,
            "face": self.face,
            "face_holes": serde_json::Value::Object(face_holes_json),
            "halfedge": self.halfedge,
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
            "widths": self.widths,
            "xform": serde_json::to_value(&self.xform).unwrap_or(serde_json::Value::Null)
        })
    }

    pub fn jsonload(data: &serde_json::Value) -> Option<Self> {
        let mut mesh = Mesh::new();

        if let Some(guid) = data.get("guid").and_then(|v| v.as_str()) {
            mesh.guid = guid.to_string();
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
                .map(|c| Color::new(c[0], c[1], c[2], c[3]))
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
                .map(|c| Color::new(c[0], c[1], c[2], c[3]))
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
                .map(|c| Color::new(c[0], c[1], c[2], c[3]))
                .collect();
        }

        if let Some(widths) = data.get("widths").and_then(|v| v.as_array()) {
            mesh.widths = widths.iter().filter_map(|v| v.as_f64()).collect();
        }
        if let Some(xf) = data.get("xform") {
            if let Ok(xform) = serde_json::from_value::<crate::Xform>(xf.clone()) {
                mesh.xform = xform;
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

    pub fn json_dumps(&self) -> String {
        serde_json::to_string_pretty(&self.jsondump()).unwrap_or_default()
    }

    pub fn json_loads(json_string: &str) -> Self {
        let data: serde_json::Value = serde_json::from_str(json_string).unwrap_or_default();
        Self::jsonload(&data).unwrap_or_else(|| Self::new())
    }

    pub fn json_dump(&self, filename: &str) -> std::io::Result<()> {
        let data = self.jsondump();
        std::fs::write(filename, serde_json::to_string_pretty(&data)?)
    }

    pub fn json_load(filename: &str) -> std::io::Result<Self> {
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
        use std::collections::HashMap;

        let mut vertices: HashMap<u64, crate::proto::VertexData> = HashMap::new();
        for (&vkey, vdata) in &self.vertex {
            let mut attrs: HashMap<String, f64> = HashMap::new();
            for (k, v) in &vdata.attributes {
                attrs.insert(k.clone(), *v);
            }
            vertices.insert(vkey as u64, crate::proto::VertexData {
                x: vdata.x,
                y: vdata.y,
                z: vdata.z,
                attributes: attrs,
            });
        }

        let mut faces: HashMap<u64, crate::proto::FaceData> = HashMap::new();
        for (&fkey, fverts) in &self.face {
            let mut attrs: HashMap<String, f64> = HashMap::new();
            if let Some(fdata) = self.facedata.get(&fkey) {
                for (k, v) in fdata {
                    attrs.insert(k.clone(), *v);
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

        let mut halfedges: HashMap<u64, crate::proto::HalfedgeMap> = HashMap::new();
        for (&u, neighbors) in &self.halfedge {
            let mut neighbor_map: HashMap<u64, u64> = HashMap::new();
            for (&v, &fkey_opt) in neighbors {
                neighbor_map.insert(v as u64, fkey_opt.unwrap_or(usize::MAX) as u64);
            }
            halfedges.insert(u as u64, crate::proto::HalfedgeMap {
                neighbors: neighbor_map,
            });
        }

        let mut edge_data_vec: Vec<crate::proto::EdgeData> = Vec::new();
        for ((v1, v2), attrs) in &self.edgedata {
            let mut attr_map: HashMap<String, f64> = HashMap::new();
            for (k, v) in attrs {
                attr_map.insert(k.clone(), *v);
            }
            edge_data_vec.push(crate::proto::EdgeData {
                vertex1: *v1 as u64,
                vertex2: *v2 as u64,
                attributes: attr_map,
            });
        }

        let pointcolors: Vec<crate::proto::Color> = self.pointcolors.iter().map(|c| {
            crate::proto::Color {
                guid: c.guid.clone(),
                name: c.name.clone(),
                r: c.r as i32,
                g: c.g as i32,
                b: c.b as i32,
                a: c.a as i32,
            }
        }).collect();

        let facecolors: Vec<crate::proto::Color> = self.facecolors.iter().map(|c| {
            crate::proto::Color {
                guid: c.guid.clone(),
                name: c.name.clone(),
                r: c.r as i32,
                g: c.g as i32,
                b: c.b as i32,
                a: c.a as i32,
            }
        }).collect();

        let linecolors: Vec<crate::proto::Color> = self.linecolors.iter().map(|c| {
            crate::proto::Color {
                guid: c.guid.clone(),
                name: c.name.clone(),
                r: c.r as i32,
                g: c.g as i32,
                b: c.b as i32,
                a: c.a as i32,
            }
        }).collect();

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

        let proto = crate::proto::Mesh {
            guid: self.guid.clone(),
            name: self.name.clone(),
            vertices,
            faces,
            halfedges,
            edge_data: edge_data_vec,
            default_vertex_attributes: self.default_vertex_attributes.clone(),
            default_face_attributes: self.default_face_attributes.clone(),
            default_edge_attributes: self.default_edge_attributes.clone(),
            pointcolors,
            facecolors,
            linecolors,
            widths: self.widths.clone(),
            objectcolor: Some(crate::proto::Color {
                guid: self.objectcolor.guid.clone(),
                name: self.objectcolor.name.clone(),
                r: self.objectcolor.r as i32,
                g: self.objectcolor.g as i32,
                b: self.objectcolor.b as i32,
                a: self.objectcolor.a as i32,
            }),
            color_mode: self.color_mode.to_i32(),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid.clone(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
            triangulation: triangulation_map,
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        let proto = crate::proto::Mesh::decode(data)?;
        let mut mesh = Self::new();
        mesh.guid = proto.guid;
        mesh.name = proto.name;

        for (vkey, vdata) in proto.vertices {
            let mut attrs: std::collections::HashMap<String, f64> = std::collections::HashMap::new();
            for (k, v) in vdata.attributes {
                attrs.insert(k, v);
            }
            mesh.vertex.insert(vkey as usize, VertexData {
                x: vdata.x,
                y: vdata.y,
                z: vdata.z,
                attributes: attrs,
            });
            mesh.halfedge.entry(vkey as usize).or_insert_with(std::collections::HashMap::new);
        }

        for (fkey, fdata) in proto.faces {
            let verts: Vec<usize> = fdata.vertices.iter().map(|&v| v as usize).collect();
            mesh.face.insert(fkey as usize, verts);
            if !fdata.attributes.is_empty() {
                mesh.facedata.insert(fkey as usize, fdata.attributes);
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

        for (u, hmap) in proto.halfedges {
            let mut neighbors: std::collections::HashMap<usize, Option<usize>> = std::collections::HashMap::new();
            for (v, fkey) in hmap.neighbors {
                let fkey_opt = if fkey == u64::MAX { None } else { Some(fkey as usize) };
                neighbors.insert(v as usize, fkey_opt);
            }
            mesh.halfedge.insert(u as usize, neighbors);
        }

        for edata in proto.edge_data {
            let key = (edata.vertex1 as usize, edata.vertex2 as usize);
            mesh.edgedata.insert(key, edata.attributes);
        }

        mesh.default_vertex_attributes = proto.default_vertex_attributes;
        mesh.default_face_attributes = proto.default_face_attributes;
        mesh.default_edge_attributes = proto.default_edge_attributes;

        mesh.pointcolors = proto.pointcolors.iter().map(|c| {
            let mut color = Color::new(c.r as u8, c.g as u8, c.b as u8, c.a as u8);
            color.guid = c.guid.clone();
            color.name = c.name.clone();
            color
        }).collect();

        mesh.facecolors = proto.facecolors.iter().map(|c| {
            let mut color = Color::new(c.r as u8, c.g as u8, c.b as u8, c.a as u8);
            color.guid = c.guid.clone();
            color.name = c.name.clone();
            color
        }).collect();

        mesh.linecolors = proto.linecolors.iter().map(|c| {
            let mut color = Color::new(c.r as u8, c.g as u8, c.b as u8, c.a as u8);
            color.guid = c.guid.clone();
            color.name = c.name.clone();
            color
        }).collect();

        mesh.widths = proto.widths;

        if let Some(oc) = proto.objectcolor {
            mesh.objectcolor = Color::new(oc.r as u8, oc.g as u8, oc.b as u8, oc.a as u8);
            mesh.objectcolor.guid = oc.guid;
            mesh.objectcolor.name = oc.name;
        }
        mesh.color_mode = ColorMode::from_i32(proto.color_mode);

        if let Some(xform) = proto.xform {
            mesh.xform.guid = xform.guid;
            mesh.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 {
                    mesh.xform.m[i] = *val;
                }
            }
        }

        // Update max_vertex and max_face
        if let Some(&max_v) = mesh.vertex.keys().max() {
            mesh.max_vertex = max_v + 1;
        }
        if let Some(&max_f) = mesh.face.keys().max() {
            mesh.max_face = max_f + 1;
        }

        Ok(mesh)
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

    fn invalidate_triangle_bvh(&mut self) {
        self.tri_bvh = None;
        self.tri_tris.clear();
        self.tri_vertices.clear();
    }

    fn ensure_triangle_bvh(&mut self) {
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
        let mut tri_boxes: Vec<Obb> = Vec::new();

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
                        tri_boxes.push(Obb::from_points(&pts, 0.0));
                    }
                    continue;
                }
            }
            let v0 = face[0];
            for i in 1..(face.len() - 1) {
                let t = [v0, face[i], face[i + 1]];
                tris.push(t);
                let pts = [vertices[t[0]].clone(), vertices[t[1]].clone(), vertices[t[2]].clone()];
                tri_boxes.push(Obb::from_points(&pts, 0.0));
            }
        }

        if tris.is_empty() {
            self.tri_bvh = None;
            self.tri_tris.clear();
            self.tri_vertices = vertices; // keep for consistency
            return;
        }

        let world_size = BVH::compute_world_size(&tri_boxes);
        let bvh = BVH::from_boxes(&tri_boxes, world_size);
        self.tri_vertices = vertices;
        self.tri_tris = tris;
        self.tri_bvh = Some(bvh);
    }

    pub fn triangle_bvh_ray_cast(&mut self, ray: &Line, epsilon: f64) -> Option<Point> {
        self.ray_cast_bvh(ray, epsilon)
    }

    pub fn ray_cast_bvh(&mut self, ray: &Line, epsilon: f64) -> Option<Point> {
        self.ensure_triangle_bvh();
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
            && self.face == other.face && self.xform == other.xform
    }
}

#[cfg(test)]
#[path = "mesh_test.rs"]
mod mesh_test;
