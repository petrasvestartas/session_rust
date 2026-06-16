use crate::closest::Closest;
use crate::nurbssurface::NurbsSurface;
use crate::nurbscurve::NurbsCurve;
use crate::primitives::Primitives;
use crate::xform::Xform;
use crate::color::Color;
use crate::point::Point;
use crate::vector::Vector;
use crate::mesh::Mesh;
use serde::{Serialize, Deserialize};

// ---- Bowyer-Watson Constrained Delaunay Triangulation in 2D UV space ----

pub struct Vertex2D { pub x: f64, pub y: f64 }

pub struct Triangle {
    pub v: [i32; 3],
    pub adj: [i32; 3],
    pub constrained: [bool; 3],
    pub alive: bool,
}

pub struct Delaunay2D {
    pub vertices: Vec<Vertex2D>,
    pub triangles: Vec<Triangle>,
    super_v: [i32; 3],
    edge_map: std::collections::HashMap<(i32,i32),(i32,i32)>,
    last_found: i32,
}

impl Delaunay2D {
    fn edge_key(a: i32, b: i32) -> (i32, i32) { (a.min(b), a.max(b)) }

    fn in_circumcircle(ax:f64,ay:f64,bx:f64,by:f64,cx:f64,cy:f64,dx:f64,dy:f64) -> f64 {
        let adx=ax-dx; let ady=ay-dy; let bdx=bx-dx; let bdy=by-dy; let cdx=cx-dx; let cdy=cy-dy;
        (adx*adx+ady*ady)*(bdx*cdy-cdx*bdy)
       +(bdx*bdx+bdy*bdy)*(cdx*ady-adx*cdy)
       +(cdx*cdx+cdy*cdy)*(adx*bdy-bdx*ady)
    }

    fn orient2d(ax:f64,ay:f64,bx:f64,by:f64,cx:f64,cy:f64) -> f64 {
        (bx-ax)*(cy-ay)-(by-ay)*(cx-ax)
    }

    pub fn new(xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Self {
        let dx = xmax-xmin; let dy = ymax-ymin; let d = dx.max(dy);
        let cx = (xmin+xmax)*0.5; let cy = (ymin+ymax)*0.5; let scale = 20.0;
        let mut dt = Delaunay2D {
            vertices: Vec::new(),
            triangles: Vec::new(),
            super_v: [-1; 3],
            edge_map: std::collections::HashMap::new(),
            last_found: 0,
        };
        dt.vertices.push(Vertex2D { x: cx-scale*d, y: cy-scale*d });
        dt.vertices.push(Vertex2D { x: cx+scale*d, y: cy-scale*d });
        dt.vertices.push(Vertex2D { x: cx,          y: cy+scale*d });
        dt.super_v = [0, 1, 2];
        dt.triangles.push(Triangle { v: [0,1,2], adj: [-1,-1,-1], constrained: [false;3], alive: true });
        dt.register_edges(0);
        dt
    }

    fn register_edges(&mut self, ti: i32) {
        for k in 0..3 {
            let a = self.triangles[ti as usize].v[(k+1)%3];
            let b = self.triangles[ti as usize].v[(k+2)%3];
            let key = Self::edge_key(a, b);
            if let Some(&(oti, ok)) = self.edge_map.get(&key) {
                self.triangles[ti as usize].adj[k] = oti;
                self.triangles[oti as usize].adj[ok as usize] = ti;
                self.edge_map.remove(&key);
            } else {
                self.edge_map.insert(key, (ti, k as i32));
            }
        }
    }

    fn unregister_edges(&mut self, ti: i32) {
        for k in 0..3 {
            let a = self.triangles[ti as usize].v[(k+1)%3];
            let b = self.triangles[ti as usize].v[(k+2)%3];
            let key = Self::edge_key(a, b);
            let adj_ti = self.triangles[ti as usize].adj[k];
            if adj_ti >= 0 && adj_ti < self.triangles.len() as i32 && self.triangles[adj_ti as usize].alive {
                let mut adj_kk = -1i32;
                for kk in 0..3 { if self.triangles[adj_ti as usize].adj[kk] == ti { adj_kk = kk as i32; break; } }
                if adj_kk >= 0 {
                    self.triangles[adj_ti as usize].adj[adj_kk as usize] = -1;
                    let adj_a = self.triangles[adj_ti as usize].v[(adj_kk as usize+1)%3];
                    let adj_b = self.triangles[adj_ti as usize].v[(adj_kk as usize+2)%3];
                    self.edge_map.insert(Self::edge_key(adj_a, adj_b), (adj_ti, adj_kk));
                }
            } else if let Some(&(eti, _)) = self.edge_map.get(&key) {
                if eti == ti { self.edge_map.remove(&key); }
            }
        }
    }

    fn locate(&self, x: f64, y: f64, mut start: i32) -> i32 {
        if start < 0 || start >= self.triangles.len() as i32 || !self.triangles[start as usize].alive {
            start = self.triangles.len() as i32 - 1;
            while start >= 0 && !self.triangles[start as usize].alive { start -= 1; }
            if start < 0 { return -1; }
        }
        let mut cur = start;
        let max_iter = self.triangles.len() as i32;
        for _ in 0..max_iter {
            let t = &self.triangles[cur as usize];
            let mut moved = false;
            for k in 0..3 {
                let a = t.v[k]; let b = t.v[(k+1)%3];
                let ax=self.vertices[a as usize].x; let ay=self.vertices[a as usize].y;
                let bx=self.vertices[b as usize].x; let by=self.vertices[b as usize].y;
                if Self::orient2d(ax,ay,bx,by,x,y) < 0.0 {
                    let opp = t.adj[(k+2)%3];
                    if opp >= 0 && (opp as usize) < self.triangles.len() && self.triangles[opp as usize].alive {
                        cur = opp; moved = true; break;
                    }
                }
            }
            if !moved { return cur; }
        }
        cur
    }

    pub fn insert(&mut self, x: f64, y: f64) -> i32 {
        let start = self.locate(x, y, self.last_found);
        if start >= 0 {
            let t = &self.triangles[start as usize];
            for k in 0..3 {
                let vi2 = t.v[k];
                let ddx = self.vertices[vi2 as usize].x - x;
                let ddy = self.vertices[vi2 as usize].y - y;
                if ddx*ddx + ddy*ddy < 1e-12 { return vi2; }
            }
        }
        let vi = self.vertices.len() as i32;
        self.vertices.push(Vertex2D { x, y });
        // BFS to find bad triangles
        let mut bad: Vec<i32> = Vec::new();
        let mut visited: std::collections::HashSet<i32> = std::collections::HashSet::new();
        if start >= 0 { bad.push(start); visited.insert(start); }
        let mut bfs_front = 0;
        while bfs_front < bad.len() {
            let ti = bad[bfs_front]; bfs_front += 1;
            if !self.triangles[ti as usize].alive { continue; }
            let [v0,v1,v2] = self.triangles[ti as usize].v;
            let ax=self.vertices[v0 as usize].x; let ay=self.vertices[v0 as usize].y;
            let bx=self.vertices[v1 as usize].x; let by=self.vertices[v1 as usize].y;
            let cx=self.vertices[v2 as usize].x; let cy=self.vertices[v2 as usize].y;
            let o = Self::orient2d(ax,ay,bx,by,cx,cy);
            let ic = if o > 0.0 { Self::in_circumcircle(ax,ay,bx,by,cx,cy,x,y) }
                     else        { Self::in_circumcircle(ax,ay,cx,cy,bx,by,x,y) };
            if ic > 0.0 {
                for k in 0..3 {
                    if self.triangles[ti as usize].constrained[k] { continue; }
                    let nb = self.triangles[ti as usize].adj[k];
                    if nb >= 0 && !visited.contains(&nb) { visited.insert(nb); bad.push(nb); }
                }
            } else {
                // mark as not-bad: remove from bad list
                // handled by only using `bad` entries that stay valid
            }
        }
        // Filter: keep only truly bad triangles
        let bad: Vec<i32> = bad.into_iter().filter(|&ti| {
            if !self.triangles[ti as usize].alive { return false; }
            let [v0,v1,v2] = self.triangles[ti as usize].v;
            let ax=self.vertices[v0 as usize].x; let ay=self.vertices[v0 as usize].y;
            let bx=self.vertices[v1 as usize].x; let by=self.vertices[v1 as usize].y;
            let cx=self.vertices[v2 as usize].x; let cy=self.vertices[v2 as usize].y;
            let o = Self::orient2d(ax,ay,bx,by,cx,cy);
            let ic = if o > 0.0 { Self::in_circumcircle(ax,ay,bx,by,cx,cy,x,y) }
                     else        { Self::in_circumcircle(ax,ay,cx,cy,bx,by,x,y) };
            ic > 0.0
        }).collect();
        if bad.is_empty() { self.vertices.pop(); return -1; }
        // Collect cavity boundary edges
        let bad_set: std::collections::HashSet<i32> = bad.iter().copied().collect();
        let mut polygon: Vec<(i32, i32, bool)> = Vec::new();
        for &ti in &bad {
            let t = &self.triangles[ti as usize];
            for k in 0..3 {
                let nb = t.adj[k];
                if nb < 0 || !bad_set.contains(&nb) {
                    polygon.push((t.v[(k+1)%3], t.v[(k+2)%3], t.constrained[k]));
                }
            }
        }
        for &ti in &bad { self.unregister_edges(ti); self.triangles[ti as usize].alive = false; }
        for (e0, e1, constr) in polygon {
            let o = Self::orient2d(self.vertices[vi as usize].x, self.vertices[vi as usize].y,
                                   self.vertices[e0 as usize].x, self.vertices[e0 as usize].y,
                                   self.vertices[e1 as usize].x, self.vertices[e1 as usize].y);
            if o.abs() < 1e-20 { continue; }
            let new_ti = self.triangles.len() as i32;
            let (va, vb) = if o > 0.0 { (e0, e1) } else { (e1, e0) };
            self.triangles.push(Triangle { v: [vi, va, vb], adj: [-1,-1,-1], constrained: [constr, false, false], alive: true });
            self.register_edges(new_ti);
        }
        self.last_found = self.triangles.len() as i32 - 1;
        vi
    }

    pub fn insert_constraint(&mut self, v0: i32, v1: i32) {
        if v0 == v1 { return; }
        // Check if edge already exists directly
        for ti in 0..self.triangles.len() {
            if !self.triangles[ti].alive { continue; }
            for k in 0..3 {
                let e0 = self.triangles[ti].v[(k+1)%3];
                let e1 = self.triangles[ti].v[(k+2)%3];
                if (e0==v0&&e1==v1)||(e0==v1&&e1==v0) {
                    self.triangles[ti].constrained[k] = true;
                    let nb = self.triangles[ti].adj[k];
                    if nb >= 0 && (nb as usize) < self.triangles.len() && self.triangles[nb as usize].alive {
                        for kk in 0..3 {
                            if self.triangles[nb as usize].adj[kk] == ti as i32 {
                                self.triangles[nb as usize].constrained[kk] = true; break;
                            }
                        }
                    }
                    return;
                }
            }
        }
        // Find start triangle containing v0
        let mut start_ti = -1i32;
        for i in (0..self.triangles.len()).rev() {
            if !self.triangles[i].alive { continue; }
            for k in 0..3 { if self.triangles[i].v[k] == v0 { start_ti = i as i32; break; } }
        }
        if start_ti < 0 { return; }
        let ax = self.vertices[v0 as usize].x; let ay = self.vertices[v0 as usize].y;
        let bx = self.vertices[v1 as usize].x; let by = self.vertices[v1 as usize].y;
        let mut ivl = -1i32; let mut ivr = -1i32; let mut it = -1i32;
        {
            let mut ti = start_ti;
            let guard = self.triangles.len() as i32 + 4;
            let mut g = 0;
            loop {
                if g > guard || !self.triangles[ti as usize].alive { break; }
                g += 1;
                let mut k_v0 = -1i32;
                for i in 0..3 { if self.triangles[ti as usize].v[i] == v0 { k_v0 = i as i32; break; } }
                if k_v0 < 0 { break; }
                let k = k_v0 as usize;
                let ip2 = self.triangles[ti as usize].v[(k+1)%3];
                let ip1 = self.triangles[ti as usize].v[(k+2)%3];
                let op2 = Self::orient2d(ax,ay,bx,by,self.vertices[ip2 as usize].x,self.vertices[ip2 as usize].y);
                let op1 = Self::orient2d(ax,ay,bx,by,self.vertices[ip1 as usize].x,self.vertices[ip1 as usize].y);
                if op2 < 0.0 && op1 >= 0.0 { ivl = ip1; ivr = ip2; it = ti; break; }
                let next = self.triangles[ti as usize].adj[(k+1)%3];
                if next < 0 || !((next as usize) < self.triangles.len()) || !self.triangles[next as usize].alive { break; }
                if next == start_ti { break; }
                ti = next;
            }
        }
        if it < 0 { return; }
        let mut poly_l: Vec<i32> = vec![v0, ivl];
        let mut poly_r: Vec<i32> = vec![v0, ivr];
        let mut intersected: Vec<i32> = vec![it];
        let mut iv = v0;
        let mut cur_it = it;
        let guard = self.triangles.len() as i32 * 2 + 8;
        let mut g = 0;
        let tri_has = |ti: i32, v: i32| -> bool {
            self.triangles[ti as usize].v[0]==v || self.triangles[ti as usize].v[1]==v || self.triangles[ti as usize].v[2]==v
        };
        while !tri_has(cur_it, v1) && g < guard {
            g += 1;
            let mut k_iv = -1i32;
            for i in 0..3 { if self.triangles[cur_it as usize].v[i] == iv { k_iv = i as i32; break; } }
            if k_iv < 0 { break; }
            let i_topo = self.triangles[cur_it as usize].adj[k_iv as usize];
            if i_topo < 0 || !self.triangles[i_topo as usize].alive { break; }
            let mut i_vopo = -1i32;
            for k in 0..3 { if self.triangles[i_topo as usize].adj[k] == cur_it { i_vopo = self.triangles[i_topo as usize].v[k]; break; } }
            if i_vopo < 0 { break; }
            let o = Self::orient2d(ax,ay,bx,by,self.vertices[i_vopo as usize].x,self.vertices[i_vopo as usize].y);
            if o < 0.0 {
                if i_vopo != v1 { poly_r.push(i_vopo); }
                iv = ivr; ivr = i_vopo;
            } else {
                if i_vopo != v1 { poly_l.push(i_vopo); }
                iv = ivl; ivl = i_vopo;
            }
            intersected.push(i_topo); cur_it = i_topo;
        }
        poly_l.push(v1); poly_r.push(v1);
        let first_new = self.triangles.len() as i32;
        for &ti in &intersected { self.unregister_edges(ti); self.triangles[ti as usize].alive = false; }
        let mut new_tris: Vec<(i32,i32,i32)> = Vec::new();
        { let apex = v1; for i in 0..poly_l.len().saturating_sub(2) { new_tris.push((apex, poly_l[i+1], poly_l[i])); } }
        { let apex = v0; for i in 1..poly_r.len().saturating_sub(1) { new_tris.push((apex, poly_r[i], poly_r[i+1])); } }
        for (pa, pb, pc) in new_tris {
            let o = Self::orient2d(self.vertices[pa as usize].x,self.vertices[pa as usize].y,
                                   self.vertices[pb as usize].x,self.vertices[pb as usize].y,
                                   self.vertices[pc as usize].x,self.vertices[pc as usize].y);
            if o.abs() < 1e-20 { continue; }
            let new_ti = self.triangles.len() as i32;
            let (vb, vc) = if o > 0.0 { (pb, pc) } else { (pc, pb) };
            self.triangles.push(Triangle { v: [pa, vb, vc], adj: [-1,-1,-1], constrained: [false;3], alive: true });
            self.register_edges(new_ti);
        }
        for new_ti in (first_new as usize)..self.triangles.len() {
            if !self.triangles[new_ti].alive { continue; }
            for k in 0..3 {
                let nb = self.triangles[new_ti].adj[k];
                if nb < 0 || nb >= first_new || !self.triangles[nb as usize].alive { continue; }
                for kk in 0..3 {
                    if self.triangles[nb as usize].adj[kk] == new_ti as i32 && self.triangles[nb as usize].constrained[kk] {
                        self.triangles[new_ti].constrained[k] = true; break;
                    }
                }
            }
        }
        // Mark constrained flag on the new shared edge
        for ti in 0..self.triangles.len() {
            if !self.triangles[ti].alive { continue; }
            for k in 0..3 {
                let e0 = self.triangles[ti].v[(k+1)%3];
                let e1 = self.triangles[ti].v[(k+2)%3];
                if (e0==v0&&e1==v1)||(e0==v1&&e1==v0) { self.triangles[ti].constrained[k] = true; }
            }
        }
    }

    pub fn cleanup(&mut self) {
        let sv = self.super_v;
        for ti in 0..self.triangles.len() {
            if !self.triangles[ti].alive { continue; }
            for k in 0..3 {
                if self.triangles[ti].v[k] == sv[0] || self.triangles[ti].v[k] == sv[1] || self.triangles[ti].v[k] == sv[2] {
                    self.unregister_edges(ti as i32);
                    self.triangles[ti].alive = false;
                    break;
                }
            }
        }
        self.last_found = 0;
        for i in 0..self.triangles.len() { if self.triangles[i].alive { self.last_found = i as i32; break; } }
    }

    pub fn get_triangles(&self) -> Vec<[i32; 3]> {
        let mut result = Vec::new();
        for t in &self.triangles {
            if !t.alive { continue; }
            let [a, b, c] = [t.v[0], t.v[1], t.v[2]];
            let o = Self::orient2d(self.vertices[a as usize].x, self.vertices[a as usize].y,
                                   self.vertices[b as usize].x, self.vertices[b as usize].y,
                                   self.vertices[c as usize].x, self.vertices[c as usize].y);
            result.push(if o > 0.0 { [a, b, c] } else { [a, c, b] });
        }
        result
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(tag = "type", rename = "NurbsSurfaceTrimmed")]
pub struct NurbsSurfaceTrimmed {
    #[serde(serialize_with = "crate::guid_serde::serialize", deserialize_with = "crate::guid_serde::deserialize")]
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub width: f64,
    pub surfacecolor: Color,
    pub xform: Xform,
    #[serde(rename = "surface")]
    pub m_surface: NurbsSurface,
    #[serde(rename = "outer_loop")]
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(default)]
    pub m_outer_loop: Option<NurbsCurve>,
    #[serde(rename = "inner_loops")]
    #[serde(default)]
    pub m_inner_loops: Vec<NurbsCurve>,
    // Optional plane-cut definition: when set, the trim region is { (S-q0).n <= 0 } and the
    // mesh comes from mesh_by_plane (boundary on the plane, seams welded) instead of the UV loop.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cut_q0: Option<Point>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cut_n: Option<Vector>,
    // Multi-plane SPLIT region: keep the part inside ALL half-spaces { (S-q).n <= 0 }. When
    // non-empty this takes priority over the single cut_q0/cut_n and mesh_render uses
    // mesh_by_planes. split_by_planes() fills one trimmed surface per sign-combination region.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub cut_planes: Vec<(Point, Vector)>,
}


impl NurbsSurfaceTrimmed {
    pub fn new() -> Self {
        NurbsSurfaceTrimmed {
            guid: std::sync::OnceLock::new(),
            name: "my_nurbssurface_trimmed".to_string(),
            width: 1.0,
            surfacecolor: Color::black(),
            xform: Xform::identity(),
            m_surface: NurbsSurface::new(),
            m_outer_loop: None,
            m_inner_loops: Vec::new(),
            cut_q0: None,
            cut_n: None,
            cut_planes: Vec::new(),
        }
    }

    /// Tessellate for display: plane-cut trims use mesh_by_plane (accurate boundary + welded
    /// seams); loop trims use the unified CDT mesher. Single entry point for the viewer.
    pub fn mesh_render(&self, max_angle_deg: f64, chord_factor: f64) -> Mesh {
        if !self.cut_planes.is_empty() {
            self.mesh_by_planes(&self.cut_planes, max_angle_deg, chord_factor)
        } else if let (Some(q0), Some(n)) = (self.cut_q0.as_ref(), self.cut_n.as_ref()) {
            self.mesh_by_plane(q0, n, max_angle_deg, chord_factor)
        } else {
            self.mesh_q(max_angle_deg, chord_factor)
        }
    }

    /// Apply a transform matrix to the surface (and the stored cut plane), matrix baked into
    /// geometry. Mirrors NurbsSurface::transform(&xf) so the viewer can move it like a BRep.
    pub fn transform(&mut self, xf: &Xform) {
        self.m_surface.transform(xf);
        if let Some(q0) = self.cut_q0.as_mut() { q0.xform = xf.clone(); q0.transform(); q0.xform = Xform::identity(); }
        if let Some(n) = self.cut_n.as_mut() { n.xform = xf.clone(); n.transform(); n.xform = Xform::identity(); }
        for (q, n) in self.cut_planes.iter_mut() {
            q.xform = xf.clone(); q.transform(); q.xform = Xform::identity();
            n.xform = xf.clone(); n.transform(); n.xform = Xform::identity();
        }
    }

    pub fn create(surface: &NurbsSurface, outer_loop: &NurbsCurve) -> Self {
        let mut ts = Self::new();
        ts.m_surface = surface.duplicate();
        ts.m_outer_loop = Some(outer_loop.duplicate());
        ts
    }

    pub fn create_planar(boundary: &NurbsCurve) -> Option<Self> {
        let srf = Primitives::create_planar(boundary);
        if !srf.is_valid() { return None; }

        let dom_u = srf.domain(0)?;
        let dom_v = srf.domain(1)?;
        let p00 = srf.point_at(dom_u.0, dom_v.0)?;
        let p10 = srf.point_at(dom_u.1, dom_v.0)?;
        let p01 = srf.point_at(dom_u.0, dom_v.1)?;
        let ux = p10[0]-p00[0]; let uy = p10[1]-p00[1]; let uz = p10[2]-p00[2];
        let vx = p01[0]-p00[0]; let vy = p01[1]-p00[1]; let vz = p01[2]-p00[2];
        let u_len2 = ux*ux + uy*uy + uz*uz;
        let v_len2 = vx*vx + vy*vy + vz*vz;
        if u_len2 < 1e-28 || v_len2 < 1e-28 { return None; }

        let mut uv_pts: Vec<Point> = Vec::new();
        if boundary.degree() <= 1 {
            for i in 0..boundary.cv_count() {
                if let Some(cv) = boundary.get_cv(i) {
                    let dx = cv[0]-p00[0]; let dy = cv[1]-p00[1]; let dz = cv[2]-p00[2];
                    uv_pts.push(Point::new((dx*ux+dy*uy+dz*uz)/u_len2, (dx*vx+dy*vy+dz*vz)/v_len2, 0.0));
                }
            }
        } else {
            let spans = boundary.get_span_vector();
            for si in 0..spans.len().saturating_sub(1) {
                for k in 0..=10i32 {
                    let t = spans[si] + (spans[si+1]-spans[si]) * k as f64 / 10.0;
                    let pt = boundary.point_at(t);
                    let dx = pt[0]-p00[0]; let dy = pt[1]-p00[1]; let dz = pt[2]-p00[2];
                    let nu = (dx*ux+dy*uy+dz*uz)/u_len2;
                    let nv = (dx*vx+dy*vy+dz*vz)/v_len2;
                    let ok = uv_pts.is_empty() || {
                        let last = uv_pts.last().unwrap();
                        (nu-last[0]).powi(2) + (nv-last[1]).powi(2) > 1e-24
                    };
                    if ok { uv_pts.push(Point::new(nu, nv, 0.0)); }
                }
            }
        }

        let mut ts = Self::new();
        ts.m_surface = srf;
        if uv_pts.len() >= 3 {
            ts.m_outer_loop = Some(NurbsCurve::create(false, 1, &uv_pts));
        }
        Some(ts)
    }

    /// Split a surface into trimmed faces by UV pcurves.
    ///
    /// Builds a planar arrangement of the UV domain rectangle and the given
    /// pcurves (NurbsCurves with x=u, y=v, z=0), extracts faces, and emits one
    /// NurbsSurfaceTrimmed per face. Loops are exact trims of the input
    /// pcurves joined with straight border segments. Dangling open cutters
    /// that do not reach the border or another cutter are discarded.
    pub fn split_by_uv_curves(srf: &NurbsSurface, pcurves: &[NurbsCurve], tolerance: Option<f64>) -> Vec<NurbsSurfaceTrimmed> {
        Self::split_by_uv_curves_ex(srf, pcurves, tolerance, true, 0)
    }

    /// Split a surface by UV pcurves. When `use_domain_border` is false the four
    /// domain border sides are not added; the caller supplies closed boundary
    /// loops as the first `n_boundary` pcurves (used by BRep splitting).
    pub fn split_by_uv_curves_ex(srf: &NurbsSurface, pcurves: &[NurbsCurve], tolerance: Option<f64>, use_domain_border: bool, n_boundary: usize) -> Vec<NurbsSurfaceTrimmed> {
        use std::collections::HashMap;
        use std::collections::HashSet;

        let is_boundary = |cidx: i32| -> bool {
            cidx < 0 || (!use_domain_border && cidx >= 0 && (cidx as usize) < n_boundary)
        };

        if !srf.is_valid() {
            return Vec::new();
        }

        let (u0, u1) = srf.domain(0).unwrap_or((0.0, 1.0));
        let (v0, v1) = srf.domain(1).unwrap_or((0.0, 1.0));
        let range_u = u1 - u0;
        let range_v = v1 - v0;

        let spans_u = srf.get_span_vector(0);
        let spans_v = srf.get_span_vector(1);
        let nu = spans_u.len().saturating_sub(1).max(1) * 4;
        let nv = spans_v.len().saturating_sub(1).max(1) * 4;
        let du = range_u / nu as f64;
        let dv = range_v / nv as f64;
        let mu = (u0 + u1) * 0.5;
        let mv = (v0 + v1) * 0.5;
        let pmid = srf.point_at(mu, mv).unwrap_or_else(|| Point::new(0.0, 0.0, 0.0));
        let pdu = srf.point_at((mu + du).min(u1), mv).unwrap_or_else(|| Point::new(0.0, 0.0, 0.0));
        let pdv = srf.point_at(mu, (mv + dv).min(v1)).unwrap_or_else(|| Point::new(0.0, 0.0, 0.0));
        let uv_to_3d_u = pmid.distance(&pdu, None) / du;
        let uv_to_3d_v = pmid.distance(&pdv, None) / dv;
        let mut uv_to_3d = uv_to_3d_u.max(uv_to_3d_v);
        if uv_to_3d < 1e-10 {
            uv_to_3d = 1.0;
        }

        let snap_uv = match tolerance {
            Some(tol) if tol > 0.0 => (tol / uv_to_3d).max(1e-9),
            _ => range_u.min(range_v) * 1e-7,
        };

        // ---- 1. Sample cutters into tagged UV polylines ----
        let samp_tol = range_u.max(range_v) * 1e-3;
        struct PolyLine { cidx: i32, pts: Vec<[f64; 2]>, ts: Vec<f64> }
        let mut polylines: Vec<PolyLine> = Vec::new();

        let snap_border = |p: &mut [f64; 2]| {
            if (p[0] - u0).abs() < snap_uv {
                p[0] = u0;
            }
            if (p[0] - u1).abs() < snap_uv {
                p[0] = u1;
            }
            if (p[1] - v0).abs() < snap_uv {
                p[1] = v0;
            }
            if (p[1] - v1).abs() < snap_uv {
                p[1] = v1;
            }
        };

        for (cidx, crv) in pcurves.iter().enumerate() {
            if !crv.is_valid() {
                continue;
            }
            let (ct0, ct1) = crv.domain();
            let mut entries: Vec<[f64; 3]> = Vec::new();
            let n = (crv.cv_count() * 4).max(16);
            for i in 0..=n {
                let t = ct0 + (ct1 - ct0) * i as f64 / n as f64;
                let p = crv.point_at(t);
                entries.push([t, p[0], p[1]]);
            }
            let mut depth = 0;
            while depth < 6 {
                let mut inserted = 0;
                let mut i = 0;
                while i < entries.len() - 1 {
                    let a = entries[i];
                    let b = entries[i + 1];
                    let tm = (a[0] + b[0]) * 0.5;
                    let pm = crv.point_at(tm);
                    let exu = b[1] - a[1];
                    let exv = b[2] - a[2];
                    let l2 = exu*exu + exv*exv;
                    let dev = if l2 > 1e-30 {
                        let s = ((pm[0]-a[1])*exu + (pm[1]-a[2])*exv) / l2;
                        let cx = a[1] + s*exu;
                        let cy = a[2] + s*exv;
                        (pm[0]-cx).hypot(pm[1]-cy)
                    } else {
                        0.0
                    };
                    if dev > samp_tol && entries.len() < 4096 {
                        entries.insert(i + 1, [tm, pm[0], pm[1]]);
                        inserted += 1;
                        i += 2;
                    } else {
                        i += 1;
                    }
                }
                if inserted == 0 {
                    break;
                }
                depth += 1;
            }
            let mut pts: Vec<[f64; 2]> = Vec::new();
            let mut ts: Vec<f64> = Vec::new();
            for e in &entries {
                let mut p = [e[1].max(u0).min(u1), e[2].max(v0).min(v1)];
                snap_border(&mut p);
                if let Some(last) = pts.last() {
                    if (p[0]-last[0]).abs() < 1e-15 && (p[1]-last[1]).abs() < 1e-15 {
                        continue;
                    }
                }
                pts.push(p);
                ts.push(e[0]);
            }
            if pts.len() < 2 {
                continue;
            }
            // A cutter lying entirely on one border line coincides with the
            // domain edge (e.g. a cut circle on the seam) and splits nothing.
            // When the caller supplies its own boundary loops (no domain
            // border), those loops legitimately run along the domain edges.
            if use_domain_border && (
               pts.iter().all(|p| (p[0] - u0).abs() < snap_uv) ||
               pts.iter().all(|p| (p[0] - u1).abs() < snap_uv) ||
               pts.iter().all(|p| (p[1] - v0).abs() < snap_uv) ||
               pts.iter().all(|p| (p[1] - v1).abs() < snap_uv)) {
                continue;
            }
            polylines.push(PolyLine { cidx: cidx as i32, pts, ts });
        }

        // Border sides as polylines: cidx -1 bottom, -2 right, -3 top, -4 left
        if use_domain_border {
            polylines.push(PolyLine { cidx: -1, pts: vec![[u0, v0], [u1, v0]], ts: vec![u0, u1] });
            polylines.push(PolyLine { cidx: -2, pts: vec![[u1, v0], [u1, v1]], ts: vec![v0, v1] });
            polylines.push(PolyLine { cidx: -3, pts: vec![[u1, v1], [u0, v1]], ts: vec![u1, u0] });
            polylines.push(PolyLine { cidx: -4, pts: vec![[u0, v1], [u0, v0]], ts: vec![v1, v0] });
        }

        // ---- 2. Segment-segment intersections (Newton-refined on real curves) ----
        fn seg_seg(p1: &[f64; 2], p2: &[f64; 2], p3: &[f64; 2], p4: &[f64; 2]) -> Option<(f64, f64)> {
            let d1u = p2[0] - p1[0];
            let d1v = p2[1] - p1[1];
            let d2u = p4[0] - p3[0];
            let d2v = p4[1] - p3[1];
            let den = d1u * d2v - d1v * d2u;
            if den.abs() < 1e-20 {
                return None;
            }
            let s = ((p3[0]-p1[0]) * d2v - (p3[1]-p1[1]) * d2u) / den;
            let t = ((p3[0]-p1[0]) * d1v - (p3[1]-p1[1]) * d1u) / den;
            if -1e-12 <= s && s <= 1.0 + 1e-12 && -1e-12 <= t && t <= 1.0 + 1e-12 {
                return Some((s, t));
            }
            None
        }

        let newton_cc = |ca: &NurbsCurve, mut ta: f64, cb: &NurbsCurve, mut tb: f64| -> (f64, f64) {
            for _ in 0..8 {
                let da = ca.evaluate(ta, 1);
                let db = cb.evaluate(tb, 1);
                let fu = da[0][0] - db[0][0];
                let fv = da[0][1] - db[0][1];
                if fu.hypot(fv) < snap_uv * 0.01 {
                    break;
                }
                let j00 = da[1][0];
                let j01 = -db[1][0];
                let j10 = da[1][1];
                let j11 = -db[1][1];
                let den = j00 * j11 - j01 * j10;
                if den.abs() < 1e-20 {
                    break;
                }
                ta -= (fu * j11 - j01 * fv) / den;
                tb -= (j00 * fv - fu * j10) / den;
                let (a0, a1) = ca.domain();
                let (b0, b1) = cb.domain();
                ta = ta.max(a0).min(a1);
                tb = tb.max(b0).min(b1);
            }
            (ta, tb)
        };

        let mut splits: HashMap<(usize, usize), Vec<(f64, f64, f64, f64)>> = HashMap::new();
        for pi in 0..polylines.len() {
            for pj in (pi + 1)..polylines.len() {
                let a_poly = &polylines[pi];
                let b_poly = &polylines[pj];
                if is_boundary(a_poly.cidx) && is_boundary(b_poly.cidx) {
                    continue;
                }
                let aminu = a_poly.pts.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min) - snap_uv;
                let amaxu = a_poly.pts.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max) + snap_uv;
                let aminv = a_poly.pts.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min) - snap_uv;
                let amaxv = a_poly.pts.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max) + snap_uv;
                let bminu = b_poly.pts.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
                let bmaxu = b_poly.pts.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
                let bminv = b_poly.pts.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
                let bmaxv = b_poly.pts.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max);
                if bminu > amaxu || bmaxu < aminu || bminv > amaxv || bmaxv < aminv {
                    continue;
                }
                for ia in 0..a_poly.pts.len() - 1 {
                    for ib in 0..b_poly.pts.len() - 1 {
                        let hit = seg_seg(&a_poly.pts[ia], &a_poly.pts[ia+1], &b_poly.pts[ib], &b_poly.pts[ib+1]);
                        let (s, t) = match hit {
                            Some(h) => h,
                            None => continue,
                        };
                        let mut ta = a_poly.ts[ia] + (a_poly.ts[ia+1] - a_poly.ts[ia]) * s;
                        let mut tb = b_poly.ts[ib] + (b_poly.ts[ib+1] - b_poly.ts[ib]) * t;
                        let mut hu = a_poly.pts[ia][0] + (a_poly.pts[ia+1][0] - a_poly.pts[ia][0]) * s;
                        let mut hv = a_poly.pts[ia][1] + (a_poly.pts[ia+1][1] - a_poly.pts[ia][1]) * s;
                        if a_poly.cidx >= 0 && b_poly.cidx >= 0 {
                            let (nta, ntb) = newton_cc(&pcurves[a_poly.cidx as usize], ta, &pcurves[b_poly.cidx as usize], tb);
                            ta = nta;
                            tb = ntb;
                            let pa = pcurves[a_poly.cidx as usize].point_at(ta);
                            hu = pa[0];
                            hv = pa[1];
                        } else if a_poly.cidx >= 0 {
                            let pa = pcurves[a_poly.cidx as usize].point_at(ta);
                            hu = pa[0];
                            hv = pa[1];
                        } else if b_poly.cidx >= 0 {
                            let pb = pcurves[b_poly.cidx as usize].point_at(tb);
                            hu = pb[0];
                            hv = pb[1];
                        }
                        let mut hp = [hu, hv];
                        snap_border(&mut hp);
                        if b_poly.cidx < 0 {
                            if b_poly.cidx == -1 || b_poly.cidx == -3 {
                                tb = hp[0];
                            } else {
                                tb = hp[1];
                            }
                        }
                        if a_poly.cidx < 0 {
                            if a_poly.cidx == -1 || a_poly.cidx == -3 {
                                ta = hp[0];
                            } else {
                                ta = hp[1];
                            }
                        }
                        splits.entry((pi, ia)).or_insert_with(Vec::new).push((s, hp[0], hp[1], ta));
                        splits.entry((pj, ib)).or_insert_with(Vec::new).push((t, hp[0], hp[1], tb));
                    }
                }
            }
        }

        // ---- 3. Rebuild polylines with split vertices; build the vertex pool ----
        let mut cell_map: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
        let mut verts: Vec<[f64; 2]> = Vec::new();

        fn vert_id(p: [f64; 2], snap_uv: f64, verts: &mut Vec<[f64; 2]>, cell_map: &mut std::collections::HashMap<(i64, i64), Vec<usize>>) -> usize {
            let ci = (p[0] / snap_uv).floor() as i64;
            let cj = (p[1] / snap_uv).floor() as i64;
            for di in -1i64..=1 {
                for dj in -1i64..=1 {
                    if let Some(bucket) = cell_map.get(&(ci+di, cj+dj)) {
                        for &vk in bucket {
                            let q = verts[vk];
                            if (q[0]-p[0]).hypot(q[1]-p[1]) <= snap_uv {
                                return vk;
                            }
                        }
                    }
                }
            }
            let vk = verts.len();
            verts.push([p[0], p[1]]);
            cell_map.entry((ci, cj)).or_insert_with(Vec::new).push(vk);
            vk
        }

        struct EdgeRec { a: usize, b: usize, cidx: i32, ta: f64, tb: f64 }
        let mut edges: Vec<EdgeRec> = Vec::new();

        for (pi, poly) in polylines.iter().enumerate() {
            let mut chain: Vec<(usize, f64)> = Vec::new();
            for i in 0..poly.pts.len() {
                chain.push((vert_id(poly.pts[i], snap_uv, &mut verts, &mut cell_map), poly.ts[i]));
                if i < poly.pts.len() - 1 {
                    if let Some(sp) = splits.get(&(pi, i)) {
                        let mut evs = sp.clone();
                        evs.sort_by(|x, y| x.partial_cmp(y).unwrap());
                        for (_frac, hu, hv, tc) in evs {
                            chain.push((vert_id([hu, hv], snap_uv, &mut verts, &mut cell_map), tc));
                        }
                    }
                }
            }
            for i in 0..chain.len() - 1 {
                let (a, ta) = chain[i];
                let (b, tb) = chain[i + 1];
                if a == b {
                    continue;
                }
                edges.push(EdgeRec { a, b, cidx: poly.cidx, ta, tb });
            }
        }

        // ---- 4. Prune dangling edges (valence-1 chains) ----
        let mut alive = vec![true; edges.len()];
        let mut changed = true;
        while changed {
            changed = false;
            let mut degree: HashMap<usize, usize> = HashMap::new();
            for (ei, e) in edges.iter().enumerate() {
                if !alive[ei] {
                    continue;
                }
                *degree.entry(e.a).or_insert(0) += 1;
                *degree.entry(e.b).or_insert(0) += 1;
            }
            for (ei, e) in edges.iter().enumerate() {
                if !alive[ei] {
                    continue;
                }
                if degree.get(&e.a).copied().unwrap_or(0) == 1 || degree.get(&e.b).copied().unwrap_or(0) == 1 {
                    alive[ei] = false;
                    changed = true;
                }
            }
        }

        let live_edges: Vec<&EdgeRec> = edges.iter().enumerate().filter(|(ei, _)| alive[*ei]).map(|(_, e)| e).collect();
        if live_edges.is_empty() {
            return Vec::new();
        }

        // ---- 5. Half-edge face extraction (leftmost-turn walk) ----
        let mut out_map: HashMap<usize, Vec<usize>> = HashMap::new();
        let mut hes: Vec<(usize, usize, usize, bool)> = Vec::new(); // (tail, head, live_edge_index, fwd)
        for (li, e) in live_edges.iter().enumerate() {
            hes.push((e.a, e.b, li, true));
            hes.push((e.b, e.a, li, false));
        }
        for (hi, he) in hes.iter().enumerate() {
            out_map.entry(he.0).or_insert_with(Vec::new).push(hi);
        }
        for (&vid, outs) in out_map.iter_mut() {
            outs.sort_by(|&h1, &h2| {
                let a1 = (verts[hes[h1].1][1] - verts[vid][1]).atan2(verts[hes[h1].1][0] - verts[vid][0]);
                let a2 = (verts[hes[h2].1][1] - verts[vid][1]).atan2(verts[hes[h2].1][0] - verts[vid][0]);
                a1.partial_cmp(&a2).unwrap()
            });
        }

        let mut next_he = vec![usize::MAX; hes.len()];
        for (_vid, outs) in out_map.iter() {
            for (pos, &hi) in outs.iter().enumerate() {
                let tw = hi ^ 1;
                // at vertex vid, incoming tw arrives; next outgoing is the one
                // clockwise from the reversed incoming (leftmost turn)
                let nxt = outs[(pos + outs.len() - 1) % outs.len()];
                next_he[tw] = nxt;
            }
        }

        let mut visited = vec![false; hes.len()];
        let mut faces: Vec<Vec<usize>> = Vec::new(); // list of list of he indices
        for hi in 0..hes.len() {
            if visited[hi] {
                continue;
            }
            let mut cycle = Vec::new();
            let mut cur = hi;
            while !visited[cur] {
                visited[cur] = true;
                cycle.push(cur);
                cur = next_he[cur];
            }
            if cycle.len() >= 2 {
                faces.push(cycle);
            }
        }

        let face_area = |cycle: &Vec<usize>| -> f64 {
            let mut s = 0.0;
            for &hi in cycle {
                let a = verts[hes[hi].0];
                let b = verts[hes[hi].1];
                s += a[0]*b[1] - b[0]*a[1];
            }
            s * 0.5
        };

        let mut border_vids: HashSet<usize> = HashSet::new();
        for e in &live_edges {
            if is_boundary(e.cidx) {
                border_vids.insert(e.a);
                border_vids.insert(e.b);
            }
        }

        let point_in_cycle = |p: [f64; 2], cycle: &Vec<usize>| -> bool {
            let mut inside = false;
            for &hi in cycle {
                let a = verts[hes[hi].0];
                let b = verts[hes[hi].1];
                if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0] {
                    inside = !inside;
                }
            }
            inside
        };

        let mut pos_faces: Vec<(Vec<usize>, f64)> = Vec::new();
        let mut neg_faces: Vec<Vec<usize>> = Vec::new();
        for cycle in faces {
            let area = face_area(&cycle);
            if area > snap_uv * snap_uv {
                pos_faces.push((cycle, area));
            } else if area < -snap_uv * snap_uv {
                let touches_border = cycle.iter().any(|&hi| border_vids.contains(&hes[hi].0));
                if !touches_border {
                    neg_faces.push(cycle);
                }
            }
        }

        // ---- 6. Assign floating hole loops to their containing faces ----
        let mut holes_of: Vec<Vec<Vec<usize>>> = vec![Vec::new(); pos_faces.len()];
        for cycle in &neg_faces {
            let sample = verts[hes[cycle[0]].0];
            let mut best: i32 = -1;
            let mut best_area = f64::INFINITY;
            for (fi, (fc, area)) in pos_faces.iter().enumerate() {
                if *area < best_area && point_in_cycle(sample, fc) {
                    // the hole vertex lies ON the cycle of its own disk face;
                    // skip faces sharing vertices with the hole cycle
                    let hole_vids: HashSet<usize> = cycle.iter().map(|&hi| hes[hi].0).collect();
                    let face_vids: HashSet<usize> = fc.iter().map(|&hi| hes[hi].0).collect();
                    if hole_vids == face_vids {
                        continue;
                    }
                    best = fi as i32;
                    best_area = *area;
                }
            }
            if best >= 0 {
                holes_of[best as usize].push(cycle.clone());
            }
        }

        // ---- 7. Emit one trimmed surface per face ----
        let cycle_to_loop = |cycle: &Vec<usize>| -> NurbsCurve {
            // Collapse consecutive same-curve half-edges into exact trims,
            // border runs into straight segments, then join into one loop
            struct Run { cidx: i32, va: usize, vb: usize, ta: f64, tb: f64 }
            let mut runs: Vec<Run> = Vec::new();
            for &hi in cycle {
                let (tail, head, li, fwd) = hes[hi];
                let e = live_edges[li];
                let ta = if fwd { e.ta } else { e.tb };
                let tb = if fwd { e.tb } else { e.ta };
                let merged = match runs.last_mut() {
                    Some(last) if last.cidx == e.cidx && last.vb == tail => {
                        last.vb = head;
                        last.tb = tb;
                        true
                    }
                    _ => false,
                };
                if !merged {
                    runs.push(Run { cidx: e.cidx, va: tail, vb: head, ta, tb });
                }
            }
            let mut pieces: Vec<NurbsCurve> = Vec::new();
            for run in &runs {
                if run.cidx >= 0 {
                    let crv = &pcurves[run.cidx as usize];
                    let (c0, c1) = crv.domain();
                    let lo = run.ta.min(run.tb);
                    let hi_ = run.ta.max(run.tb);
                    let mut piece = Some(crv.duplicate());
                    if hi_ - lo < (c1 - c0) - 1e-12 && hi_ - lo > 1e-14 {
                        if !piece.as_mut().unwrap().trim(lo, hi_) {
                            piece = None;
                        }
                    }
                    if let Some(p) = piece {
                        if p.is_valid() {
                            pieces.push(p);
                            continue;
                        }
                    }
                }
                let pa = verts[run.va];
                let pb = verts[run.vb];
                if (pb[0]-pa[0]).hypot(pb[1]-pa[1]) > 1e-14 {
                    let seg_pts = vec![
                        Point::new(pa[0], pa[1], 0.0),
                        Point::new(pb[0], pb[1], 0.0),
                    ];
                    pieces.push(NurbsCurve::create(false, 1, &seg_pts));
                }
            }
            if pieces.is_empty() {
                return NurbsCurve::default();
            }
            let mut joined = NurbsCurve::join(&pieces, Some(snap_uv * 4.0));
            if joined.len() == 1 && joined[0].is_closed() {
                return joined.remove(0);
            }
            // Fallback: polyline loop from the face walk
            let mut loop_pts: Vec<Point> = Vec::new();
            for &hi in cycle {
                let a = verts[hes[hi].0];
                loop_pts.push(Point::new(a[0], a[1], 0.0));
            }
            loop_pts.push(Point::new(loop_pts[0][0], loop_pts[0][1], 0.0));
            NurbsCurve::create(false, 1, &loop_pts)
        };

        let loop_signed_area = |loop_crv: &NurbsCurve| -> f64 {
            let n = 64;
            let (l0, l1) = loop_crv.domain();
            let mut s = 0.0;
            let mut prev = loop_crv.point_at(l0);
            for i in 1..=n {
                let p = loop_crv.point_at(l0 + (l1 - l0) * i as f64 / n as f64);
                s += prev[0]*p[1] - p[0]*prev[1];
                prev = p;
            }
            s * 0.5
        };

        let mut result: Vec<NurbsSurfaceTrimmed> = Vec::new();
        for (fi, (cycle, _area)) in pos_faces.iter().enumerate() {
            let mut outer = cycle_to_loop(cycle);
            if !outer.is_valid() {
                continue;
            }
            if loop_signed_area(&outer) < 0.0 {
                outer.reverse();
            }
            let mut ts = NurbsSurfaceTrimmed::create(srf, &outer);
            for hole_cycle in &holes_of[fi] {
                let mut hole = cycle_to_loop(hole_cycle);
                if hole.is_valid() {
                    if loop_signed_area(&hole) > 0.0 {
                        hole.reverse();
                    }
                    ts.add_inner_loop(hole);
                }
            }
            result.push(ts);
        }
        result
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    pub fn surface(&self) -> &NurbsSurface { &self.m_surface }
    pub fn get_outer_loop(&self) -> Option<&NurbsCurve> { self.m_outer_loop.as_ref() }

    pub fn set_outer_loop(&mut self, loop_crv: NurbsCurve) {
        self.m_outer_loop = Some(loop_crv);
    }

    pub fn is_trimmed(&self) -> bool {
        self.m_outer_loop.as_ref().map_or(false, |c| c.is_valid())
    }

    pub fn is_valid(&self) -> bool { self.m_surface.is_valid() }

    pub fn add_inner_loop(&mut self, loop_2d: NurbsCurve) {
        self.m_inner_loops.push(loop_2d);
    }

    pub fn add_hole(&mut self, curve_3d: &NurbsCurve) {
        let dom = curve_3d.domain();
        let sdom_u = self.m_surface.domain(0).unwrap_or((0.0, 1.0));
        let sdom_v = self.m_surface.domain(1).unwrap_or((0.0, 1.0));
        let range_u = sdom_u.1 - sdom_u.0;
        let range_v = sdom_v.1 - sdom_v.0;

        let n_samples = std::cmp::max(curve_3d.cv_count() * 4, 32);
        let mut uv_pts = Vec::new();
        for i in 0..n_samples {
            let t = dom.0 + (dom.1 - dom.0) * i as f64 / n_samples as f64;
            let pt3d = curve_3d.point_at(t);
            let (u, v, _) = Closest::surface_point(&self.m_surface, &pt3d, 0.0, 0.0, 0.0, 0.0);
            let nu = (u - sdom_u.0) / range_u;
            let nv = (v - sdom_v.0) / range_v;
            uv_pts.push(Point::new(nu, nv, 0.0));
        }
        if uv_pts.len() >= 3 {
            self.m_inner_loops.push(NurbsCurve::create(true, 1, &uv_pts));
        }
    }

    pub fn add_holes(&mut self, curves_3d: &[NurbsCurve]) {
        for crv in curves_3d {
            self.add_hole(crv);
        }
    }

    pub fn get_inner_loop(&self, index: usize) -> Option<&NurbsCurve> {
        self.m_inner_loops.get(index)
    }

    pub fn inner_loop_count(&self) -> usize { self.m_inner_loops.len() }

    pub fn clear_inner_loops(&mut self) { self.m_inner_loops.clear(); }

    pub fn point_at(&self, u: f64, v: f64) -> Option<Point> { self.m_surface.point_at(u, v) }
    pub fn normal_at(&self, u: f64, v: f64) -> Vector { self.m_surface.normal_at(u, v) }

    pub fn mesh(&self) -> Mesh { self.mesh_q(20.0, 0.005) }

    // Unified trimmed-surface tessellation (BRepMesh / OpenNURBS model):
    //   1. Discretize each UV trim loop into a polygon, adaptively refining segments whose
    //      lifted 3D midpoint deviates from the chord — so the boundary follows the surface.
    //   2. Constrained Delaunay of the UV domain with the trim loops as boundary constraints.
    //   3. Refine the interior by surface DEFLECTION: repeatedly split any triangle whose surface
    //      point at its UV centroid lies farther than the deflection tolerance from the triangle's
    //      3D plane (or whose corner normals turn more than max_angle_deg). New points go in at the
    //      centroid only when it is inside the trim region, keeping the CDT boundary-conforming and
    //      producing graded, well-shaped triangles — dense where curved, coarse where flat.
    //   4. Delete exterior triangles, lift to 3D, set per-vertex analytic normals.
    // Planar surfaces need no interior refinement (deflection is ~0), so step 3 exits immediately
    // and the same code path yields the minimal boundary triangulation.
    pub fn mesh_q(&self, max_angle_deg: f64, chord_factor: f64) -> Mesh {
        if !self.is_trimmed() { return self.m_surface.mesh(); }

        // 3D bbox diagonal from surface control points -> deflection tolerance.
        let mut bmin = [1e30f64; 3]; let mut bmax = [-1e30f64; 3];
        for i in 0..self.m_surface.cv_count_dir(Some(0)) {
            for j in 0..self.m_surface.cv_count_dir(Some(1)) {
                if let Some(p) = self.m_surface.get_cv(i, j) {
                    for k in 0..3 { let c = p[k] as f64; if c < bmin[k] { bmin[k]=c; } if c > bmax[k] { bmax[k]=c; } }
                }
            }
        }
        let mut bbox_diag = (0..3).map(|k| (bmax[k]-bmin[k]).powi(2)).sum::<f64>().sqrt();
        if bbox_diag < 1e-12 { bbox_diag = 1.0; }
        let deflection = bbox_diag * chord_factor;
        let cos_max_angle = (max_angle_deg.max(0.1).min(179.0) * std::f64::consts::PI / 180.0).cos();

        let eval3 = |u: f64, v: f64| -> [f64; 3] {
            let p = self.m_surface.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0));
            [p[0] as f64, p[1] as f64, p[2] as f64]
        };

        // ---- 1. Adaptive trim-wire discretization in UV ----
        let disc_loop = |crv: &NurbsCurve| -> Vec<[f64; 2]> {
            let mut raw: Vec<[f64; 2]> = if crv.degree() <= 1 && !crv.is_rational() {
                (0..crv.cv_count()).filter_map(|i| crv.get_cv(i)).map(|p| [p[0] as f64, p[1] as f64]).collect()
            } else {
                let n = (crv.cv_count() * 4).max(16);
                let (sampled, _) = crv.divide_by_count(n, true);
                sampled.iter().map(|p| [p[0] as f64, p[1] as f64]).collect()
            };
            while raw.len() > 1 {
                let dx = raw[0][0] - raw[raw.len()-1][0];
                let dy = raw[0][1] - raw[raw.len()-1][1];
                if dx*dx + dy*dy < 1e-20 { raw.pop(); } else { break; }
            }
            if raw.len() < 2 { return raw; }
            // Recursively split each segment while its lifted 3D midpoint deviates from the chord.
            // Explicit stack, pushing right-then-left so points are emitted in boundary order.
            let mut out: Vec<[f64; 2]> = Vec::with_capacity(raw.len() * 2);
            let m = raw.len();
            for i in 0..m {
                let a = raw[i]; let b = raw[(i+1)%m];
                let mut stack: Vec<([f64;2],[f64;2],i32)> = vec![(a, b, 0)];
                while let Some((sa, sb, depth)) = stack.pop() {
                    let mu = (sa[0]+sb[0])*0.5; let mv = (sa[1]+sb[1])*0.5;
                    let pa = eval3(sa[0], sa[1]); let pb = eval3(sb[0], sb[1]); let pm = eval3(mu, mv);
                    let ex = pb[0]-pa[0]; let ey = pb[1]-pa[1]; let ez = pb[2]-pa[2];
                    let l2 = ex*ex + ey*ey + ez*ez;
                    let dev = if l2 > 1e-30 {
                        let t = ((pm[0]-pa[0])*ex+(pm[1]-pa[1])*ey+(pm[2]-pa[2])*ez)/l2;
                        let cx = pa[0]+t*ex; let cy = pa[1]+t*ey; let cz = pa[2]+t*ez;
                        ((pm[0]-cx).powi(2)+(pm[1]-cy).powi(2)+(pm[2]-cz).powi(2)).sqrt()
                    } else { 0.0 };
                    if dev > deflection && depth < 6 {
                        stack.push(([mu,mv], sb, depth+1));
                        stack.push((sa, [mu,mv], depth+1));
                    } else {
                        out.push(sa);
                    }
                }
            }
            out
        };

        let outer_uv = match self.m_outer_loop.as_ref() { Some(c) => disc_loop(c), None => return self.m_surface.mesh() };
        if outer_uv.len() < 3 { return self.m_surface.mesh(); }
        let hole_uvs: Vec<Vec<[f64;2]>> = self.m_inner_loops.iter().map(|c| disc_loop(c)).collect();

        let mut bb_umin = 1e30_f64; let mut bb_vmin = 1e30_f64;
        let mut bb_umax = -1e30_f64; let mut bb_vmax = -1e30_f64;
        for p in &outer_uv {
            if p[0] < bb_umin { bb_umin = p[0]; } if p[1] < bb_vmin { bb_vmin = p[1]; }
            if p[0] > bb_umax { bb_umax = p[0]; } if p[1] > bb_vmax { bb_vmax = p[1]; }
        }

        let point_in_polygon = |u: f64, v: f64, poly: &[[f64;2]]| -> bool {
            let n = poly.len(); if n < 3 { return false; } let mut inside = false; let mut j = n - 1;
            for i in 0..n {
                let (xi, yi) = (poly[i][0], poly[i][1]);
                let (xj, yj) = (poly[j][0], poly[j][1]);
                if ((yi > v) != (yj > v)) && (u < (xj-xi)*(v-yi)/(yj-yi)+xi) { inside = !inside; }
                j = i;
            }
            inside
        };
        let inside_trim = |u: f64, v: f64| -> bool {
            if !point_in_polygon(u, v, &outer_uv) { return false; }
            for h in &hole_uvs { if point_in_polygon(u, v, h) { return false; } }
            true
        };

        // ---- 2. Constrained Delaunay of the trim wire ----
        let mut dt = crate::nurbssurface_trimmed::Delaunay2D::new(bb_umin, bb_vmin, bb_umax, bb_vmax);
        {
            let vis: Vec<i32> = outer_uv.iter().map(|p| dt.insert(p[0], p[1])).collect();
            for i in 0..vis.len() {
                let j = (i + 1) % vis.len();
                if vis[i] >= 0 && vis[j] >= 0 && vis[i] != vis[j] { dt.insert_constraint(vis[i], vis[j]); }
            }
        }
        for h in hole_uvs.iter() {
            let vis: Vec<i32> = h.iter().map(|p| dt.insert(p[0], p[1])).collect();
            for i in 0..vis.len() {
                let j = (i + 1) % vis.len();
                if vis[i] >= 0 && vis[j] >= 0 && vis[i] != vis[j] { dt.insert_constraint(vis[i], vis[j]); }
            }
        }

        // ---- 3. Interior refinement by surface deflection ----
        const MAX_ITERS: i32 = 8;
        const MAX_VERTS: usize = 200000;
        for _iter in 0..MAX_ITERS {
            let mut to_insert: Vec<[f64; 2]> = Vec::new();
            for tri in &dt.triangles {
                if !tri.alive { continue; }
                let a = &dt.vertices[tri.v[0] as usize];
                let b = &dt.vertices[tri.v[1] as usize];
                let c = &dt.vertices[tri.v[2] as usize];
                let cu = (a.x + b.x + c.x) / 3.0; let cv = (a.y + b.y + c.y) / 3.0;
                if !inside_trim(cu, cv) { continue; }
                let pa = eval3(a.x, a.y); let pb = eval3(b.x, b.y); let pc = eval3(c.x, c.y); let pm = eval3(cu, cv);
                let ux = pb[0]-pa[0]; let uy = pb[1]-pa[1]; let uz = pb[2]-pa[2];
                let vx = pc[0]-pa[0]; let vy = pc[1]-pa[1]; let vz = pc[2]-pa[2];
                let nx = uy*vz-uz*vy; let ny = uz*vx-ux*vz; let nz = ux*vy-uy*vx;
                let nl = (nx*nx+ny*ny+nz*nz).sqrt();
                if nl < 1e-30 { continue; }
                let dev = (((pm[0]-pa[0])*nx+(pm[1]-pa[1])*ny+(pm[2]-pa[2])*nz)/nl).abs();
                let mut refine = dev > deflection;
                if !refine {
                    let na = self.m_surface.normal_at(a.x, a.y);
                    let nb = self.m_surface.normal_at(b.x, b.y);
                    let nc2 = self.m_surface.normal_at(c.x, c.y);
                    let d1 = na[0]*nb[0]+na[1]*nb[1]+na[2]*nb[2];
                    let d2 = nb[0]*nc2[0]+nb[1]*nc2[1]+nb[2]*nc2[2];
                    let d3 = na[0]*nc2[0]+na[1]*nc2[1]+na[2]*nc2[2];
                    let mind = d1.min(d2.min(d3)) as f64;
                    if mind < cos_max_angle { refine = true; }
                }
                if refine { to_insert.push([cu, cv]); }
            }
            if to_insert.is_empty() { break; }
            for uv in &to_insert {
                if dt.vertices.len() >= MAX_VERTS { break; }
                dt.insert(uv[0], uv[1]);
            }
            if dt.vertices.len() >= MAX_VERTS { break; }
        }

        // ---- 4. Trim, lift, normals ----
        dt.cleanup();
        for ti in 0..dt.triangles.len() {
            if !dt.triangles[ti].alive { continue; }
            let cu = (dt.vertices[dt.triangles[ti].v[0] as usize].x
                    + dt.vertices[dt.triangles[ti].v[1] as usize].x
                    + dt.vertices[dt.triangles[ti].v[2] as usize].x) / 3.0;
            let cv = (dt.vertices[dt.triangles[ti].v[0] as usize].y
                    + dt.vertices[dt.triangles[ti].v[1] as usize].y
                    + dt.vertices[dt.triangles[ti].v[2] as usize].y) / 3.0;
            if !inside_trim(cu, cv) { dt.triangles[ti].alive = false; }
        }
        let tris = dt.get_triangles();
        if tris.is_empty() { return self.m_surface.mesh(); }
        let mut result = Mesh::new();
        let nv = dt.vertices.len();
        let mut vert_map: Vec<Option<usize>> = vec![None; nv];

        // Lift to 3D, welding coincident vertices so a closed/periodic surface (cylinder, cone,
        // torus, sphere) stitches at its seam: distinct UV columns u0 and u1 (or rows v0/v1)
        // evaluate to the SAME 3D point, so they must share one mesh vertex. Spatial hash on a
        // weld-tolerance grid; new points scan the 3x3x3 neighbour cells.
        let weld_tol = bbox_diag * 1e-5;
        let cell = if weld_tol > 0.0 { weld_tol } else { 1.0 };
        let mut cell_map: std::collections::HashMap<(i64, i64, i64), Vec<([f64; 3], usize)>> = std::collections::HashMap::new();
        for &[a, b, c] in &tris {
            for &vi in &[a, b, c] {
                if vert_map[vi as usize].is_none() {
                    let u = dt.vertices[vi as usize].x;
                    let v = dt.vertices[vi as usize].y;
                    let p3d = self.m_surface.point_at(u, v).unwrap_or(Point::new(0.0,0.0,0.0));
                    let x = p3d[0] as f64; let y = p3d[1] as f64; let z = p3d[2] as f64;
                    let ci = (x/cell).floor() as i64;
                    let cj = (y/cell).floor() as i64;
                    let ck = (z/cell).floor() as i64;
                    let mut found: Option<usize> = None;
                    'scan: for di in -1..=1 { for dj in -1..=1 { for dk in -1..=1 {
                        if let Some(bucket) = cell_map.get(&(ci+di, cj+dj, ck+dk)) {
                            for &(p, wvk) in bucket {
                                let dx = p[0]-x; let dy = p[1]-y; let dz = p[2]-z;
                                if dx*dx + dy*dy + dz*dz <= weld_tol*weld_tol { found = Some(wvk); break 'scan; }
                            }
                        }
                    }}}
                    let vk = match found {
                        Some(wvk) => wvk,
                        None => {
                            let vk = result.add_vertex(p3d, None);
                            cell_map.entry((ci, cj, ck)).or_default().push(([x, y, z], vk));
                            vk
                        }
                    };
                    vert_map[vi as usize] = Some(vk);
                }
            }
        }
        for &[a, b, c] in &tris {
            let v0 = vert_map[a as usize].unwrap();
            let v1 = vert_map[b as usize].unwrap();
            let v2 = vert_map[c as usize].unwrap();
            if v0 == v1 || v1 == v2 || v2 == v0 { continue; }
            result.add_face(vec![v0, v1, v2], None);
        }
        for vi in 0..nv {
            if let Some(vk) = vert_map[vi] {
                let u = dt.vertices[vi].x;
                let v = dt.vertices[vi].y;
                let nrm = self.m_surface.normal_at(u, v);
                if let Some(vd) = result.vertex.get_mut(&vk) { vd.set_normal(nrm[0], nrm[1], nrm[2]); }
            }
        }
        result
    }

    /// Mesh the surface trimmed by a plane (q0, normal), keeping the half where (S-q0).n <= 0.
    /// OCCT path-A: span-adaptive UV grid + marching-squares clip at the signed-distance field
    /// f(u,v)=(S(u,v)-q0).n, every boundary crossing Newton-refined onto the plane, coincident-3D
    /// vertices welded so periodic seams close watertight.
    pub fn mesh_by_plane(&self, q0: &Point, normal: &Vector, max_angle_deg: f64, chord_factor: f64) -> Mesh {
        let srf = &self.m_surface;
        let (mut nx, mut ny, mut nz) = (normal[0] as f64, normal[1] as f64, normal[2] as f64);
        let nl = (nx*nx+ny*ny+nz*nz).sqrt();
        if nl < 1e-12 { return srf.mesh(); }
        nx/=nl; ny/=nl; nz/=nl;
        let (qx, qy, qz) = (q0[0] as f64, q0[1] as f64, q0[2] as f64);
        let e3 = |u: f64, v: f64| -> [f64;3] {
            let p = srf.point_at(u,v).unwrap_or(Point::new(0.0,0.0,0.0)); [p[0] as f64, p[1] as f64, p[2] as f64]
        };
        let field = |u: f64, v: f64| -> f64 { let p=e3(u,v); (p[0]-qx)*nx+(p[1]-qy)*ny+(p[2]-qz)*nz };
        let refine = |mut u: f64, mut v: f64| -> (f64,f64) {
            for _ in 0..12 {
                let fv = field(u,v);
                if fv.abs() < 1e-9 { break; }
                let h = 1e-4; let a=e3(u+h,v); let b=e3(u-h,v); let c=e3(u,v+h); let d=e3(u,v-h);
                let gu = ((a[0]-b[0])*nx+(a[1]-b[1])*ny+(a[2]-b[2])*nz)/(2.0*h);
                let gv = ((c[0]-d[0])*nx+(c[1]-d[1])*ny+(c[2]-d[2])*nz)/(2.0*h);
                let g2 = gu*gu+gv*gv;
                if g2 < 1e-20 { break; }
                u -= fv*gu/g2; v -= fv*gv/g2;
            }
            (u,v)
        };
        let usp: Vec<f64> = srf.get_span_vector(0).iter().map(|&x| x as f64).collect();
        let vsp: Vec<f64> = srf.get_span_vector(1).iter().map(|&x| x as f64).collect();
        if usp.len() < 2 || vsp.len() < 2 { return srf.mesh(); }
        let deg_u = srf.degree(0); let deg_v = srf.degree(1);
        let mut bmin=[1e30f64;3]; let mut bmax=[-1e30f64;3];
        for i in 0..srf.cv_count_dir(Some(0)) { for j in 0..srf.cv_count_dir(Some(1)) {
            if let Some(p)=srf.get_cv(i,j) { for k in 0..3 { let c=p[k] as f64; if c<bmin[k]{bmin[k]=c;} if c>bmax[k]{bmax[k]=c;} } }
        }}
        let mut diag=(0..3).map(|k|(bmax[k]-bmin[k]).powi(2)).sum::<f64>().sqrt(); if diag<1e-12 { diag=1.0; }
        let ctol = diag*chord_factor;
        let span_subs = |dr: usize, sp: &[f64], osp: &[f64], deg: usize| -> Vec<usize> {
            let n = sp.len()-1; let mut out = vec![if deg>1 {2usize} else {1}; n];
            let smid = (osp[0]+osp[osp.len()-1])*0.5;
            for i in 0..n {
                let (t0,t1)=(sp[i],sp[i+1]);
                if deg>1 {
                    let mut ma=0.0_f64; let mut pn=[0.0f64;3];
                    for k in 0..=4 {
                        let t=t0+k as f64*(t1-t0)/4.0;
                        let nm = if dr==0 { srf.normal_at(t,smid) } else { srf.normal_at(smid,t) };
                        if k>0 { let d=(pn[0]*nm[0]+pn[1]*nm[1]+pn[2]*nm[2]).max(-1.0).min(1.0); ma+=d.acos()*180.0/std::f64::consts::PI; }
                        pn=[nm[0],nm[1],nm[2]];
                    }
                    out[i]=out[i].max(1.max(((ma/max_angle_deg).ceil() as usize).min(64)));
                }
                let p0 = if dr==0 { e3(t0,smid) } else { e3(smid,t0) };
                let p1 = if dr==0 { e3(t1,smid) } else { e3(smid,t1) };
                let mut dev=0.0_f64;
                for k in 1..=3 {
                    let fr=k as f64/4.0; let tm=t0+fr*(t1-t0);
                    let pm = if dr==0 { e3(tm,smid) } else { e3(smid,tm) };
                    let (lx,ly,lz)=(p0[0]+fr*(p1[0]-p0[0]),p0[1]+fr*(p1[1]-p0[1]),p0[2]+fr*(p1[2]-p0[2]));
                    dev=dev.max(((pm[0]-lx).powi(2)+(pm[1]-ly).powi(2)+(pm[2]-lz).powi(2)).sqrt());
                }
                if dev>ctol { out[i]=out[i].max(((dev/ctol).sqrt().ceil() as usize).min(64)); }
            }
            out
        };
        let us_subs=span_subs(0,&usp,&vsp,deg_u); let vs_subs=span_subs(1,&vsp,&usp,deg_v);
        let mut us=Vec::new();
        for i in 0..usp.len()-1 { for s in 0..us_subs[i] { us.push(usp[i]+(s as f64)*(usp[i+1]-usp[i])/(us_subs[i] as f64)); } }
        us.push(*usp.last().unwrap());
        let mut vs=Vec::new();
        for i in 0..vsp.len()-1 { for s in 0..vs_subs[i] { vs.push(vsp[i]+(s as f64)*(vsp[i+1]-vsp[i])/(vs_subs[i] as f64)); } }
        vs.push(*vsp.last().unwrap());
        let nu=us.len(); let nv=vs.len();
        if nu<2 || nv<2 { return srf.mesh(); }
        let f_grid: Vec<Vec<f64>> = (0..nu).map(|i| (0..nv).map(|j| field(us[i],vs[j])).collect()).collect();
        let mut result = Mesh::new();
        let wt = diag*1e-5; let cell = if wt>0.0 { wt } else { 1.0 };
        let mut cmap: std::collections::HashMap<(i64,i64,i64), Vec<([f64;3],usize)>> = std::collections::HashMap::new();
        let mut weld = |result: &mut Mesh, cmap: &mut std::collections::HashMap<(i64,i64,i64),Vec<([f64;3],usize)>>, u: f64, v: f64| -> usize {
            let p = srf.point_at(u,v).unwrap_or(Point::new(0.0,0.0,0.0));
            let (x,y,z)=(p[0] as f64,p[1] as f64,p[2] as f64);
            let (ci,cj,ck)=((x/cell).floor() as i64,(y/cell).floor() as i64,(z/cell).floor() as i64);
            for di in -1..=1 { for dj in -1..=1 { for dk in -1..=1 {
                if let Some(b)=cmap.get(&(ci+di,cj+dj,ck+dk)) { for &(pp,vk) in b { if (pp[0]-x).powi(2)+(pp[1]-y).powi(2)+(pp[2]-z).powi(2)<=wt*wt { return vk; } } }
            }}}
            let vk=result.add_vertex(p,None); let nm=srf.normal_at(u,v);
            if let Some(vd)=result.vertex.get_mut(&vk){vd.set_normal(nm[0],nm[1],nm[2]);}
            cmap.entry((ci,cj,ck)).or_default().push(([x,y,z],vk)); vk
        };
        for i in 0..nu-1 { for j in 0..nv-1 {
            let cu=[us[i],us[i+1],us[i+1],us[i]]; let cv=[vs[j],vs[j],vs[j+1],vs[j+1]];
            let inn=[f_grid[i][j]<=0.0,f_grid[i+1][j]<=0.0,f_grid[i+1][j+1]<=0.0,f_grid[i][j+1]<=0.0];
            if !inn.iter().any(|&b|b) { continue; }
            let mut poly: Vec<usize>=Vec::new();
            for k in 0..4 {
                let kn=(k+1)%4;
                if inn[k] { poly.push(weld(&mut result,&mut cmap,cu[k],cv[k])); }
                if inn[k]!=inn[kn] {
                    let (fa,fb)=(f_grid[[i,i+1,i+1,i][k]][[j,j,j+1,j+1][k]], f_grid[[i,i+1,i+1,i][kn]][[j,j,j+1,j+1][kn]]);
                    let t = if (fa-fb).abs()>1e-30 { fa/(fa-fb) } else { 0.5 };
                    let (cx,cyv)=(cu[k]+(cu[kn]-cu[k])*t, cv[k]+(cv[kn]-cv[k])*t);
                    let (ru,rv)=refine(cx,cyv);
                    poly.push(weld(&mut result,&mut cmap,ru,rv));
                }
            }
            for t in 1..poly.len().saturating_sub(1) {
                let (a,b,c)=(poly[0],poly[t],poly[t+1]);
                if a==b||b==c||c==a { continue; }
                result.add_face(vec![a,b,c],None);
            }
        }}
        if result.face.is_empty() { return srf.mesh(); }
        result
    }

    /// Multi-plane SPLIT clip: keep the region inside ALL half-spaces { (S-q).n <= 0 }. Tessellates
    /// the surface into a triangle soup (UV), then clips that soup sequentially by each plane
    /// (Sutherland-Hodgman per triangle, crossings Newton-refined onto the crossing plane), so K
    /// planes carve a clean region without per-cell CSG. Coincident 3D verts are welded.
    pub fn mesh_by_planes(&self, planes: &[(Point, Vector)], max_angle_deg: f64, chord_factor: f64) -> Mesh {
        let srf = &self.m_surface;
        let pl: Vec<([f64;3],[f64;3])> = planes.iter().filter_map(|(q,n)| {
            let (nx,ny,nz)=(n[0] as f64,n[1] as f64,n[2] as f64);
            let l=(nx*nx+ny*ny+nz*nz).sqrt();
            if l<1e-12 { None } else { Some(([q[0] as f64,q[1] as f64,q[2] as f64],[nx/l,ny/l,nz/l])) }
        }).collect();
        if pl.is_empty() { return srf.mesh(); }
        let e3 = |u: f64, v: f64| -> [f64;3] { let p=srf.point_at(u,v).unwrap_or(Point::new(0.0,0.0,0.0)); [p[0] as f64,p[1] as f64,p[2] as f64] };
        let field_k = |k: usize, u: f64, v: f64| -> f64 { let p=e3(u,v); let (q,n)=&pl[k]; (p[0]-q[0])*n[0]+(p[1]-q[1])*n[1]+(p[2]-q[2])*n[2] };
        let refine_k = |k: usize, mut u: f64, mut v: f64| -> (f64,f64) {
            let (_q,n)=&pl[k];
            for _ in 0..12 {
                let fv=field_k(k,u,v); if fv.abs()<1e-9 { break; }
                let h=1e-4; let a=e3(u+h,v); let b=e3(u-h,v); let c=e3(u,v+h); let d=e3(u,v-h);
                let gu=((a[0]-b[0])*n[0]+(a[1]-b[1])*n[1]+(a[2]-b[2])*n[2])/(2.0*h);
                let gv=((c[0]-d[0])*n[0]+(c[1]-d[1])*n[1]+(c[2]-d[2])*n[2])/(2.0*h);
                let g2=gu*gu+gv*gv; if g2<1e-20 { break; }
                u-=fv*gu/g2; v-=fv*gv/g2;
            }
            (u,v)
        };
        let usp: Vec<f64> = srf.get_span_vector(0).iter().map(|&x| x as f64).collect();
        let vsp: Vec<f64> = srf.get_span_vector(1).iter().map(|&x| x as f64).collect();
        if usp.len()<2 || vsp.len()<2 { return srf.mesh(); }
        let deg_u=srf.degree(0); let deg_v=srf.degree(1);
        let mut bmin=[1e30f64;3]; let mut bmax=[-1e30f64;3];
        for i in 0..srf.cv_count_dir(Some(0)) { for j in 0..srf.cv_count_dir(Some(1)) {
            if let Some(p)=srf.get_cv(i,j) { for k in 0..3 { let c=p[k] as f64; if c<bmin[k]{bmin[k]=c;} if c>bmax[k]{bmax[k]=c;} } }
        }}
        let mut diag=(0..3).map(|k|(bmax[k]-bmin[k]).powi(2)).sum::<f64>().sqrt(); if diag<1e-12 { diag=1.0; }
        let ctol=diag*chord_factor;
        let span_subs = |dr: usize, sp: &[f64], osp: &[f64], deg: usize| -> Vec<usize> {
            let n=sp.len()-1; let mut out=vec![if deg>1 {2usize} else {1}; n];
            let smid=(osp[0]+osp[osp.len()-1])*0.5;
            for i in 0..n {
                let (t0,t1)=(sp[i],sp[i+1]);
                if deg>1 {
                    let mut ma=0.0f64; let mut pn=[0.0f64;3];
                    for k in 0..=4 { let t=t0+k as f64*(t1-t0)/4.0; let nm=if dr==0 {srf.normal_at(t,smid)} else {srf.normal_at(smid,t)};
                        if k>0 { let d=(pn[0]*nm[0]+pn[1]*nm[1]+pn[2]*nm[2]).max(-1.0).min(1.0); ma+=d.acos()*180.0/std::f64::consts::PI; } pn=[nm[0],nm[1],nm[2]]; }
                    out[i]=out[i].max(1.max(((ma/max_angle_deg).ceil() as usize).min(64)));
                }
                let p0=if dr==0 {e3(t0,smid)} else {e3(smid,t0)}; let p1=if dr==0 {e3(t1,smid)} else {e3(smid,t1)};
                let mut dev=0.0f64;
                for k in 1..=3 { let fr=k as f64/4.0; let tm=t0+fr*(t1-t0); let pm=if dr==0 {e3(tm,smid)} else {e3(smid,tm)};
                    let (lx,ly,lz)=(p0[0]+fr*(p1[0]-p0[0]),p0[1]+fr*(p1[1]-p0[1]),p0[2]+fr*(p1[2]-p0[2]));
                    dev=dev.max(((pm[0]-lx).powi(2)+(pm[1]-ly).powi(2)+(pm[2]-lz).powi(2)).sqrt()); }
                if dev>ctol { out[i]=out[i].max(((dev/ctol).sqrt().ceil() as usize).min(64)); }
            }
            out
        };
        let us_subs=span_subs(0,&usp,&vsp,deg_u); let vs_subs=span_subs(1,&vsp,&usp,deg_v);
        let mut us=Vec::new(); for i in 0..usp.len()-1 { for s in 0..us_subs[i] { us.push(usp[i]+(s as f64)*(usp[i+1]-usp[i])/(us_subs[i] as f64)); } } us.push(*usp.last().unwrap());
        let mut vs=Vec::new(); for i in 0..vsp.len()-1 { for s in 0..vs_subs[i] { vs.push(vsp[i]+(s as f64)*(vsp[i+1]-vsp[i])/(vs_subs[i] as f64)); } } vs.push(*vsp.last().unwrap());
        let nu=us.len(); let nv=vs.len(); if nu<2||nv<2 { return srf.mesh(); }
        let mut tris: Vec<[(f64,f64);3]> = Vec::with_capacity((nu-1)*(nv-1)*2);
        for i in 0..nu-1 { for j in 0..nv-1 {
            let a=(us[i],vs[j]); let b=(us[i+1],vs[j]); let c=(us[i+1],vs[j+1]); let d=(us[i],vs[j+1]);
            tris.push([a,b,c]); tris.push([a,c,d]);
        }}
        let eps=1e-9;
        for k in 0..pl.len() {
            let mut next: Vec<[(f64,f64);3]> = Vec::new();
            for t in &tris {
                let mut poly: Vec<(f64,f64)> = Vec::new();
                for e in 0..3 {
                    let p=t[e]; let q=t[(e+1)%3];
                    let fp=field_k(k,p.0,p.1); let fq=field_k(k,q.0,q.1);
                    let (pin,qin)=(fp<=eps, fq<=eps);
                    if pin { poly.push(p); }
                    if pin!=qin {
                        let tt= if (fp-fq).abs()>1e-30 { fp/(fp-fq) } else { 0.5 };
                        let cu=p.0+(q.0-p.0)*tt; let cv=p.1+(q.1-p.1)*tt;
                        poly.push(refine_k(k,cu,cv));
                    }
                }
                for w in 1..poly.len().saturating_sub(1) { next.push([poly[0],poly[w],poly[w+1]]); }
            }
            tris=next;
            if tris.is_empty() { break; }
        }
        if tris.is_empty() { return Mesh::new(); }
        let mut result=Mesh::new();
        let wt=diag*1e-5; let cell=if wt>0.0 {wt} else {1.0};
        let mut cmap: std::collections::HashMap<(i64,i64,i64),Vec<([f64;3],usize)>> = std::collections::HashMap::new();
        let mut weld = |result:&mut Mesh, cmap:&mut std::collections::HashMap<(i64,i64,i64),Vec<([f64;3],usize)>>, u:f64,v:f64| -> usize {
            let p=srf.point_at(u,v).unwrap_or(Point::new(0.0,0.0,0.0));
            let (x,y,z)=(p[0] as f64,p[1] as f64,p[2] as f64);
            let (ci,cj,ck)=((x/cell).floor() as i64,(y/cell).floor() as i64,(z/cell).floor() as i64);
            for di in -1..=1 { for dj in -1..=1 { for dk in -1..=1 {
                if let Some(b)=cmap.get(&(ci+di,cj+dj,ck+dk)) { for &(pp,vk) in b { if (pp[0]-x).powi(2)+(pp[1]-y).powi(2)+(pp[2]-z).powi(2)<=wt*wt { return vk; } } }
            }}}
            let vk=result.add_vertex(p,None); let nm=srf.normal_at(u,v);
            if let Some(vd)=result.vertex.get_mut(&vk){vd.set_normal(nm[0],nm[1],nm[2]);}
            cmap.entry((ci,cj,ck)).or_default().push(([x,y,z],vk)); vk
        };
        for t in &tris {
            let a=weld(&mut result,&mut cmap,t[0].0,t[0].1);
            let b=weld(&mut result,&mut cmap,t[1].0,t[1].1);
            let c=weld(&mut result,&mut cmap,t[2].0,t[2].1);
            if a==b||b==c||c==a { continue; }
            result.add_face(vec![a,b,c],None);
        }
        if result.face.is_empty() { return Mesh::new(); }
        result
    }

    /// Split a surface into every non-empty region carved by `planes` (all 2^K sign combinations).
    /// Each region comes back as a first-class multi-plane NurbsSurfaceTrimmed, so the viewer can
    /// select / hide / transform each piece. Plane normals define which half is f<=0.
    pub fn split_by_planes(srf: &NurbsSurface, planes: &[(Point, Vector)]) -> Vec<NurbsSurfaceTrimmed> {
        let k=planes.len(); if k==0 || k>16 { return Vec::new(); }
        let mut out=Vec::new();
        for mask in 0u32..(1u32<<k) {
            let mut cp: Vec<(Point,Vector)> = Vec::with_capacity(k);
            for (i,(q,n)) in planes.iter().enumerate() {
                let flip=((mask>>i)&1)==1;
                let nn=if flip { Vector::new(-n[0],-n[1],-n[2]) } else { Vector::new(n[0],n[1],n[2]) };
                cp.push((q.clone(), nn));
            }
            let mut ts=NurbsSurfaceTrimmed::new();
            ts.m_surface=srf.clone();
            ts.cut_planes=cp;
            let m=ts.mesh_render(20.0,0.01);
            if m.number_of_faces()>0 { out.push(ts); }
        }
        out
    }

    pub fn transform_self(&mut self) {
        let xf = self.xform.clone();
        self.m_surface.transform(&xf);
        self.xform = Xform::identity();
    }

    pub fn transformed(&self) -> Self {
        let mut ts = self.clone();
        ts.transform_self();
        ts
    }

    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = std::sync::OnceLock::new();
        copy
    }

    pub fn to_string(&self) -> String {
        format!("NurbsSurfaceTrimmed(name={}, trimmed={}, holes={})",
                self.name, self.is_trimmed(), self.inner_loop_count())
    }

    pub fn repr(&self) -> String {
        format!("NurbsSurfaceTrimmed(\n  name={},\n  trimmed={},\n  holes={},\n  surface={}\n)",
                self.name, self.is_trimmed(), self.inner_loop_count(), self.m_surface.to_string())
    }

    // JSON serialization
    pub fn file_json_dump(&self, filename: &str) {
        use std::fs::File;
        use std::io::Write;
        if let Ok(json) = serde_json::to_string_pretty(self) {
            if let Ok(mut file) = File::create(filename) {
                let _ = file.write_all(json.as_bytes());
            }
        }
    }

    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn file_json_loads(json_string: &str) -> Self {
        serde_json::from_str(json_string).unwrap_or_else(|_| Self::default())
    }

    pub fn file_json_load(filename: &str) -> Self {
        use std::fs::File;
        use std::io::Read;
        let mut file = match File::open(filename) {
            Ok(f) => f,
            Err(_) => return Self::default(),
        };
        let mut contents = String::new();
        if file.read_to_string(&mut contents).is_err() {
            return Self::default();
        }
        serde_json::from_str(&contents).unwrap_or_else(|_| Self::default())
    }

    // Protobuf serialization
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        let surface_data = self.m_surface.pb_dumps();
        let surface_proto = crate::proto::NurbsSurface::decode(surface_data.as_slice()).unwrap();

        let outer_loop = if self.is_trimmed() {
            let data = self.m_outer_loop.as_ref().unwrap().to_protobuf();
            Some(crate::proto::NurbsCurve::decode(data.as_slice()).unwrap())
        } else {
            None
        };

        let inner_loops: Vec<_> = self.m_inner_loops.iter().map(|c| {
            let data = c.to_protobuf();
            crate::proto::NurbsCurve::decode(data.as_slice()).unwrap()
        }).collect();

        let proto = crate::proto::NurbsSurfaceTrimmed {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            width: self.width as f64,
            surface: Some(surface_proto),
            outer_loop,
            inner_loops,
            surfacecolor: Some(crate::proto::Color {
                guid: self.surfacecolor.guid().to_string(),
                name: self.surfacecolor.name.clone(),
                r: self.surfacecolor.r,
                g: self.surfacecolor.g,
                b: self.surfacecolor.b,
                a: self.surfacecolor.a,
            }),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid().to_string(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.iter().map(|&v| v as f64).collect(),
            }),
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        let proto = crate::proto::NurbsSurfaceTrimmed::decode(data)?;
        let mut ts = Self::new();
        ts.set_guid(proto.guid.clone());
        ts.name = proto.name;
        ts.width = proto.width as f64;

        if let Some(srf_proto) = proto.surface {
            let srf_data = srf_proto.encode_to_vec();
            ts.m_surface = NurbsSurface::pb_loads(&srf_data)?;
        }

        if let Some(ol) = proto.outer_loop {
            let data = ol.encode_to_vec();
            if let Ok(crv) = NurbsCurve::from_protobuf(&data) {
                ts.m_outer_loop = Some(crv);
            }
        }

        for il in proto.inner_loops {
            let data = il.encode_to_vec();
            if let Ok(crv) = NurbsCurve::from_protobuf(&data) {
                ts.m_inner_loops.push(crv);
            }
        }

        if let Some(color) = proto.surfacecolor {
            ts.surfacecolor.set_guid(color.guid.clone());
            ts.surfacecolor.name = color.name;
            ts.surfacecolor.r = color.r;
            ts.surfacecolor.g = color.g;
            ts.surfacecolor.b = color.b;
            ts.surfacecolor.a = color.a;
        }

        if let Some(xform) = proto.xform {
            ts.xform.set_guid(xform.guid.clone());
            ts.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 { ts.xform.m[i] = *val as f64; }
            }
        }

        Ok(ts)
    }

    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }
}

impl Default for NurbsSurfaceTrimmed {
    fn default() -> Self { Self::new() }
}

impl PartialEq for NurbsSurfaceTrimmed {
    fn eq(&self, other: &Self) -> bool {
        if self.name != other.name { return false; }
        if self.width != other.width { return false; }
        if self.surfacecolor != other.surfacecolor { return false; }
        if self.xform != other.xform { return false; }
        if self.m_surface != other.m_surface { return false; }
        true
    }
}

impl std::fmt::Display for NurbsSurfaceTrimmed {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_string())
    }
}
