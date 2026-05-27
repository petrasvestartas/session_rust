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
        'find: for i in (0..self.triangles.len()).rev() {
            if !self.triangles[i].alive { continue; }
            for k in 0..3 { if self.triangles[i].v[k] == v0 { start_ti = i as i32; break 'find; } }
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
        let _first_new = self.triangles.len() as i32;
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
    pub width: f32,
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
                    let t = spans[si] + (spans[si+1]-spans[si]) * k as f32 / 10.0;
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
            let t = dom.0 + (dom.1 - dom.0) * i as f32 / n_samples as f32;
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

    pub fn point_at(&self, u: f32, v: f32) -> Option<Point> { self.m_surface.point_at(u, v) }
    pub fn normal_at(&self, u: f32, v: f32) -> Vector { self.m_surface.normal_at(u, v) }

    pub fn mesh(&self) -> Mesh {
        if !self.is_trimmed() { return self.m_surface.mesh(); }
        let planar = self.m_surface.is_planar(1e-6);
        let disc_loop = |crv: &NurbsCurve| -> Vec<[f64; 2]> {
            let mut pts: Vec<[f64; 2]> = if crv.degree() <= 1 && !crv.is_rational() {
                (0..crv.cv_count()).filter_map(|i| crv.get_cv(i)).map(|p| [p[0] as f64, p[1] as f64]).collect()
            } else {
                let n = (crv.cv_count() * 4).max(16);
                let (sampled, _) = crv.divide_by_count(n, false);
                sampled.iter().map(|p| [p[0] as f64, p[1] as f64]).collect()
            };
            while pts.len() > 1 {
                let dx = pts[0][0] - pts[pts.len()-1][0];
                let dy = pts[0][1] - pts[pts.len()-1][1];
                if dx*dx + dy*dy < 1e-20 { pts.pop(); } else { break; }
            }
            pts
        };
        let outer_uv = disc_loop(self.m_outer_loop.as_ref().unwrap());
        let hole_uvs: Vec<Vec<[f64;2]>> = self.m_inner_loops.iter().map(|c| disc_loop(c)).collect();
        if outer_uv.len() < 3 { return self.m_surface.mesh(); }
        let mut bb_umin = 1e30_f64; let mut bb_vmin = 1e30_f64;
        let mut bb_umax = -1e30_f64; let mut bb_vmax = -1e30_f64;
        for p in &outer_uv {
            if p[0] < bb_umin { bb_umin = p[0]; } if p[1] < bb_vmin { bb_vmin = p[1]; }
            if p[0] > bb_umax { bb_umax = p[0]; } if p[1] > bb_vmax { bb_vmax = p[1]; }
        }
        let point_in_polygon = |u: f64, v: f64, poly: &[[f64;2]]| -> bool {
            let n = poly.len(); let mut inside = false; let mut j = n - 1;
            for i in 0..n {
                let xi = poly[i][0]; let yi = poly[i][1];
                let xj = poly[j][0]; let yj = poly[j][1];
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
        if planar {
            use crate::remesh_cdt::cdt_triangulate;
            let signed_area = |pts: &[[f64; 2]]| -> f64 {
                let n = pts.len();
                let mut a = 0.0_f64;
                for i in 0..n {
                    let j = (i + 1) % n;
                    a += pts[i][0] * pts[j][1] - pts[j][0] * pts[i][1];
                }
                a * 0.5
            };
            let mut border_uv = outer_uv.clone();
            if signed_area(&border_uv) < 0.0 { border_uv.reverse(); }
            let mut holes_uv = hole_uvs.clone();
            for h in &mut holes_uv {
                if signed_area(h) > 0.0 { h.reverse(); }
            }
            let border_pts: Vec<Point> = border_uv.iter().map(|p| Point::new(p[0] as f32, p[1] as f32, 0.0)).collect();
            let holes_pts: Vec<Vec<Point>> = holes_uv.iter().map(|h| h.iter().map(|p| Point::new(p[0] as f32, p[1] as f32, 0.0)).collect()).collect();
            let tris = cdt_triangulate(&border_pts, &holes_pts);
            if tris.is_empty() { return self.m_surface.mesh(); }
            let mut flat_uv: Vec<[f32; 2]> = border_uv.iter().map(|p| [p[0] as f32, p[1] as f32]).collect();
            for h in &holes_uv { flat_uv.extend(h.iter().map(|p| [p[0] as f32, p[1] as f32])); }
            let mut result = Mesh::new();
            let mut vert_map: Vec<Option<usize>> = vec![None; flat_uv.len()];
            for &(a, b, c) in &tris {
                for &vi in &[a, b, c] {
                    if vert_map[vi].is_none() {
                        let u = flat_uv[vi][0];
                        let v = flat_uv[vi][1];
                        let p3d = self.m_surface.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0));
                        vert_map[vi] = Some(result.add_vertex(p3d, None));
                    }
                }
            }
            for &(a, b, c) in &tris {
                let v0 = vert_map[a].unwrap();
                let v1 = vert_map[b].unwrap();
                let v2 = vert_map[c].unwrap();
                if v0 == v1 || v1 == v2 || v2 == v0 { continue; }
                result.add_face(vec![v0, v1, v2], None);
            }
            let dom_u = self.m_surface.domain(0).unwrap_or((0.0, 1.0));
            let dom_v = self.m_surface.domain(1).unwrap_or((0.0, 1.0));
            let nrm = self.m_surface.normal_at((dom_u.0 + dom_u.1) / 2.0, (dom_v.0 + dom_v.1) / 2.0);
            for (_, vd) in result.vertex.iter_mut() { vd.set_normal(nrm[0], nrm[1], nrm[2]); }
            return result;
        }
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
        if !planar {
            let usp: Vec<f64> = self.m_surface.get_span_vector(0).iter().map(|&v| v as f64).collect();
            let vsp: Vec<f64> = self.m_surface.get_span_vector(1).iter().map(|&v| v as f64).collect();
            let deg_u = self.m_surface.degree(0);
            let deg_v = self.m_surface.degree(1);
            let ns_u = usp.len().saturating_sub(1);
            let ns_v = vsp.len().saturating_sub(1);
            let mut bmin = [1e30f32; 3]; let mut bmax = [-1e30f32; 3];
            for i in 0..self.m_surface.cv_count_dir(Some(0)) {
                for j in 0..self.m_surface.cv_count_dir(Some(1)) {
                    if let Some(p) = self.m_surface.get_cv(i, j) {
                        for k in 0..3 { if p[k] < bmin[k] { bmin[k]=p[k]; } if p[k] > bmax[k] { bmax[k]=p[k]; } }
                    }
                }
            }
            let bbox_diag = (0..3).map(|k| (bmax[k]-bmin[k]).powi(2)).sum::<f32>().sqrt() as f64;
            let max_angle_deg = 20.0_f64;
            let pe00 = self.m_surface.point_at(usp[0] as f32, vsp[0] as f32).unwrap_or(Point::new(0.0,0.0,0.0));
            let pe10 = self.m_surface.point_at(*usp.last().unwrap() as f32, vsp[0] as f32).unwrap_or(Point::new(0.0,0.0,0.0));
            let pe01 = self.m_surface.point_at(usp[0] as f32, *vsp.last().unwrap() as f32).unwrap_or(Point::new(0.0,0.0,0.0));
            let l1 = ((pe10[0]-pe00[0]).powi(2)+(pe10[1]-pe00[1]).powi(2)+(pe10[2]-pe00[2]).powi(2)).sqrt() as f64;
            let l2 = ((pe01[0]-pe00[0]).powi(2)+(pe01[1]-pe00[1]).powi(2)+(pe01[2]-pe00[2]).powi(2)).sqrt() as f64;
            let max_dim = l1.max(l2);
            let max_edge_len = if max_dim > 1e-10 { max_dim / 10.0 } else { 0.0 };
            let span_subs = |dir: usize, sp: &[f64], osp: &[f64], deg: usize| -> Vec<usize> {
                let n = sp.len().saturating_sub(1);
                let mut subs = vec![1usize; n];
                let s_pos: Vec<f64> = (0..osp.len().saturating_sub(1)).map(|k| (osp[k]+osp[k+1])*0.5).collect();
                for i in 0..n {
                    let t0 = sp[i]; let t1 = sp[i+1];
                    if deg > 1 {
                        let mut ma = 0.0_f64;
                        for &s in &s_pos {
                            let mut ta = 0.0_f64;
                            let mut pn = [0.0f32; 3];
                            for k in 0..=4 {
                                let t = (t0 + k as f64*(t1-t0)/4.0) as f32;
                                let (su, sv) = if dir==0 { (t, s as f32) } else { (s as f32, t) };
                                let nrm = self.m_surface.normal_at(su, sv);
                                if k > 0 {
                                    let d = (pn[0]*nrm[0]+pn[1]*nrm[1]+pn[2]*nrm[2]).max(-1.0).min(1.0);
                                    ta += (d.acos() as f64) * 180.0 / std::f64::consts::PI;
                                }
                                pn = [nrm[0], nrm[1], nrm[2]];
                            }
                            if ta > ma { ma = ta; }
                        }
                        subs[i] = 1.max(((ma / max_angle_deg).ceil() as usize).min(24));
                    }
                    let chord_tol = bbox_diag * 0.005;
                    let nc = s_pos.len().min(3);
                    let mut max_dev = 0.0_f64;
                    for ci in 0..=nc {
                        let s = osp[0] + ci as f64*(osp[osp.len()-1]-osp[0])/(nc.max(1) as f64);
                        let (p0u, p0v) = if dir==0 { (t0 as f32, s as f32) } else { (s as f32, t0 as f32) };
                        let (p1u, p1v) = if dir==0 { (t1 as f32, s as f32) } else { (s as f32, t1 as f32) };
                        let pt0 = self.m_surface.point_at(p0u, p0v).unwrap_or(Point::new(0.0,0.0,0.0));
                        let pt1 = self.m_surface.point_at(p1u, p1v).unwrap_or(Point::new(0.0,0.0,0.0));
                        for k in 1..=3 {
                            let frac = k as f64 / 4.0;
                            let tm = (t0 + frac*(t1-t0)) as f32;
                            let (pmu, pmv) = if dir==0 { (tm, s as f32) } else { (s as f32, tm) };
                            let ptm = self.m_surface.point_at(pmu, pmv).unwrap_or(Point::new(0.0,0.0,0.0));
                            let lx = pt0[0]+(pt1[0]-pt0[0])*frac as f32;
                            let ly = pt0[1]+(pt1[1]-pt0[1])*frac as f32;
                            let lz = pt0[2]+(pt1[2]-pt0[2])*frac as f32;
                            let dev = (((ptm[0]-lx).powi(2)+(ptm[1]-ly).powi(2)+(ptm[2]-lz).powi(2)).sqrt()) as f64;
                            if dev > max_dev { max_dev = dev; }
                        }
                    }
                    if max_dev > chord_tol {
                        let cs = 2.max(((max_dev/chord_tol).sqrt().ceil() as usize).min(24));
                        if cs > subs[i] { subs[i] = cs; }
                    }
                    if max_edge_len > 0.0 {
                        let s_mid = (osp[0] + osp[osp.len()-1]) * 0.5;
                        let (a0u, a0v) = if dir==0 { (t0 as f32, s_mid as f32) } else { (s_mid as f32, t0 as f32) };
                        let (a1u, a1v) = if dir==0 { (t1 as f32, s_mid as f32) } else { (s_mid as f32, t1 as f32) };
                        let pa0 = self.m_surface.point_at(a0u, a0v).unwrap_or(Point::new(0.0,0.0,0.0));
                        let pa1 = self.m_surface.point_at(a1u, a1v).unwrap_or(Point::new(0.0,0.0,0.0));
                        let sl = ((pa1[0]-pa0[0]).powi(2)+(pa1[1]-pa0[1]).powi(2)+(pa1[2]-pa0[2]).powi(2)).sqrt() as f64;
                        let es = 1.max(((sl/max_edge_len).ceil() as usize).min(64));
                        if es > subs[i] { subs[i] = es; }
                    }
                    if deg > 1 && subs[i] < 2 { subs[i] = 2; }
                }
                subs
            };
            let u_subs = span_subs(0, &usp, &vsp, deg_u);
            let v_subs = span_subs(1, &vsp, &usp, deg_v);
            let mut us: Vec<f64> = Vec::new();
            for i in 0..ns_u {
                for s in 0..u_subs[i] { us.push(usp[i] + (s as f64)*(usp[i+1]-usp[i])/(u_subs[i] as f64)); }
            }
            if let Some(&last) = usp.last() { us.push(last); }
            let mut vs: Vec<f64> = Vec::new();
            for i in 0..ns_v {
                for s in 0..v_subs[i] { vs.push(vsp[i] + (s as f64)*(vsp[i+1]-vsp[i])/(v_subs[i] as f64)); }
            }
            if let Some(&last) = vsp.last() { vs.push(last); }
            for &u in &us { for &v in &vs { if inside_trim(u, v) { dt.insert(u, v); } } }
        }
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
        for &[a, b, c] in &tris {
            for &vi in &[a, b, c] {
                if vert_map[vi as usize].is_none() {
                    let u = dt.vertices[vi as usize].x as f32;
                    let v = dt.vertices[vi as usize].y as f32;
                    let p3d = self.m_surface.point_at(u, v).unwrap_or(Point::new(0.0,0.0,0.0));
                    let vk = result.add_vertex(p3d, None);
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
        if planar {
            let dom_u = self.m_surface.domain(0).unwrap_or((0.0, 1.0));
            let dom_v = self.m_surface.domain(1).unwrap_or((0.0, 1.0));
            let nrm = self.m_surface.normal_at((dom_u.0+dom_u.1)/2.0, (dom_v.0+dom_v.1)/2.0);
            for (_, vd) in result.vertex.iter_mut() { vd.set_normal(nrm[0], nrm[1], nrm[2]); }
        } else {
            for vi in 0..nv {
                if let Some(vk) = vert_map[vi] {
                    let u = dt.vertices[vi].x as f32;
                    let v = dt.vertices[vi].y as f32;
                    let nrm = self.m_surface.normal_at(u, v);
                    if let Some(vd) = result.vertex.get_mut(&vk) { vd.set_normal(nrm[0], nrm[1], nrm[2]); }
                }
            }
        }
        result
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
        ts.width = proto.width as f32;

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
                if i < 16 { ts.xform.m[i] = *val as f32; }
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
