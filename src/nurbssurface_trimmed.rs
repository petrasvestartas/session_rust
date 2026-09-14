#![allow(
    clippy::manual_range_contains,
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]
use crate::closest::Closest;
use crate::color::Color;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
use crate::primitives::Primitives;
use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
use crate::vector::Vector;
use crate::xform::Xform;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::collections::HashSet;

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Normal on the side of a C0 knot line that belongs to the triangle around center
fn crease_side_normal(
    surface: &NurbsSurface,
    knots: &[Vec<f64>; 2],
    center: [f64; 2],
    mut uv: [f64; 2],
) -> Vector {
    for dir in 0..2 {
        if knots[dir].contains(&uv[dir]) {
            if center[dir] < uv[dir] {
                uv[dir] = uv[dir].next_down();
            }
            if center[dir] > uv[dir] {
                uv[dir] = uv[dir].next_up();
            }
        }
    }
    surface.normal_at(uv[0], uv[1])
}

/// Winding-number test of (u, v) against a closed UV polygon
fn point_in_polygon_2d(u: f64, v: f64, poly: &[Point]) -> bool {
    let mut winding = 0;
    let n = poly.len();
    for i in 0..n {
        let j = (i + 1) % n;
        let x0 = poly[i][0];
        let y0 = poly[i][1];
        let x1 = poly[j][0];
        let y1 = poly[j][1];
        let cross = (x1 - x0) * (v - y0) - (y1 - y0) * (u - x0);
        if y0 <= v && y1 > v && cross > 0.0 {
            winding += 1;
        }
        if y0 > v && y1 <= v && cross < 0.0 {
            winding -= 1;
        }
    }
    winding != 0
}

// ═══════════════════════════════════════════════════════════════════════════
// Delaunay2D
// ═══════════════════════════════════════════════════════════════════════════

struct Vertex2D {
    x: f64,
    y: f64,
}

struct Triangle {
    v: [i32; 3],
    adj: [i32; 3],
    constrained: [bool; 3],
    alive: bool,
}

struct Delaunay2D {
    vertices: Vec<Vertex2D>,
    triangles: Vec<Triangle>,
    super_v: [i32; 3],
    edge_map: HashMap<(i32, i32), (i32, i32)>,
    last_found: i32,
}

impl Delaunay2D {
    fn edge_key(a: i32, b: i32) -> (i32, i32) {
        (a.min(b), a.max(b))
    }

    fn in_circumcircle(
        ax: f64,
        ay: f64,
        bx: f64,
        by: f64,
        cx: f64,
        cy: f64,
        dx: f64,
        dy: f64,
    ) -> f64 {
        let adx = ax - dx;
        let ady = ay - dy;
        let bdx = bx - dx;
        let bdy = by - dy;
        let cdx = cx - dx;
        let cdy = cy - dy;
        (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy)
            + (bdx * bdx + bdy * bdy) * (cdx * ady - adx * cdy)
            + (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady)
    }

    fn orient2d(ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> f64 {
        (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    }

    fn new(xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Self {
        let dx = xmax - xmin;
        let dy = ymax - ymin;
        let d = dx.max(dy);
        let cx = (xmin + xmax) * 0.5;
        let cy = (ymin + ymax) * 0.5;
        let scale = 20.0;
        let mut dt = Delaunay2D {
            vertices: Vec::new(),
            triangles: Vec::new(),
            super_v: [-1; 3],
            edge_map: HashMap::new(),
            last_found: 0,
        };
        dt.vertices.push(Vertex2D {
            x: cx - scale * d,
            y: cy - scale * d,
        });
        dt.vertices.push(Vertex2D {
            x: cx + scale * d,
            y: cy - scale * d,
        });
        dt.vertices.push(Vertex2D {
            x: cx,
            y: cy + scale * d,
        });
        dt.super_v = [0, 1, 2];
        dt.triangles.push(Triangle {
            v: [0, 1, 2],
            adj: [-1, -1, -1],
            constrained: [false; 3],
            alive: true,
        });
        dt.register_edges(0);
        dt
    }

    fn register_edges(&mut self, ti: i32) {
        for k in 0..3 {
            let a = self.triangles[ti as usize].v[(k + 1) % 3];
            let b = self.triangles[ti as usize].v[(k + 2) % 3];
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
            let a = self.triangles[ti as usize].v[(k + 1) % 3];
            let b = self.triangles[ti as usize].v[(k + 2) % 3];
            let key = Self::edge_key(a, b);
            let adj_ti = self.triangles[ti as usize].adj[k];
            if adj_ti >= 0
                && adj_ti < self.triangles.len() as i32
                && self.triangles[adj_ti as usize].alive
            {
                let mut adj_kk = -1i32;
                for kk in 0..3 {
                    if self.triangles[adj_ti as usize].adj[kk] == ti {
                        adj_kk = kk as i32;
                        break;
                    }
                }
                if adj_kk >= 0 {
                    self.triangles[adj_ti as usize].adj[adj_kk as usize] = -1;
                    let adj_a = self.triangles[adj_ti as usize].v[(adj_kk as usize + 1) % 3];
                    let adj_b = self.triangles[adj_ti as usize].v[(adj_kk as usize + 2) % 3];
                    self.edge_map
                        .insert(Self::edge_key(adj_a, adj_b), (adj_ti, adj_kk));
                }
            } else if let Some(&(eti, _)) = self.edge_map.get(&key) {
                if eti == ti {
                    self.edge_map.remove(&key);
                }
            }
        }
    }

    fn locate(&self, x: f64, y: f64, mut start: i32) -> i32 {
        if start < 0
            || start >= self.triangles.len() as i32
            || !self.triangles[start as usize].alive
        {
            start = self.triangles.len() as i32 - 1;
            while start >= 0 && !self.triangles[start as usize].alive {
                start -= 1;
            }
            if start < 0 {
                return -1;
            }
        }
        let mut cur = start;
        let max_iter = self.triangles.len() as i32;
        for _ in 0..max_iter {
            let t = &self.triangles[cur as usize];
            let mut moved = false;
            for k in 0..3 {
                let a = t.v[k];
                let b = t.v[(k + 1) % 3];
                let ax = self.vertices[a as usize].x;
                let ay = self.vertices[a as usize].y;
                let bx = self.vertices[b as usize].x;
                let by = self.vertices[b as usize].y;
                if Self::orient2d(ax, ay, bx, by, x, y) < 0.0 {
                    let opp = t.adj[(k + 2) % 3];
                    if opp >= 0
                        && (opp as usize) < self.triangles.len()
                        && self.triangles[opp as usize].alive
                    {
                        cur = opp;
                        moved = true;
                        break;
                    }
                }
            }
            if !moved {
                return cur;
            }
        }
        cur
    }

    fn insert(&mut self, x: f64, y: f64) -> i32 {
        let start = self.locate(x, y, self.last_found);
        if start >= 0 {
            let t = &self.triangles[start as usize];
            for k in 0..3 {
                let vi2 = t.v[k];
                let ddx = self.vertices[vi2 as usize].x - x;
                let ddy = self.vertices[vi2 as usize].y - y;
                if ddx * ddx + ddy * ddy < 1e-12 {
                    return vi2;
                }
            }
        }
        let vi = self.vertices.len() as i32;
        self.vertices.push(Vertex2D { x, y });
        let mut bad: Vec<i32> = Vec::new();
        let mut visited: HashSet<i32> = HashSet::new();
        if start >= 0 {
            bad.push(start);
            visited.insert(start);
        }
        let mut bfs_front = 0;
        while bfs_front < bad.len() {
            let ti = bad[bfs_front];
            bfs_front += 1;
            if !self.triangles[ti as usize].alive {
                continue;
            }
            let [v0, v1, v2] = self.triangles[ti as usize].v;
            let ax = self.vertices[v0 as usize].x;
            let ay = self.vertices[v0 as usize].y;
            let bx = self.vertices[v1 as usize].x;
            let by = self.vertices[v1 as usize].y;
            let cx = self.vertices[v2 as usize].x;
            let cy = self.vertices[v2 as usize].y;
            let o = Self::orient2d(ax, ay, bx, by, cx, cy);
            let ic = if o > 0.0 {
                Self::in_circumcircle(ax, ay, bx, by, cx, cy, x, y)
            } else {
                Self::in_circumcircle(ax, ay, cx, cy, bx, by, x, y)
            };
            if ic > 0.0 {
                for k in 0..3 {
                    if self.triangles[ti as usize].constrained[k] {
                        continue;
                    }
                    let nb = self.triangles[ti as usize].adj[k];
                    if nb >= 0 && !visited.contains(&nb) {
                        visited.insert(nb);
                        bad.push(nb);
                    }
                }
            }
        }
        let mut kept: Vec<i32> = Vec::new();
        for &ti in &bad {
            if !self.triangles[ti as usize].alive {
                continue;
            }
            let [v0, v1, v2] = self.triangles[ti as usize].v;
            let ax = self.vertices[v0 as usize].x;
            let ay = self.vertices[v0 as usize].y;
            let bx = self.vertices[v1 as usize].x;
            let by = self.vertices[v1 as usize].y;
            let cx = self.vertices[v2 as usize].x;
            let cy = self.vertices[v2 as usize].y;
            let o = Self::orient2d(ax, ay, bx, by, cx, cy);
            let ic = if o > 0.0 {
                Self::in_circumcircle(ax, ay, bx, by, cx, cy, x, y)
            } else {
                Self::in_circumcircle(ax, ay, cx, cy, bx, by, x, y)
            };
            if ic > 0.0 {
                kept.push(ti);
            }
        }
        let bad = kept;
        if bad.is_empty() {
            self.vertices.pop();
            return -1;
        }
        let bad_set: HashSet<i32> = bad.iter().copied().collect();
        let mut polygon: Vec<(i32, i32, bool)> = Vec::new();
        for &ti in &bad {
            let t = &self.triangles[ti as usize];
            for k in 0..3 {
                let nb = t.adj[k];
                if nb < 0 || !bad_set.contains(&nb) {
                    polygon.push((t.v[(k + 1) % 3], t.v[(k + 2) % 3], t.constrained[k]));
                }
            }
        }
        for &ti in &bad {
            self.unregister_edges(ti);
            self.triangles[ti as usize].alive = false;
        }
        for (e0, e1, constr) in polygon {
            let o = Self::orient2d(
                self.vertices[vi as usize].x,
                self.vertices[vi as usize].y,
                self.vertices[e0 as usize].x,
                self.vertices[e0 as usize].y,
                self.vertices[e1 as usize].x,
                self.vertices[e1 as usize].y,
            );
            if o.abs() < 1e-20 {
                continue;
            }
            let new_ti = self.triangles.len() as i32;
            let (va, vb) = if o > 0.0 { (e0, e1) } else { (e1, e0) };
            self.triangles.push(Triangle {
                v: [vi, va, vb],
                adj: [-1, -1, -1],
                constrained: [constr, false, false],
                alive: true,
            });
            self.register_edges(new_ti);
        }
        self.last_found = self.triangles.len() as i32 - 1;
        vi
    }

    fn insert_constraint(&mut self, v0: i32, v1: i32) {
        if v0 == v1 {
            return;
        }
        for ti in 0..self.triangles.len() {
            if !self.triangles[ti].alive {
                continue;
            }
            for k in 0..3 {
                let e0 = self.triangles[ti].v[(k + 1) % 3];
                let e1 = self.triangles[ti].v[(k + 2) % 3];
                if (e0 == v0 && e1 == v1) || (e0 == v1 && e1 == v0) {
                    self.triangles[ti].constrained[k] = true;
                    let nb = self.triangles[ti].adj[k];
                    if nb >= 0
                        && (nb as usize) < self.triangles.len()
                        && self.triangles[nb as usize].alive
                    {
                        for kk in 0..3 {
                            if self.triangles[nb as usize].adj[kk] == ti as i32 {
                                self.triangles[nb as usize].constrained[kk] = true;
                                break;
                            }
                        }
                    }
                    return;
                }
            }
        }
        let mut start_ti = -1i32;
        for i in (0..self.triangles.len()).rev() {
            if !self.triangles[i].alive {
                continue;
            }
            for k in 0..3 {
                if self.triangles[i].v[k] == v0 {
                    start_ti = i as i32;
                    break;
                }
            }
        }
        if start_ti < 0 {
            return;
        }
        let ax = self.vertices[v0 as usize].x;
        let ay = self.vertices[v0 as usize].y;
        let bx = self.vertices[v1 as usize].x;
        let by = self.vertices[v1 as usize].y;
        let mut ivl = -1i32;
        let mut ivr = -1i32;
        let mut it = -1i32;
        {
            let mut ti = start_ti;
            let guard = self.triangles.len() as i32 + 4;
            let mut g = 0;
            loop {
                if g > guard || !self.triangles[ti as usize].alive {
                    break;
                }
                g += 1;
                let mut k_v0 = -1i32;
                for i in 0..3 {
                    if self.triangles[ti as usize].v[i] == v0 {
                        k_v0 = i as i32;
                        break;
                    }
                }
                if k_v0 < 0 {
                    break;
                }
                let k = k_v0 as usize;
                let ip2 = self.triangles[ti as usize].v[(k + 1) % 3];
                let ip1 = self.triangles[ti as usize].v[(k + 2) % 3];
                let op2 = Self::orient2d(
                    ax,
                    ay,
                    bx,
                    by,
                    self.vertices[ip2 as usize].x,
                    self.vertices[ip2 as usize].y,
                );
                let op1 = Self::orient2d(
                    ax,
                    ay,
                    bx,
                    by,
                    self.vertices[ip1 as usize].x,
                    self.vertices[ip1 as usize].y,
                );
                if op2 < 0.0 && op1 >= 0.0 {
                    ivl = ip1;
                    ivr = ip2;
                    it = ti;
                    break;
                }
                let next = self.triangles[ti as usize].adj[(k + 1) % 3];
                if next < 0
                    || !((next as usize) < self.triangles.len())
                    || !self.triangles[next as usize].alive
                {
                    break;
                }
                if next == start_ti {
                    break;
                }
                ti = next;
            }
        }
        if it < 0 {
            return;
        }
        let mut poly_l: Vec<i32> = vec![v0, ivl];
        let mut poly_r: Vec<i32> = vec![v0, ivr];
        let mut intersected: Vec<i32> = vec![it];
        let mut iv = v0;
        let mut cur_it = it;
        let guard = self.triangles.len() as i32 * 2 + 8;
        let mut g = 0;
        let tri_has = |ti: i32, v: i32| -> bool {
            self.triangles[ti as usize].v[0] == v
                || self.triangles[ti as usize].v[1] == v
                || self.triangles[ti as usize].v[2] == v
        };
        while !tri_has(cur_it, v1) && g < guard {
            g += 1;
            let mut k_iv = -1i32;
            for i in 0..3 {
                if self.triangles[cur_it as usize].v[i] == iv {
                    k_iv = i as i32;
                    break;
                }
            }
            if k_iv < 0 {
                break;
            }
            let i_topo = self.triangles[cur_it as usize].adj[k_iv as usize];
            if i_topo < 0 || !self.triangles[i_topo as usize].alive {
                break;
            }
            let mut i_vopo = -1i32;
            for k in 0..3 {
                if self.triangles[i_topo as usize].adj[k] == cur_it {
                    i_vopo = self.triangles[i_topo as usize].v[k];
                    break;
                }
            }
            if i_vopo < 0 {
                break;
            }
            let o = Self::orient2d(
                ax,
                ay,
                bx,
                by,
                self.vertices[i_vopo as usize].x,
                self.vertices[i_vopo as usize].y,
            );
            if o < 0.0 {
                if i_vopo != v1 {
                    poly_r.push(i_vopo);
                }
                iv = ivr;
                ivr = i_vopo;
            } else {
                if i_vopo != v1 {
                    poly_l.push(i_vopo);
                }
                iv = ivl;
                ivl = i_vopo;
            }
            intersected.push(i_topo);
            cur_it = i_topo;
        }
        poly_l.push(v1);
        poly_r.push(v1);
        let first_new = self.triangles.len() as i32;
        for &ti in &intersected {
            self.unregister_edges(ti);
            self.triangles[ti as usize].alive = false;
        }
        let mut new_tris: Vec<(i32, i32, i32)> = Vec::new();
        {
            let apex = v1;
            for i in 0..poly_l.len().saturating_sub(2) {
                new_tris.push((apex, poly_l[i + 1], poly_l[i]));
            }
        }
        {
            let apex = v0;
            for i in 1..poly_r.len().saturating_sub(1) {
                new_tris.push((apex, poly_r[i], poly_r[i + 1]));
            }
        }
        for (pa, pb, pc) in new_tris {
            let o = Self::orient2d(
                self.vertices[pa as usize].x,
                self.vertices[pa as usize].y,
                self.vertices[pb as usize].x,
                self.vertices[pb as usize].y,
                self.vertices[pc as usize].x,
                self.vertices[pc as usize].y,
            );
            if o.abs() < 1e-20 {
                continue;
            }
            let new_ti = self.triangles.len() as i32;
            let (vb, vc) = if o > 0.0 { (pb, pc) } else { (pc, pb) };
            self.triangles.push(Triangle {
                v: [pa, vb, vc],
                adj: [-1, -1, -1],
                constrained: [false; 3],
                alive: true,
            });
            self.register_edges(new_ti);
        }
        for new_ti in (first_new as usize)..self.triangles.len() {
            if !self.triangles[new_ti].alive {
                continue;
            }
            for k in 0..3 {
                let nb = self.triangles[new_ti].adj[k];
                if nb < 0 || nb >= first_new || !self.triangles[nb as usize].alive {
                    continue;
                }
                for kk in 0..3 {
                    if self.triangles[nb as usize].adj[kk] == new_ti as i32
                        && self.triangles[nb as usize].constrained[kk]
                    {
                        self.triangles[new_ti].constrained[k] = true;
                        break;
                    }
                }
            }
        }
        for ti in 0..self.triangles.len() {
            if !self.triangles[ti].alive {
                continue;
            }
            for k in 0..3 {
                let e0 = self.triangles[ti].v[(k + 1) % 3];
                let e1 = self.triangles[ti].v[(k + 2) % 3];
                if (e0 == v0 && e1 == v1) || (e0 == v1 && e1 == v0) {
                    self.triangles[ti].constrained[k] = true;
                }
            }
        }
    }

    fn cleanup(&mut self) {
        let sv = self.super_v;
        for ti in 0..self.triangles.len() {
            if !self.triangles[ti].alive {
                continue;
            }
            for k in 0..3 {
                if self.triangles[ti].v[k] == sv[0]
                    || self.triangles[ti].v[k] == sv[1]
                    || self.triangles[ti].v[k] == sv[2]
                {
                    self.unregister_edges(ti as i32);
                    self.triangles[ti].alive = false;
                    break;
                }
            }
        }
        self.last_found = 0;
        for i in 0..self.triangles.len() {
            if self.triangles[i].alive {
                self.last_found = i as i32;
                break;
            }
        }
    }

    fn get_triangles(&self) -> Vec<[i32; 3]> {
        let mut result = Vec::new();
        for t in &self.triangles {
            if !t.alive {
                continue;
            }
            let [a, b, c] = [t.v[0], t.v[1], t.v[2]];
            let o = Self::orient2d(
                self.vertices[a as usize].x,
                self.vertices[a as usize].y,
                self.vertices[b as usize].x,
                self.vertices[b as usize].y,
                self.vertices[c as usize].x,
                self.vertices[c as usize].y,
            );
            result.push(if o > 0.0 { [a, b, c] } else { [a, c, b] });
        }
        result
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TrimLoops
// ═══════════════════════════════════════════════════════════════════════════

/// Trim wires of one face as UV polygons, optional 3D points per loop vertex shared bit for bit with the neighbouring face, and interior UV seeds
#[derive(Debug, Clone, Default)]
pub struct TrimLoops {
    pub uv: Vec<Vec<Point>>,
    pub xyz: Vec<Vec<Point>>,
    pub interior_uv: Vec<Point>,
}

// ═══════════════════════════════════════════════════════════════════════════
// NurbsSurfaceTrimmed
// ═══════════════════════════════════════════════════════════════════════════

/// A NURBS surface bounded by a closed outer loop and optional inner loops in its UV space
#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(tag = "type", rename = "NurbsSurfaceTrimmed")]
pub struct NurbsSurfaceTrimmed {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub width: f64,
    pub surfacecolor: Color,
    #[serde(rename = "surface")]
    pub m_surface: NurbsSurface,
    #[serde(rename = "outer_loop")]
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(default)]
    pub m_outer_loop: Option<NurbsCurve>,
    #[serde(rename = "inner_loops")]
    #[serde(default)]
    pub m_inner_loops: Vec<NurbsCurve>,
    // SESSION_VIEWER
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cut_q0: Option<Point>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cut_n: Option<Vector>,
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
            m_surface: NurbsSurface::default(),
            m_outer_loop: None,
            m_inner_loops: Vec::new(),
            cut_q0: None,
            cut_n: None,
            cut_planes: Vec::new(),
        }
    }

    /// Copy (new guid, same data)
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = std::sync::OnceLock::new();
        copy
    }

    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Surface with a closed outer loop given in its UV parameter space
    pub fn create(surface: &NurbsSurface, outer_loop: &NurbsCurve) -> Self {
        let mut ts = Self::new();
        ts.m_surface = surface.duplicate();
        ts.m_outer_loop = Some(outer_loop.duplicate());
        ts
    }

    /// Planar surface fitted to a closed 3D boundary, the boundary projected as the outer loop
    pub fn create_planar(boundary: &NurbsCurve) -> Self {
        let srf = Primitives::create_planar(boundary);
        if !srf.is_valid() {
            return Self::new();
        }

        let p00 = srf.get_cv(0, 0).unwrap_or_default();
        let p10 = srf.get_cv(1, 0).unwrap_or_default();
        let p01 = srf.get_cv(0, 1).unwrap_or_default();
        let u_axis = Vector::new(p10[0] - p00[0], p10[1] - p00[1], p10[2] - p00[2]);
        let v_axis = Vector::new(p01[0] - p00[0], p01[1] - p00[1], p01[2] - p00[2]);
        let u_len2 = u_axis[0] * u_axis[0] + u_axis[1] * u_axis[1] + u_axis[2] * u_axis[2];
        let v_len2 = v_axis[0] * v_axis[0] + v_axis[1] * v_axis[1] + v_axis[2] * v_axis[2];
        if u_len2 < 1e-28 || v_len2 < 1e-28 {
            return Self::new();
        }

        let project_to_uv = |pt: &Point| -> Point {
            let dx = pt[0] - p00[0];
            let dy = pt[1] - p00[1];
            let dz = pt[2] - p00[2];
            let nu = (dx * u_axis[0] + dy * u_axis[1] + dz * u_axis[2]) / u_len2;
            let nv = (dx * v_axis[0] + dy * v_axis[1] + dz * v_axis[2]) / v_len2;
            Point::new(nu, nv, 0.0)
        };

        let mut uv_pts: Vec<Point> = Vec::new();
        if boundary.degree() <= 1 {
            for i in 0..boundary.cv_count() {
                uv_pts.push(project_to_uv(&boundary.get_cv(i).unwrap_or_default()));
            }
        } else {
            let spans = boundary.get_span_vector();
            for si in 0..spans.len().saturating_sub(1) {
                let n_sub = 10;
                for k in 0..=n_sub {
                    let t = spans[si] + (spans[si + 1] - spans[si]) * k as f64 / n_sub as f64;
                    let uv = project_to_uv(&boundary.point_at(t));
                    if uv_pts.is_empty()
                        || (uv[0] - uv_pts[uv_pts.len() - 1][0])
                            * (uv[0] - uv_pts[uv_pts.len() - 1][0])
                            + (uv[1] - uv_pts[uv_pts.len() - 1][1])
                                * (uv[1] - uv_pts[uv_pts.len() - 1][1])
                            > 1e-24
                    {
                        uv_pts.push(uv);
                    }
                }
            }
        }

        let mut ts = Self::new();
        ts.m_surface = srf;
        if uv_pts.len() >= 3 {
            ts.m_outer_loop = Some(NurbsCurve::create(false, 1, &uv_pts));
        }
        ts
    }

    /// One trimmed face per region of the UV domain carved by the pcurves (x=u, y=v, z=0); dangling cutters are discarded
    pub fn split_by_uv_curves(
        srf: &NurbsSurface,
        pcurves: &[NurbsCurve],
        tolerance: f64,
    ) -> Vec<NurbsSurfaceTrimmed> {
        if !srf.is_valid() {
            return Vec::new();
        }

        let is_boundary = |cidx: i32| -> bool { cidx < 0 };

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
        let pmid = srf.point_at(mu, mv).unwrap_or_default();
        let uv_to_3d_u = pmid.distance(
            &srf.point_at((mu + du).min(u1), mv).unwrap_or_default(),
            None,
        ) / du;
        let uv_to_3d_v = pmid.distance(
            &srf.point_at(mu, (mv + dv).min(v1)).unwrap_or_default(),
            None,
        ) / dv;
        let mut uv_to_3d = uv_to_3d_u.max(uv_to_3d_v);
        if uv_to_3d < 1e-10 {
            uv_to_3d = 1.0;
        }

        let snap_uv = if tolerance > 0.0 {
            (tolerance / uv_to_3d).max(1e-9)
        } else {
            range_u.min(range_v) * 1e-7
        };

        let samp_tol = range_u.max(range_v) * 2e-5;
        struct UVPoly {
            cidx: i32,
            pts: Vec<[f64; 2]>,
            ts: Vec<f64>,
        }
        let mut polylines: Vec<UVPoly> = Vec::new();

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
            let n = (crv.cv_count() * 4).clamp(16, 2048);
            for i in 0..=n {
                let t = ct0 + (ct1 - ct0) * i as f64 / n as f64;
                let p = crv.point_at(t);
                entries.push([t, p[0], p[1]]);
            }
            let mut depth = 0;
            while depth < 6 {
                let mut inserted = 0;
                let mut i = 0;
                while i + 1 < entries.len() {
                    let a = entries[i];
                    let b = entries[i + 1];
                    let tm = (a[0] + b[0]) * 0.5;
                    let pm = crv.point_at(tm);
                    let exu = b[1] - a[1];
                    let exv = b[2] - a[2];
                    let l2 = exu * exu + exv * exv;
                    let dev = if l2 > 1e-30 {
                        let s = ((pm[0] - a[1]) * exu + (pm[1] - a[2]) * exv) / l2;
                        let cx = a[1] + s * exu;
                        let cy = a[2] + s * exv;
                        (pm[0] - cx).hypot(pm[1] - cy)
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
                if !pts.is_empty()
                    && (p[0] - pts[pts.len() - 1][0]).abs() < 1e-15
                    && (p[1] - pts[pts.len() - 1][1]).abs() < 1e-15
                {
                    continue;
                }
                pts.push(p);
                ts.push(e[0]);
            }
            if pts.len() < 2 {
                continue;
            }
            let mut on_u0 = true;
            let mut on_u1 = true;
            let mut on_v0 = true;
            let mut on_v1 = true;
            for p in &pts {
                if (p[0] - u0).abs() >= snap_uv {
                    on_u0 = false;
                }
                if (p[0] - u1).abs() >= snap_uv {
                    on_u1 = false;
                }
                if (p[1] - v0).abs() >= snap_uv {
                    on_v0 = false;
                }
                if (p[1] - v1).abs() >= snap_uv {
                    on_v1 = false;
                }
            }
            if on_u0 || on_u1 || on_v0 || on_v1 {
                continue;
            }
            polylines.push(UVPoly {
                cidx: cidx as i32,
                pts,
                ts,
            });
        }

        polylines.push(UVPoly {
            cidx: -1,
            pts: vec![[u0, v0], [u1, v0]],
            ts: vec![u0, u1],
        });
        polylines.push(UVPoly {
            cidx: -2,
            pts: vec![[u1, v0], [u1, v1]],
            ts: vec![v0, v1],
        });
        polylines.push(UVPoly {
            cidx: -3,
            pts: vec![[u1, v1], [u0, v1]],
            ts: vec![u1, u0],
        });
        polylines.push(UVPoly {
            cidx: -4,
            pts: vec![[u0, v1], [u0, v0]],
            ts: vec![v1, v0],
        });

        let min_ext = (snap_uv * 8.0).max(range_u.min(range_v) * 1e-5);
        let mut kept: Vec<UVPoly> = Vec::new();
        for poly in polylines {
            let mut ext = 0.0;
            for k in 1..poly.pts.len() {
                ext += (poly.pts[k][0] - poly.pts[k - 1][0])
                    .hypot(poly.pts[k][1] - poly.pts[k - 1][1]);
            }
            if is_boundary(poly.cidx) || ext >= min_ext {
                kept.push(poly);
            }
        }
        let polylines = kept;

        fn seg_seg(
            p1: &[f64; 2],
            p2: &[f64; 2],
            p3: &[f64; 2],
            p4: &[f64; 2],
        ) -> Option<(f64, f64)> {
            let d1u = p2[0] - p1[0];
            let d1v = p2[1] - p1[1];
            let d2u = p4[0] - p3[0];
            let d2v = p4[1] - p3[1];
            let den = d1u * d2v - d1v * d2u;
            if den.abs() < 1e-20 {
                return None;
            }
            let s = ((p3[0] - p1[0]) * d2v - (p3[1] - p1[1]) * d2u) / den;
            let t = ((p3[0] - p1[0]) * d1v - (p3[1] - p1[1]) * d1u) / den;
            if -1e-12 <= s && s <= 1.0 + 1e-12 && -1e-12 <= t && t <= 1.0 + 1e-12 {
                return Some((s, t));
            }
            None
        }

        let newton_cc =
            |ca: &NurbsCurve, mut ta: f64, cb: &NurbsCurve, mut tb: f64| -> (f64, f64) {
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
                    let adom = ca.domain();
                    let bdom = cb.domain();
                    ta = ta.max(adom.0).min(adom.1);
                    tb = tb.max(bdom.0).min(bdom.1);
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
                let mut aminu = a_poly.pts[0][0];
                let mut amaxu = a_poly.pts[0][0];
                let mut aminv = a_poly.pts[0][1];
                let mut amaxv = a_poly.pts[0][1];
                for p in &a_poly.pts {
                    aminu = aminu.min(p[0]);
                    amaxu = amaxu.max(p[0]);
                    aminv = aminv.min(p[1]);
                    amaxv = amaxv.max(p[1]);
                }
                aminu -= snap_uv;
                amaxu += snap_uv;
                aminv -= snap_uv;
                amaxv += snap_uv;
                let mut bminu = b_poly.pts[0][0];
                let mut bmaxu = b_poly.pts[0][0];
                let mut bminv = b_poly.pts[0][1];
                let mut bmaxv = b_poly.pts[0][1];
                for p in &b_poly.pts {
                    bminu = bminu.min(p[0]);
                    bmaxu = bmaxu.max(p[0]);
                    bminv = bminv.min(p[1]);
                    bmaxv = bmaxv.max(p[1]);
                }
                if bminu > amaxu || bmaxu < aminu || bminv > amaxv || bmaxv < aminv {
                    continue;
                }
                for ia in 0..a_poly.pts.len() - 1 {
                    for ib in 0..b_poly.pts.len() - 1 {
                        let Some((s, t)) = seg_seg(
                            &a_poly.pts[ia],
                            &a_poly.pts[ia + 1],
                            &b_poly.pts[ib],
                            &b_poly.pts[ib + 1],
                        ) else {
                            continue;
                        };
                        let mut ta = a_poly.ts[ia] + (a_poly.ts[ia + 1] - a_poly.ts[ia]) * s;
                        let mut tb = b_poly.ts[ib] + (b_poly.ts[ib + 1] - b_poly.ts[ib]) * t;
                        let mut hu =
                            a_poly.pts[ia][0] + (a_poly.pts[ia + 1][0] - a_poly.pts[ia][0]) * s;
                        let mut hv =
                            a_poly.pts[ia][1] + (a_poly.pts[ia + 1][1] - a_poly.pts[ia][1]) * s;
                        if a_poly.cidx >= 0 && b_poly.cidx >= 0 {
                            (ta, tb) = newton_cc(
                                &pcurves[a_poly.cidx as usize],
                                ta,
                                &pcurves[b_poly.cidx as usize],
                                tb,
                            );
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
                        splits
                            .entry((pi, ia))
                            .or_default()
                            .push((s, hp[0], hp[1], ta));
                        splits
                            .entry((pj, ib))
                            .or_default()
                            .push((t, hp[0], hp[1], tb));
                    }
                }
            }
        }

        let mut cell_map: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
        let mut verts: Vec<[f64; 2]> = Vec::new();

        let mut vert_id = |p: [f64; 2]| -> usize {
            let ci = (p[0] / snap_uv).floor() as i64;
            let cj = (p[1] / snap_uv).floor() as i64;
            for di in -1i64..=1 {
                for dj in -1i64..=1 {
                    let Some(bucket) = cell_map.get(&(ci + di, cj + dj)) else {
                        continue;
                    };
                    for &vk in bucket {
                        let q = verts[vk];
                        if (q[0] - p[0]).hypot(q[1] - p[1]) <= snap_uv {
                            return vk;
                        }
                    }
                }
            }
            let vk = verts.len();
            verts.push([p[0], p[1]]);
            cell_map.entry((ci, cj)).or_default().push(vk);
            vk
        };

        struct SplitEdge {
            a: usize,
            b: usize,
            cidx: i32,
            ta: f64,
            tb: f64,
        }
        let mut edges: Vec<SplitEdge> = Vec::new();

        for (pi, poly) in polylines.iter().enumerate() {
            let mut chain: Vec<(usize, f64)> = Vec::new();
            for i in 0..poly.pts.len() {
                chain.push((vert_id(poly.pts[i]), poly.ts[i]));
                if i + 1 < poly.pts.len() {
                    if let Some(sp) = splits.get(&(pi, i)) {
                        let mut evs = sp.clone();
                        evs.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
                        for ev in evs {
                            chain.push((vert_id([ev.1, ev.2]), ev.3));
                        }
                    }
                }
            }
            for i in 0..chain.len().saturating_sub(1) {
                let (a, ta) = chain[i];
                let (b, tb) = chain[i + 1];
                if a == b {
                    continue;
                }
                edges.push(SplitEdge {
                    a,
                    b,
                    cidx: poly.cidx,
                    ta,
                    tb,
                });
            }
        }

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
                if degree.get(&e.a).copied().unwrap_or(0) == 1
                    || degree.get(&e.b).copied().unwrap_or(0) == 1
                {
                    alive[ei] = false;
                    changed = true;
                }
            }
        }

        let mut live_edges: Vec<&SplitEdge> = Vec::new();
        for (ei, e) in edges.iter().enumerate() {
            if alive[ei] {
                live_edges.push(e);
            }
        }
        if live_edges.is_empty() {
            return Vec::new();
        }

        struct HalfEdge {
            tail: usize,
            head: usize,
            eidx: usize,
            fwd: bool,
        }
        let mut hes: Vec<HalfEdge> = Vec::new();
        for (ei, e) in live_edges.iter().enumerate() {
            hes.push(HalfEdge {
                tail: e.a,
                head: e.b,
                eidx: ei,
                fwd: true,
            });
            hes.push(HalfEdge {
                tail: e.b,
                head: e.a,
                eidx: ei,
                fwd: false,
            });
        }
        let mut out_map: Vec<Vec<usize>> = vec![Vec::new(); verts.len()];
        for (hi, he) in hes.iter().enumerate() {
            out_map[he.tail].push(hi);
        }
        for vid in 0..out_map.len() {
            out_map[vid].sort_by(|&ha, &hb| {
                let aa = (verts[hes[ha].head][1] - verts[vid][1])
                    .atan2(verts[hes[ha].head][0] - verts[vid][0]);
                let ab = (verts[hes[hb].head][1] - verts[vid][1])
                    .atan2(verts[hes[hb].head][0] - verts[vid][0]);
                aa.partial_cmp(&ab).unwrap_or(std::cmp::Ordering::Equal)
            });
        }

        let mut next_he = vec![usize::MAX; hes.len()];
        for outs in &out_map {
            for (pos, &hi) in outs.iter().enumerate() {
                let tw = hi ^ 1;
                let nxt = outs[(pos + outs.len() - 1) % outs.len()];
                next_he[tw] = nxt;
            }
        }

        let mut visited = vec![false; hes.len()];
        let mut faces: Vec<Vec<usize>> = Vec::new();
        for hi in 0..hes.len() {
            if visited[hi] {
                continue;
            }
            let mut cycle = Vec::new();
            let mut cur = hi;
            while cur != usize::MAX && !visited[cur] {
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
                let a = verts[hes[hi].tail];
                let b = verts[hes[hi].head];
                s += a[0] * b[1] - b[0] * a[1];
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
                let a = verts[hes[hi].tail];
                let b = verts[hes[hi].head];
                if (a[1] > p[1]) != (b[1] > p[1])
                    && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
                {
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
                let mut touches_border = false;
                for &hi in &cycle {
                    if border_vids.contains(&hes[hi].tail) {
                        touches_border = true;
                        break;
                    }
                }
                if !touches_border {
                    neg_faces.push(cycle);
                }
            }
        }

        let mut holes_of: Vec<Vec<Vec<usize>>> = vec![Vec::new(); pos_faces.len()];
        for cycle in &neg_faces {
            let sample = verts[hes[cycle[0]].tail];
            let mut best: i32 = -1;
            let mut best_area = f64::INFINITY;
            for (fi, (fc, area)) in pos_faces.iter().enumerate() {
                if *area < best_area && point_in_cycle(sample, fc) {
                    let mut hole_vids: HashSet<usize> = HashSet::new();
                    let mut face_vids: HashSet<usize> = HashSet::new();
                    for &hi in cycle {
                        hole_vids.insert(hes[hi].tail);
                    }
                    for &hi in fc {
                        face_vids.insert(hes[hi].tail);
                    }
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

        let cycle_to_segments = |cycle: &Vec<usize>| -> Vec<NurbsCurve> {
            struct Run {
                cidx: i32,
                va: usize,
                vb: usize,
                ta: f64,
                tb: f64,
            }
            let mut runs: Vec<Run> = Vec::new();
            for &hi in cycle {
                let he = &hes[hi];
                let e = live_edges[he.eidx];
                let ta = if he.fwd { e.ta } else { e.tb };
                let tb = if he.fwd { e.tb } else { e.ta };
                let merged = match runs.last_mut() {
                    Some(last) if last.cidx == e.cidx && last.vb == he.tail => {
                        last.vb = he.head;
                        last.tb = tb;
                        true
                    }
                    _ => false,
                };
                if !merged {
                    runs.push(Run {
                        cidx: e.cidx,
                        va: he.tail,
                        vb: he.head,
                        ta,
                        tb,
                    });
                }
            }
            let mut pieces: Vec<NurbsCurve> = Vec::new();
            for run in &runs {
                let mut made = false;
                if run.cidx >= 0 {
                    let crv = &pcurves[run.cidx as usize];
                    let (c0, c1) = crv.domain();
                    let lo = c0.max(run.ta.min(run.tb));
                    let hi_ = c1.min(run.ta.max(run.tb));
                    let mut piece = crv.duplicate();
                    let mut piece_ok = true;
                    if hi_ - lo < (c1 - c0) - 1e-12 && hi_ - lo > 1e-14 {
                        if !piece.trim(lo, hi_) {
                            piece_ok = false;
                        }
                    } else if hi_ - lo <= 1e-14 && !(run.va == run.vb && piece.is_closed()) {
                        piece_ok = false;
                    }
                    if piece_ok && piece.is_valid() {
                        if run.ta > run.tb {
                            piece.reverse();
                        }
                        pieces.push(piece);
                        made = true;
                    }
                }
                if !made {
                    let pa = verts[run.va];
                    let pb = verts[run.vb];
                    if (pb[0] - pa[0]).hypot(pb[1] - pa[1]) > 1e-14 {
                        let seg_pts =
                            vec![Point::new(pa[0], pa[1], 0.0), Point::new(pb[0], pb[1], 0.0)];
                        pieces.push(NurbsCurve::create(false, 1, &seg_pts));
                    }
                }
            }
            pieces
        };

        let cycle_to_loop = |cycle: &Vec<usize>| -> NurbsCurve {
            let pieces = cycle_to_segments(cycle);
            if pieces.is_empty() {
                return NurbsCurve::default();
            }
            let join_tol = snap_uv * 4.0;
            let mut joined = NurbsCurve::join(&pieces, Some(join_tol));
            if joined.len() == 1 && joined[0].is_valid() {
                let j = &mut joined[0];
                if !j.is_closed()
                    && j.point_at_start().distance(&j.point_at_end(), None) <= join_tol
                {
                    if let Some((x, y, z, _w)) = j.get_cv_4d(0) {
                        if let Some((_xe, _ye, _ze, we)) = j.get_cv_4d(j.cv_count() - 1) {
                            j.set_cv_4d(j.cv_count() - 1, x, y, z, we);
                        }
                    }
                }
                if j.is_closed() {
                    return joined.remove(0);
                }
            }
            let mut loop_pts: Vec<Point> = Vec::new();
            for &hi in cycle {
                let a = verts[hes[hi].tail];
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
                s += prev[0] * p[1] - p[0] * prev[1];
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
                if !hole.is_valid() {
                    continue;
                }
                if loop_signed_area(&hole) > 0.0 {
                    hole.reverse();
                }
                ts.add_inner_loop(hole);
            }
            result.push(ts);
        }
        result
    }

    /// One trimmed face per non-empty region carved by the planes (all 2^K sign combinations)
    pub fn split_by_planes(
        srf: &NurbsSurface,
        planes: &[(Point, Vector)],
    ) -> Vec<NurbsSurfaceTrimmed> {
        let mut out = Vec::new();
        let k = planes.len();
        if k == 0 || k > 16 {
            return out;
        }
        for mask in 0u32..(1u32 << k) {
            let mut cp: Vec<(Point, Vector)> = Vec::new();
            for i in 0..k {
                let q = &planes[i].0;
                let n = &planes[i].1;
                let flip = ((mask >> i) & 1) == 1;
                let nn = if flip {
                    Vector::new(-n[0], -n[1], -n[2])
                } else {
                    Vector::new(n[0], n[1], n[2])
                };
                cp.push((q.clone(), nn));
            }
            let mut ts = NurbsSurfaceTrimmed::new();
            ts.m_surface = srf.clone();
            let m = ts.mesh_by_planes(&cp, 20.0, 0.01);
            if m.number_of_faces() > 0 {
                // SESSION_VIEWER
                ts.cut_planes = cp;
                out.push(ts);
            }
        }
        out
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn surface(&self) -> &NurbsSurface {
        &self.m_surface
    }

    pub fn get_outer_loop(&self) -> Option<&NurbsCurve> {
        self.m_outer_loop.as_ref()
    }

    pub fn set_outer_loop(&mut self, loop_crv: NurbsCurve) {
        self.m_outer_loop = Some(loop_crv);
    }

    /// True when the outer loop is a valid curve
    pub fn is_trimmed(&self) -> bool {
        self.m_outer_loop.as_ref().is_some_and(|c| c.is_valid())
    }

    /// True when the underlying surface is valid
    pub fn is_valid(&self) -> bool {
        self.m_surface.is_valid()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Inner loops
    // ═══════════════════════════════════════════════════════════════════════════

    /// Hole given directly as a closed 2D curve in UV space
    pub fn add_inner_loop(&mut self, loop_2d: NurbsCurve) {
        self.m_inner_loops.push(loop_2d);
    }

    /// Hole from a 3D curve pulled onto the surface and normalized into [0,1]^2
    pub fn add_hole(&mut self, curve_3d: &NurbsCurve) {
        let dom = curve_3d.domain();
        let sdom_u = self.m_surface.domain(0).unwrap_or((0.0, 1.0));
        let sdom_v = self.m_surface.domain(1).unwrap_or((0.0, 1.0));
        let range_u = sdom_u.1 - sdom_u.0;
        let range_v = sdom_v.1 - sdom_v.0;
        let n_samples = (curve_3d.cv_count() * 4).clamp(32, 2048);
        let mut uv_pts = Vec::new();
        for i in 0..n_samples {
            let t = dom.0 + (dom.1 - dom.0) * i as f64 / n_samples as f64;
            let pt3d = curve_3d.point_at(t);
            let (u, v, _dist) = Closest::surface_point(&self.m_surface, &pt3d, 0.0, 0.0, 0.0, 0.0);
            let nu = (u - sdom_u.0) / range_u;
            let nv = (v - sdom_v.0) / range_v;
            uv_pts.push(Point::new(nu, nv, 0.0));
        }
        if uv_pts.len() >= 3 {
            self.m_inner_loops
                .push(NurbsCurve::create(true, 1, &uv_pts));
        }
    }

    pub fn add_holes(&mut self, curves_3d: &[NurbsCurve]) {
        for crv in curves_3d {
            self.add_hole(crv);
        }
    }

    pub fn get_inner_loop(&self, index: usize) -> &NurbsCurve {
        &self.m_inner_loops[index]
    }

    pub fn inner_loop_count(&self) -> usize {
        self.m_inner_loops.len()
    }

    pub fn clear_inner_loops(&mut self) {
        self.m_inner_loops.clear();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn point_at(&self, u: f64, v: f64) -> Option<Point> {
        self.m_surface.point_at(u, v)
    }

    pub fn normal_at(&self, u: f64, v: f64) -> Vector {
        self.m_surface.normal_at(u, v)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Meshing
    // ═══════════════════════════════════════════════════════════════════════════

    /// mesh_q at 20 degrees and a chord factor of 0.005
    pub fn mesh(&self) -> Mesh {
        self.mesh_q(20.0, 0.005)
    }

    /// Diagonal of the control-point box, the scale every deflection tolerance is a fraction of
    fn bbox_diagonal(&self) -> f64 {
        let mut bmin = [1e30f64; 3];
        let mut bmax = [-1e30f64; 3];
        for i in 0..self.m_surface.cv_count(0) {
            for j in 0..self.m_surface.cv_count(1) {
                let p = self.m_surface.get_cv(i, j).unwrap_or_default();
                for k in 0..3 {
                    if p[k] < bmin[k] {
                        bmin[k] = p[k];
                    }
                    if p[k] > bmax[k] {
                        bmax[k] = p[k];
                    }
                }
            }
        }
        let bbox_diag = ((bmax[0] - bmin[0]) * (bmax[0] - bmin[0])
            + (bmax[1] - bmin[1]) * (bmax[1] - bmin[1])
            + (bmax[2] - bmin[2]) * (bmax[2] - bmin[2]))
            .sqrt();
        if bbox_diag < 1e-12 {
            1.0
        } else {
            bbox_diag
        }
    }

    /// Deflection-refined constrained Delaunay of the trim loops: angular bound in degrees, chord factor as a fraction of the bbox diagonal
    pub fn mesh_q(&self, max_angle_deg: f64, chord_factor: f64) -> Mesh {
        if !self.is_trimmed() {
            return self.m_surface.mesh();
        }

        let deflection = self.bbox_diagonal() * chord_factor;

        let eval3 = |u: f64, v: f64| -> [f64; 3] {
            let p = self.m_surface.point_at(u, v).unwrap_or_default();
            [p[0], p[1], p[2]]
        };

        let disc_loop = |crv: &NurbsCurve| -> Vec<Point> {
            let mut raw: Vec<Point> = Vec::new();
            if crv.degree() <= 1 && !crv.is_rational() {
                for i in 0..crv.cv_count() {
                    raw.push(crv.get_cv(i).unwrap_or_default());
                }
            } else {
                let n = (crv.cv_count() * 4).clamp(16, 2048);
                let (sampled, _params) = crv.divide_by_count(n, true);
                raw = sampled;
            }
            while raw.len() > 1 {
                let dx = raw[0][0] - raw[raw.len() - 1][0];
                let dy = raw[0][1] - raw[raw.len() - 1][1];
                if dx * dx + dy * dy < 1e-20 {
                    raw.pop();
                } else {
                    break;
                }
            }
            if raw.len() < 2 {
                return raw;
            }
            let mut out: Vec<Point> = Vec::with_capacity(raw.len() * 2);
            for i in 0..raw.len() {
                let mut stack: Vec<(Point, Point, i32)> =
                    vec![(raw[i].clone(), raw[(i + 1) % raw.len()].clone(), 0)];
                while let Some((a, b, depth)) = stack.pop() {
                    let mu = (a[0] + b[0]) * 0.5;
                    let mv = (a[1] + b[1]) * 0.5;
                    let pa = eval3(a[0], a[1]);
                    let pb = eval3(b[0], b[1]);
                    let pm = eval3(mu, mv);
                    let ex = pb[0] - pa[0];
                    let ey = pb[1] - pa[1];
                    let ez = pb[2] - pa[2];
                    let l2 = ex * ex + ey * ey + ez * ez;
                    let dev = if l2 > 1e-30 {
                        let t =
                            ((pm[0] - pa[0]) * ex + (pm[1] - pa[1]) * ey + (pm[2] - pa[2]) * ez)
                                / l2;
                        let cx = pa[0] + t * ex;
                        let cy = pa[1] + t * ey;
                        let cz = pa[2] + t * ez;
                        ((pm[0] - cx) * (pm[0] - cx)
                            + (pm[1] - cy) * (pm[1] - cy)
                            + (pm[2] - cz) * (pm[2] - cz))
                            .sqrt()
                    } else {
                        ((pm[0] - pa[0]) * (pm[0] - pa[0])
                            + (pm[1] - pa[1]) * (pm[1] - pa[1])
                            + (pm[2] - pa[2]) * (pm[2] - pa[2]))
                            .sqrt()
                    };
                    if dev > deflection && depth < 6 {
                        stack.push((Point::new(mu, mv, 0.0), b, depth + 1));
                        stack.push((a, Point::new(mu, mv, 0.0), depth + 1));
                    } else {
                        out.push(a);
                    }
                }
            }
            out
        };

        let mut loops = TrimLoops::default();
        let Some(outer) = self.m_outer_loop.as_ref() else {
            return self.m_surface.mesh();
        };
        loops.uv.push(disc_loop(outer));
        for inner in &self.m_inner_loops {
            loops.uv.push(disc_loop(inner));
        }
        self.triangulate(&loops, max_angle_deg, chord_factor)
    }

    /// Mesh sampled loops (outer first, then holes) keeping every loop vertex, tagged boundary/{loop}/{sample}; knot crossings add boundary_interval/{loop}/{segment}; empty mesh on invalid input
    pub fn mesh_loops(&self, loops: &TrimLoops, max_angle_deg: f64, chord_factor: f64) -> Mesh {
        if loops.uv.is_empty()
            || !max_angle_deg.is_finite()
            || max_angle_deg <= 0.0
            || !chord_factor.is_finite()
            || chord_factor <= 0.0
            || (!loops.xyz.is_empty() && loops.xyz.len() != loops.uv.len())
        {
            return Mesh::new();
        }
        let mut expected = 0;
        for (li, points) in loops.uv.iter().enumerate() {
            if points.len() < 3 || (!loops.xyz.is_empty() && loops.xyz[li].len() != points.len()) {
                return Mesh::new();
            }
            for point in points {
                if !point[0].is_finite() || !point[1].is_finite() {
                    return Mesh::new();
                }
            }
            if !loops.xyz.is_empty() {
                for point in &loops.xyz[li] {
                    if !point[0].is_finite() || !point[1].is_finite() || !point[2].is_finite() {
                        return Mesh::new();
                    }
                }
            }
            expected += points.len();
        }
        let result = self.triangulate(loops, max_angle_deg, chord_factor);
        let mut actual: HashSet<String> = HashSet::new();
        for vd in result.vertex.values() {
            for name in vd.attributes.keys() {
                if name.starts_with("boundary/") {
                    actual.insert(name.clone());
                }
            }
        }
        if actual.len() == expected {
            result
        } else {
            Mesh::new()
        }
    }

    /// Constrained Delaunay of the loops in UV, refined, trimmed, lifted and welded: the one body mesh_q and mesh_loops share
    fn triangulate(&self, loops: &TrimLoops, max_angle_deg: f64, chord_factor: f64) -> Mesh {
        if loops.uv.is_empty() || loops.uv[0].len() < 3 {
            return self.m_surface.mesh();
        }
        let outer_uv = &loops.uv[0];
        let bbox_diag = self.bbox_diagonal();
        let deflection = bbox_diag * chord_factor;
        let cos_max_angle = (max_angle_deg.clamp(0.1, 179.0) * std::f64::consts::PI / 180.0).cos();

        let eval3 = |u: f64, v: f64| -> [f64; 3] {
            let p = self.m_surface.point_at(u, v).unwrap_or_default();
            [p[0], p[1], p[2]]
        };

        let mut bb_umin = 1e30_f64;
        let mut bb_vmin = 1e30_f64;
        let mut bb_umax = -1e30_f64;
        let mut bb_vmax = -1e30_f64;
        for p in outer_uv {
            if p[0] < bb_umin {
                bb_umin = p[0];
            }
            if p[1] < bb_vmin {
                bb_vmin = p[1];
            }
            if p[0] > bb_umax {
                bb_umax = p[0];
            }
            if p[1] > bb_vmax {
                bb_vmax = p[1];
            }
        }

        let inside_trim = |u: f64, v: f64| -> bool {
            if !point_in_polygon_2d(u, v, outer_uv) {
                return false;
            }
            for li in 1..loops.uv.len() {
                if point_in_polygon_2d(u, v, &loops.uv[li]) {
                    return false;
                }
            }
            true
        };

        let mut crease_knots = [Vec::new(), Vec::new()];
        for dir in 0..2 {
            let Some((start, end)) = self.m_surface.domain(dir) else {
                continue;
            };
            let knots = &self.m_surface.m_nurbsknot[dir];
            for &knot in knots {
                if knot <= start || knot >= end || crease_knots[dir].contains(&knot) {
                    continue;
                }
                if knots.iter().filter(|&&value| value == knot).count()
                    >= self.m_surface.degree(dir)
                {
                    crease_knots[dir].push(knot);
                }
            }
        }
        let mut dt = Delaunay2D::new(bb_umin, bb_vmin, bb_umax, bb_vmax);
        let mut loop_vids: Vec<Vec<i32>> = Vec::new();
        let mut boundary_intervals: HashMap<usize, (usize, usize, f64)> = HashMap::new();
        for (li, pts) in loops.uv.iter().enumerate() {
            let mut vis: Vec<i32> = Vec::new();
            for p in pts {
                vis.push(dt.insert(p[0], p[1]));
            }
            for i in 0..vis.len() {
                let j = (i + 1) % vis.len();
                let mut events = vec![(0.0, vis[i]), (1.0, vis[j])];
                for dir in 0..2 {
                    let delta = pts[j][dir] - pts[i][dir];
                    if delta == 0.0 {
                        continue;
                    }
                    for &knot in &crease_knots[dir] {
                        let t = (knot - pts[i][dir]) / delta;
                        if t <= 0.0 || t >= 1.0 {
                            continue;
                        }
                        let mut uv = [
                            pts[i][0] + t * (pts[j][0] - pts[i][0]),
                            pts[i][1] + t * (pts[j][1] - pts[i][1]),
                        ];
                        uv[dir] = knot;
                        let vi = dt.insert(uv[0], uv[1]);
                        if vi >= 0 {
                            boundary_intervals.insert(vi as usize, (li, i, t));
                        }
                        events.push((t, vi));
                    }
                }
                events.sort_by(|a, b| a.0.total_cmp(&b.0));
                for pair in events.windows(2) {
                    if pair[0].1 >= 0 && pair[1].1 >= 0 && pair[0].1 != pair[1].1 {
                        dt.insert_constraint(pair[0].1, pair[1].1);
                    }
                }
            }
            loop_vids.push(vis);
        }
        for &u in &crease_knots[0] {
            for &v in &crease_knots[1] {
                if inside_trim(u, v) {
                    dt.insert(u, v);
                }
            }
        }
        for dir in 0..2 {
            for &knot in &crease_knots[dir] {
                let mut nodes = Vec::new();
                for (vi, vertex) in dt.vertices.iter().enumerate() {
                    let uv = [vertex.x, vertex.y];
                    if uv[dir] == knot {
                        nodes.push((uv[1 - dir], vi as i32));
                    }
                }
                nodes.sort_by(|a, b| a.0.total_cmp(&b.0));
                for pair in nodes.windows(2) {
                    let mut uv = [knot, knot];
                    uv[1 - dir] = (pair[0].0 + pair[1].0) * 0.5;
                    if inside_trim(uv[0], uv[1]) {
                        dt.insert_constraint(pair[0].1, pair[1].1);
                    }
                }
            }
        }
        for p in &loops.interior_uv {
            if inside_trim(p[0], p[1]) {
                dt.insert(p[0], p[1]);
            }
        }

        const MAX_ITERS: i32 = 8;
        const MAX_VERTS: usize = 200000;
        let iters = MAX_ITERS;
        for _iter in 0..iters {
            let mut to_insert: Vec<[f64; 2]> = Vec::new();
            for tri in &dt.triangles {
                if !tri.alive {
                    continue;
                }
                let a = &dt.vertices[tri.v[0] as usize];
                let b = &dt.vertices[tri.v[1] as usize];
                let c = &dt.vertices[tri.v[2] as usize];
                let cu = (a.x + b.x + c.x) / 3.0;
                let cv = (a.y + b.y + c.y) / 3.0;
                if !inside_trim(cu, cv) {
                    continue;
                }
                let pa = eval3(a.x, a.y);
                let pb = eval3(b.x, b.y);
                let pc = eval3(c.x, c.y);
                let pm = eval3(cu, cv);
                let ux = pb[0] - pa[0];
                let uy = pb[1] - pa[1];
                let uz = pb[2] - pa[2];
                let vx = pc[0] - pa[0];
                let vy = pc[1] - pa[1];
                let vz = pc[2] - pa[2];
                let nx = uy * vz - uz * vy;
                let ny = uz * vx - ux * vz;
                let nz = ux * vy - uy * vx;
                let nl = (nx * nx + ny * ny + nz * nz).sqrt();
                if nl < 1e-30 {
                    continue;
                }
                let dev = (((pm[0] - pa[0]) * nx + (pm[1] - pa[1]) * ny + (pm[2] - pa[2]) * nz)
                    / nl)
                    .abs();
                let mut refine = dev > deflection;
                if !refine {
                    let na =
                        crease_side_normal(&self.m_surface, &crease_knots, [cu, cv], [a.x, a.y]);
                    let nb =
                        crease_side_normal(&self.m_surface, &crease_knots, [cu, cv], [b.x, b.y]);
                    let nc2 =
                        crease_side_normal(&self.m_surface, &crease_knots, [cu, cv], [c.x, c.y]);
                    let d1 = na[0] * nb[0] + na[1] * nb[1] + na[2] * nb[2];
                    let d2 = nb[0] * nc2[0] + nb[1] * nc2[1] + nb[2] * nc2[2];
                    let d3 = na[0] * nc2[0] + na[1] * nc2[1] + na[2] * nc2[2];
                    let mind = d1.min(d2.min(d3));
                    if mind < cos_max_angle {
                        refine = true;
                    }
                }
                if refine {
                    to_insert.push([cu, cv]);
                }
            }
            if to_insert.is_empty() {
                break;
            }
            for uv in &to_insert {
                if dt.vertices.len() >= MAX_VERTS {
                    break;
                }
                dt.insert(uv[0], uv[1]);
            }
            if dt.vertices.len() >= MAX_VERTS {
                break;
            }
        }

        dt.cleanup();
        for ti in 0..dt.triangles.len() {
            if !dt.triangles[ti].alive {
                continue;
            }
            let cu = (dt.vertices[dt.triangles[ti].v[0] as usize].x
                + dt.vertices[dt.triangles[ti].v[1] as usize].x
                + dt.vertices[dt.triangles[ti].v[2] as usize].x)
                / 3.0;
            let cv = (dt.vertices[dt.triangles[ti].v[0] as usize].y
                + dt.vertices[dt.triangles[ti].v[1] as usize].y
                + dt.vertices[dt.triangles[ti].v[2] as usize].y)
                / 3.0;
            if !inside_trim(cu, cv) {
                dt.triangles[ti].alive = false;
            }
        }
        let tris = dt.get_triangles();
        if tris.is_empty() {
            return Mesh::new();
        }
        for tri in &tris {
            for dir in 0..2 {
                let mut low = f64::INFINITY;
                let mut high = f64::NEG_INFINITY;
                for &vi in tri {
                    let value = [dt.vertices[vi as usize].x, dt.vertices[vi as usize].y][dir];
                    low = low.min(value);
                    high = high.max(value);
                }
                for &knot in &crease_knots[dir] {
                    if low < knot && knot < high {
                        return Mesh::new();
                    }
                }
            }
        }

        let nv = dt.vertices.len();
        let mut given: Vec<Option<(usize, usize)>> = vec![None; nv];
        for (li, vids) in loop_vids.iter().enumerate() {
            if li >= loops.xyz.len() {
                break;
            }
            for (k, &vi) in vids.iter().enumerate() {
                if vi >= 0 && k < loops.xyz[li].len() {
                    given[vi as usize] = Some((li, k));
                }
            }
        }

        let mut result = Mesh::new();
        let mut vert_map: Vec<Option<usize>> = vec![None; nv];
        let weld_tol = if loops.xyz.is_empty() {
            bbox_diag * 1e-5
        } else {
            0.0
        };
        let cell = bbox_diag * 1e-5;
        let mut cell_map: HashMap<(i64, i64, i64), Vec<([f64; 3], usize)>> = HashMap::new();
        for &[a, b, c] in &tris {
            for &vi in &[a, b, c] {
                if vert_map[vi as usize].is_none() {
                    let u = dt.vertices[vi as usize].x;
                    let v = dt.vertices[vi as usize].y;
                    let p3d = match given[vi as usize] {
                        Some((li, k)) => loops.xyz[li][k].clone(),
                        None => {
                            if let Some(&(li, k, t)) = boundary_intervals.get(&(vi as usize)) {
                                if let Some(points) = loops.xyz.get(li) {
                                    let a = &points[k];
                                    let b = &points[(k + 1) % points.len()];
                                    Point::new(
                                        a[0] + t * (b[0] - a[0]),
                                        a[1] + t * (b[1] - a[1]),
                                        a[2] + t * (b[2] - a[2]),
                                    )
                                } else {
                                    self.m_surface.point_at(u, v).unwrap_or_default()
                                }
                            } else {
                                self.m_surface.point_at(u, v).unwrap_or_default()
                            }
                        }
                    };
                    let x = p3d[0];
                    let y = p3d[1];
                    let z = p3d[2];
                    let ci = (x / cell).floor() as i64;
                    let cj = (y / cell).floor() as i64;
                    let ck = (z / cell).floor() as i64;
                    let mut found: Option<usize> = None;
                    'scan: for di in -1..=1 {
                        for dj in -1..=1 {
                            for dk in -1..=1 {
                                if let Some(bucket) = cell_map.get(&(ci + di, cj + dj, ck + dk)) {
                                    for &(p, wvk) in bucket {
                                        let dx = p[0] - x;
                                        let dy = p[1] - y;
                                        let dz = p[2] - z;
                                        if dx * dx + dy * dy + dz * dz <= weld_tol * weld_tol {
                                            found = Some(wvk);
                                            break 'scan;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    let vk = match found {
                        Some(wvk) => wvk,
                        None => {
                            let vk = result.add_vertex(p3d, None);
                            cell_map
                                .entry((ci, cj, ck))
                                .or_default()
                                .push(([x, y, z], vk));
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
            if v0 == v1 || v1 == v2 || v2 == v0 {
                continue;
            }
            result.add_face(vec![v0, v1, v2], None);
        }
        let mut fan: HashMap<usize, [f64; 3]> = HashMap::new();
        let mut fkeys: Vec<usize> = result.face.keys().copied().collect();
        fkeys.sort_unstable();
        for fk in fkeys {
            let verts = &result.face[&fk];
            let a = result.vertex[&verts[0]].position();
            let b = result.vertex[&verts[1]].position();
            let c = result.vertex[&verts[2]].position();
            let (e1, e2) = (
                [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
                [c[0] - a[0], c[1] - a[1], c[2] - a[2]],
            );
            let n = [
                e1[1] * e2[2] - e1[2] * e2[1],
                e1[2] * e2[0] - e1[0] * e2[2],
                e1[0] * e2[1] - e1[1] * e2[0],
            ];
            for &vk in verts {
                let acc = fan.entry(vk).or_insert([0.0; 3]);
                acc[0] += n[0];
                acc[1] += n[1];
                acc[2] += n[2];
            }
        }
        for vi in 0..nv {
            if let Some(vk) = vert_map[vi] {
                let u = dt.vertices[vi].x;
                let v = dt.vertices[vi].y;
                let derivatives = self.m_surface.evaluate(u, v, 1);
                let mut nrm = [0.0; 3];
                if derivatives.len() >= 3 {
                    let normal = derivatives[2].cross(&derivatives[1]);
                    nrm = [normal[0], normal[1], normal[2]];
                }
                let nl = (nrm[0] * nrm[0] + nrm[1] * nrm[1] + nrm[2] * nrm[2]).sqrt();
                if nl.is_finite() && nl > 0.0 {
                    nrm = [nrm[0] / nl, nrm[1] / nl, nrm[2] / nl];
                } else {
                    let f = fan.get(&vk).copied().unwrap_or([0.0, 0.0, 1.0]);
                    let fl = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt();
                    nrm = if fl.is_finite() && fl > 0.0 {
                        [f[0] / fl, f[1] / fl, f[2] / fl]
                    } else {
                        [0.0, 0.0, 1.0]
                    };
                }
                if let Some(vd) = result.vertex.get_mut(&vk) {
                    vd.set_normal(nrm[0], nrm[1], nrm[2]);
                    vd.attributes.insert("u".to_string(), u);
                    vd.attributes.insert("v".to_string(), v);
                }
            }
        }
        for (li, vids) in loop_vids.iter().enumerate() {
            for (k, &vi) in vids.iter().enumerate() {
                if vi < 0 {
                    continue;
                }
                if let Some(vk) = vert_map[vi as usize] {
                    if let Some(vd) = result.vertex.get_mut(&vk) {
                        vd.attributes.insert(format!("boundary/{li}/{k}"), 1.0);
                    }
                }
            }
        }
        for (&vi, &(li, k, t)) in &boundary_intervals {
            if let Some(vk) = vert_map[vi] {
                result
                    .vertex
                    .get_mut(&vk)
                    .unwrap()
                    .attributes
                    .insert(format!("boundary_interval/{li}/{k}"), t);
            }
        }
        RemeshNurbsSurfaceGrid::split_crease_normals(&self.m_surface, &mut result);
        result
    }

    /// Mesh of the half (S-q0).n <= 0: span-adaptive grid, marching-squares clip with Newton-refined crossings, seams welded
    pub fn mesh_by_plane(
        &self,
        q0: &Point,
        normal: &Vector,
        max_angle_deg: f64,
        chord_factor: f64,
    ) -> Mesh {
        let srf = &self.m_surface;
        let (mut nx, mut ny, mut nz) = (normal[0], normal[1], normal[2]);
        let nl = (nx * nx + ny * ny + nz * nz).sqrt();
        if nl < 1e-12 {
            return srf.mesh();
        }
        nx /= nl;
        ny /= nl;
        nz /= nl;
        let (qx, qy, qz) = (q0[0], q0[1], q0[2]);
        let e3 = |u: f64, v: f64| -> [f64; 3] {
            let p = srf.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0));
            [p[0], p[1], p[2]]
        };
        let field = |u: f64, v: f64| -> f64 {
            let p = e3(u, v);
            (p[0] - qx) * nx + (p[1] - qy) * ny + (p[2] - qz) * nz
        };
        let refine = |mut u: f64, mut v: f64| -> (f64, f64) {
            for _ in 0..12 {
                let fv = field(u, v);
                if fv.abs() < 1e-9 {
                    break;
                }
                let h = 1e-4;
                let a = e3(u + h, v);
                let b = e3(u - h, v);
                let c = e3(u, v + h);
                let d = e3(u, v - h);
                let gu = ((a[0] - b[0]) * nx + (a[1] - b[1]) * ny + (a[2] - b[2]) * nz) / (2.0 * h);
                let gv = ((c[0] - d[0]) * nx + (c[1] - d[1]) * ny + (c[2] - d[2]) * nz) / (2.0 * h);
                let g2 = gu * gu + gv * gv;
                if g2 < 1e-20 {
                    break;
                }
                u -= fv * gu / g2;
                v -= fv * gv / g2;
            }
            (u, v)
        };
        let usp = srf.get_span_vector(0);
        let vsp = srf.get_span_vector(1);
        if usp.len() < 2 || vsp.len() < 2 {
            return srf.mesh();
        }
        let deg_u = srf.degree(0);
        let deg_v = srf.degree(1);
        let mut bmin = [1e30f64; 3];
        let mut bmax = [-1e30f64; 3];
        for i in 0..srf.cv_count(0) {
            for j in 0..srf.cv_count(1) {
                if let Some(p) = srf.get_cv(i, j) {
                    for k in 0..3 {
                        let c = p[k];
                        if c < bmin[k] {
                            bmin[k] = c;
                        }
                        if c > bmax[k] {
                            bmax[k] = c;
                        }
                    }
                }
            }
        }
        let mut diag = (0..3)
            .map(|k| (bmax[k] - bmin[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        if diag < 1e-12 {
            diag = 1.0;
        }
        let ctol = diag * chord_factor;
        let span_subs = |dr: usize, sp: &[f64], osp: &[f64], deg: usize| -> Vec<usize> {
            let n = sp.len() - 1;
            let mut out = vec![if deg > 1 { 2usize } else { 1 }; n];
            let smid = (osp[0] + osp[osp.len() - 1]) * 0.5;
            for i in 0..n {
                let (t0, t1) = (sp[i], sp[i + 1]);
                if deg > 1 {
                    let mut ma = 0.0_f64;
                    let mut pn = [0.0f64; 3];
                    for k in 0..=4 {
                        let t = t0 + k as f64 * (t1 - t0) / 4.0;
                        let nm = if dr == 0 {
                            srf.normal_at(t, smid)
                        } else {
                            srf.normal_at(smid, t)
                        };
                        if k > 0 {
                            let d =
                                (pn[0] * nm[0] + pn[1] * nm[1] + pn[2] * nm[2]).clamp(-1.0, 1.0);
                            ma += d.acos() * 180.0 / std::f64::consts::PI;
                        }
                        pn = [nm[0], nm[1], nm[2]];
                    }
                    out[i] = out[i].max(1.max(((ma / max_angle_deg).ceil() as usize).min(64)));
                }
                let p0 = if dr == 0 { e3(t0, smid) } else { e3(smid, t0) };
                let p1 = if dr == 0 { e3(t1, smid) } else { e3(smid, t1) };
                let mut dev = 0.0_f64;
                for k in 1..=3 {
                    let fr = k as f64 / 4.0;
                    let tm = t0 + fr * (t1 - t0);
                    let pm = if dr == 0 { e3(tm, smid) } else { e3(smid, tm) };
                    let (lx, ly, lz) = (
                        p0[0] + fr * (p1[0] - p0[0]),
                        p0[1] + fr * (p1[1] - p0[1]),
                        p0[2] + fr * (p1[2] - p0[2]),
                    );
                    dev = dev.max(
                        ((pm[0] - lx).powi(2) + (pm[1] - ly).powi(2) + (pm[2] - lz).powi(2)).sqrt(),
                    );
                }
                if dev > ctol {
                    out[i] = out[i].max(((dev / ctol).sqrt().ceil() as usize).min(64));
                }
            }
            out
        };
        let us_subs = span_subs(0, &usp, &vsp, deg_u);
        let vs_subs = span_subs(1, &vsp, &usp, deg_v);
        let mut us = Vec::new();
        for i in 0..usp.len() - 1 {
            for s in 0..us_subs[i] {
                us.push(usp[i] + (s as f64) * (usp[i + 1] - usp[i]) / (us_subs[i] as f64));
            }
        }
        us.push(*usp.last().unwrap());
        let mut vs = Vec::new();
        for i in 0..vsp.len() - 1 {
            for s in 0..vs_subs[i] {
                vs.push(vsp[i] + (s as f64) * (vsp[i + 1] - vsp[i]) / (vs_subs[i] as f64));
            }
        }
        vs.push(*vsp.last().unwrap());
        let nu = us.len();
        let nv = vs.len();
        if nu < 2 || nv < 2 {
            return srf.mesh();
        }
        let f_grid: Vec<Vec<f64>> = (0..nu)
            .map(|i| (0..nv).map(|j| field(us[i], vs[j])).collect())
            .collect();
        let mut result = Mesh::new();
        let wt = diag * 1e-5;
        let cell = if wt > 0.0 { wt } else { 1.0 };
        let mut cmap: std::collections::HashMap<(i64, i64, i64), Vec<([f64; 3], usize)>> =
            std::collections::HashMap::new();
        let weld = |result: &mut Mesh,
                    cmap: &mut std::collections::HashMap<
            (i64, i64, i64),
            Vec<([f64; 3], usize)>,
        >,
                    u: f64,
                    v: f64|
         -> usize {
            let p = srf.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0));
            let (x, y, z) = (p[0], p[1], p[2]);
            let (ci, cj, ck) = (
                (x / cell).floor() as i64,
                (y / cell).floor() as i64,
                (z / cell).floor() as i64,
            );
            for di in -1..=1 {
                for dj in -1..=1 {
                    for dk in -1..=1 {
                        if let Some(b) = cmap.get(&(ci + di, cj + dj, ck + dk)) {
                            for &(pp, vk) in b {
                                if (pp[0] - x).powi(2) + (pp[1] - y).powi(2) + (pp[2] - z).powi(2)
                                    <= wt * wt
                                {
                                    return vk;
                                }
                            }
                        }
                    }
                }
            }
            let vk = result.add_vertex(p, None);
            let nm = srf.normal_at(u, v);
            if let Some(vd) = result.vertex.get_mut(&vk) {
                vd.set_normal(nm[0], nm[1], nm[2]);
            }
            cmap.entry((ci, cj, ck)).or_default().push(([x, y, z], vk));
            vk
        };
        for i in 0..nu - 1 {
            for j in 0..nv - 1 {
                let cu = [us[i], us[i + 1], us[i + 1], us[i]];
                let cv = [vs[j], vs[j], vs[j + 1], vs[j + 1]];
                let inn = [
                    f_grid[i][j] <= 0.0,
                    f_grid[i + 1][j] <= 0.0,
                    f_grid[i + 1][j + 1] <= 0.0,
                    f_grid[i][j + 1] <= 0.0,
                ];
                if !inn.iter().any(|&b| b) {
                    continue;
                }
                let mut poly: Vec<usize> = Vec::new();
                for k in 0..4 {
                    let kn = (k + 1) % 4;
                    if inn[k] {
                        poly.push(weld(&mut result, &mut cmap, cu[k], cv[k]));
                    }
                    if inn[k] != inn[kn] {
                        let (fa, fb) = (
                            f_grid[[i, i + 1, i + 1, i][k]][[j, j, j + 1, j + 1][k]],
                            f_grid[[i, i + 1, i + 1, i][kn]][[j, j, j + 1, j + 1][kn]],
                        );
                        let t = if (fa - fb).abs() > 1e-30 {
                            fa / (fa - fb)
                        } else {
                            0.5
                        };
                        let (cx, cyv) =
                            (cu[k] + (cu[kn] - cu[k]) * t, cv[k] + (cv[kn] - cv[k]) * t);
                        let (ru, rv) = refine(cx, cyv);
                        poly.push(weld(&mut result, &mut cmap, ru, rv));
                    }
                }
                for t in 1..poly.len().saturating_sub(1) {
                    let (a, b, c) = (poly[0], poly[t], poly[t + 1]);
                    if a == b || b == c || c == a {
                        continue;
                    }
                    result.add_face(vec![a, b, c], None);
                }
            }
        }
        if result.face.is_empty() {
            return srf.mesh();
        }
        result
    }

    /// Mesh of the region inside every half-space (S-q).n <= 0: triangle soup clipped plane by plane, seams welded
    pub fn mesh_by_planes(
        &self,
        planes: &[(Point, Vector)],
        max_angle_deg: f64,
        chord_factor: f64,
    ) -> Mesh {
        let srf = &self.m_surface;
        let mut pl: Vec<([f64; 3], [f64; 3])> = Vec::new();
        for (q, n) in planes {
            let (nx, ny, nz) = (n[0], n[1], n[2]);
            let nl = (nx * nx + ny * ny + nz * nz).sqrt();
            if nl < 1e-12 {
                continue;
            }
            pl.push(([q[0], q[1], q[2]], [nx / nl, ny / nl, nz / nl]));
        }
        if pl.is_empty() {
            return srf.mesh();
        }
        let e3 = |u: f64, v: f64| -> [f64; 3] {
            let p = srf.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0));
            [p[0], p[1], p[2]]
        };
        let field_k = |k: usize, u: f64, v: f64| -> f64 {
            let p = e3(u, v);
            let (q, n) = &pl[k];
            (p[0] - q[0]) * n[0] + (p[1] - q[1]) * n[1] + (p[2] - q[2]) * n[2]
        };
        let refine_k = |k: usize, mut u: f64, mut v: f64| -> (f64, f64) {
            let (_q, n) = &pl[k];
            for _ in 0..12 {
                let fv = field_k(k, u, v);
                if fv.abs() < 1e-9 {
                    break;
                }
                let h = 1e-4;
                let a = e3(u + h, v);
                let b = e3(u - h, v);
                let c = e3(u, v + h);
                let d = e3(u, v - h);
                let gu = ((a[0] - b[0]) * n[0] + (a[1] - b[1]) * n[1] + (a[2] - b[2]) * n[2])
                    / (2.0 * h);
                let gv = ((c[0] - d[0]) * n[0] + (c[1] - d[1]) * n[1] + (c[2] - d[2]) * n[2])
                    / (2.0 * h);
                let g2 = gu * gu + gv * gv;
                if g2 < 1e-20 {
                    break;
                }
                u -= fv * gu / g2;
                v -= fv * gv / g2;
            }
            (u, v)
        };
        let usp = srf.get_span_vector(0);
        let vsp = srf.get_span_vector(1);
        if usp.len() < 2 || vsp.len() < 2 {
            return srf.mesh();
        }
        let deg_u = srf.degree(0);
        let deg_v = srf.degree(1);
        let mut bmin = [1e30f64; 3];
        let mut bmax = [-1e30f64; 3];
        for i in 0..srf.cv_count(0) {
            for j in 0..srf.cv_count(1) {
                if let Some(p) = srf.get_cv(i, j) {
                    for k in 0..3 {
                        let c = p[k];
                        if c < bmin[k] {
                            bmin[k] = c;
                        }
                        if c > bmax[k] {
                            bmax[k] = c;
                        }
                    }
                }
            }
        }
        let mut diag = (0..3)
            .map(|k| (bmax[k] - bmin[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        if diag < 1e-12 {
            diag = 1.0;
        }
        let ctol = diag * chord_factor;
        let span_subs = |dr: usize, sp: &[f64], osp: &[f64], deg: usize| -> Vec<usize> {
            let n = sp.len() - 1;
            let mut out = vec![if deg > 1 { 2usize } else { 1 }; n];
            let smid = (osp[0] + osp[osp.len() - 1]) * 0.5;
            for i in 0..n {
                let (t0, t1) = (sp[i], sp[i + 1]);
                if deg > 1 {
                    let mut ma = 0.0f64;
                    let mut pn = [0.0f64; 3];
                    for k in 0..=4 {
                        let t = t0 + k as f64 * (t1 - t0) / 4.0;
                        let nm = if dr == 0 {
                            srf.normal_at(t, smid)
                        } else {
                            srf.normal_at(smid, t)
                        };
                        if k > 0 {
                            let d =
                                (pn[0] * nm[0] + pn[1] * nm[1] + pn[2] * nm[2]).clamp(-1.0, 1.0);
                            ma += d.acos() * 180.0 / std::f64::consts::PI;
                        }
                        pn = [nm[0], nm[1], nm[2]];
                    }
                    out[i] = out[i].max(1.max(((ma / max_angle_deg).ceil() as usize).min(64)));
                }
                let p0 = if dr == 0 { e3(t0, smid) } else { e3(smid, t0) };
                let p1 = if dr == 0 { e3(t1, smid) } else { e3(smid, t1) };
                let mut dev = 0.0f64;
                for k in 1..=3 {
                    let fr = k as f64 / 4.0;
                    let tm = t0 + fr * (t1 - t0);
                    let pm = if dr == 0 { e3(tm, smid) } else { e3(smid, tm) };
                    let (lx, ly, lz) = (
                        p0[0] + fr * (p1[0] - p0[0]),
                        p0[1] + fr * (p1[1] - p0[1]),
                        p0[2] + fr * (p1[2] - p0[2]),
                    );
                    dev = dev.max(
                        ((pm[0] - lx).powi(2) + (pm[1] - ly).powi(2) + (pm[2] - lz).powi(2)).sqrt(),
                    );
                }
                if dev > ctol {
                    out[i] = out[i].max(((dev / ctol).sqrt().ceil() as usize).min(64));
                }
            }
            out
        };
        let us_subs = span_subs(0, &usp, &vsp, deg_u);
        let vs_subs = span_subs(1, &vsp, &usp, deg_v);
        let mut us = Vec::new();
        for i in 0..usp.len() - 1 {
            for s in 0..us_subs[i] {
                us.push(usp[i] + (s as f64) * (usp[i + 1] - usp[i]) / (us_subs[i] as f64));
            }
        }
        us.push(*usp.last().unwrap());
        let mut vs = Vec::new();
        for i in 0..vsp.len() - 1 {
            for s in 0..vs_subs[i] {
                vs.push(vsp[i] + (s as f64) * (vsp[i + 1] - vsp[i]) / (vs_subs[i] as f64));
            }
        }
        vs.push(*vsp.last().unwrap());
        let nu = us.len();
        let nv = vs.len();
        if nu < 2 || nv < 2 {
            return srf.mesh();
        }
        let mut tris: Vec<[(f64, f64); 3]> = Vec::with_capacity((nu - 1) * (nv - 1) * 2);
        for i in 0..nu - 1 {
            for j in 0..nv - 1 {
                let a = (us[i], vs[j]);
                let b = (us[i + 1], vs[j]);
                let c = (us[i + 1], vs[j + 1]);
                let d = (us[i], vs[j + 1]);
                tris.push([a, b, c]);
                tris.push([a, c, d]);
            }
        }
        let eps = 1e-9;
        for k in 0..pl.len() {
            let mut next: Vec<[(f64, f64); 3]> = Vec::new();
            for t in &tris {
                let mut poly: Vec<(f64, f64)> = Vec::new();
                for e in 0..3 {
                    let p = t[e];
                    let q = t[(e + 1) % 3];
                    let fp = field_k(k, p.0, p.1);
                    let fq = field_k(k, q.0, q.1);
                    let (pin, qin) = (fp <= eps, fq <= eps);
                    if pin {
                        poly.push(p);
                    }
                    if pin != qin {
                        let tt = if (fp - fq).abs() > 1e-30 {
                            fp / (fp - fq)
                        } else {
                            0.5
                        };
                        let cu = p.0 + (q.0 - p.0) * tt;
                        let cv = p.1 + (q.1 - p.1) * tt;
                        poly.push(refine_k(k, cu, cv));
                    }
                }
                for w in 1..poly.len().saturating_sub(1) {
                    next.push([poly[0], poly[w], poly[w + 1]]);
                }
            }
            tris = next;
            if tris.is_empty() {
                break;
            }
        }
        if tris.is_empty() {
            return Mesh::new();
        }
        let mut result = Mesh::new();
        let wt = diag * 1e-5;
        let cell = if wt > 0.0 { wt } else { 1.0 };
        let mut cmap: std::collections::HashMap<(i64, i64, i64), Vec<([f64; 3], usize)>> =
            std::collections::HashMap::new();
        let weld = |result: &mut Mesh,
                    cmap: &mut std::collections::HashMap<
            (i64, i64, i64),
            Vec<([f64; 3], usize)>,
        >,
                    u: f64,
                    v: f64|
         -> usize {
            let p = srf.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0));
            let (x, y, z) = (p[0], p[1], p[2]);
            let (ci, cj, ck) = (
                (x / cell).floor() as i64,
                (y / cell).floor() as i64,
                (z / cell).floor() as i64,
            );
            for di in -1..=1 {
                for dj in -1..=1 {
                    for dk in -1..=1 {
                        if let Some(b) = cmap.get(&(ci + di, cj + dj, ck + dk)) {
                            for &(pp, vk) in b {
                                if (pp[0] - x).powi(2) + (pp[1] - y).powi(2) + (pp[2] - z).powi(2)
                                    <= wt * wt
                                {
                                    return vk;
                                }
                            }
                        }
                    }
                }
            }
            let vk = result.add_vertex(p, None);
            let nm = srf.normal_at(u, v);
            if let Some(vd) = result.vertex.get_mut(&vk) {
                vd.set_normal(nm[0], nm[1], nm[2]);
            }
            cmap.entry((ci, cj, ck)).or_default().push(([x, y, z], vk));
            vk
        };
        for t in &tris {
            let a = weld(&mut result, &mut cmap, t[0].0, t[0].1);
            let b = weld(&mut result, &mut cmap, t[1].0, t[1].1);
            let c = weld(&mut result, &mut cmap, t[2].0, t[2].1);
            if a == b || b == c || c == a {
                continue;
            }
            result.add_face(vec![a, b, c], None);
        }
        if result.face.is_empty() {
            return Mesh::new();
        }
        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn transform(&mut self, xform: &Xform) {
        self.m_surface.transform(xform);
        // SESSION_VIEWER
        if let Some(q0) = self.cut_q0.as_mut() {
            q0.transform(xform);
        }
        if let Some(n) = self.cut_n.as_mut() {
            n.transform(xform);
        }
        for (q, n) in self.cut_planes.iter_mut() {
            q.transform(xform);
            n.transform(xform);
        }
    }

    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut ts = self.duplicate();
        ts.transform(xform);
        ts
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════

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
        Self::jsonload(json_string).unwrap_or_default()
    }

    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = serde_json::to_string_pretty(self)?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let contents = std::fs::read_to_string(filepath)?;
        Ok(serde_json::from_str(&contents)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        let mut proto = crate::proto::NurbsSurfaceTrimmed {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            width: self.width,
            surface: Some(self.m_surface.to_proto()),
            outer_loop: None,
            inner_loops: Vec::new(),
            surfacecolor: Some(crate::proto::Color {
                guid: String::new(),
                name: self.surfacecolor.name.clone(),
                r: self.surfacecolor.r,
                g: self.surfacecolor.g,
                b: self.surfacecolor.b,
                a: self.surfacecolor.a,
            }),
        };
        if self.is_trimmed() {
            if let Some(outer) = self.m_outer_loop.as_ref() {
                proto.outer_loop = Some(outer.to_proto());
            }
        }
        for inner in &self.m_inner_loops {
            proto.inner_loops.push(inner.to_proto());
        }
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::NurbsSurfaceTrimmed::decode(data)?;
        let mut ts = Self::new();
        if !proto.guid.is_empty() {
            ts.set_guid(proto.guid.clone());
        }
        ts.name = proto.name;
        ts.width = proto.width;
        if let Some(srf_proto) = proto.surface {
            ts.m_surface = NurbsSurface::from_proto(srf_proto)?;
        }
        if let Some(loop_proto) = proto.outer_loop {
            ts.m_outer_loop = Some(NurbsCurve::from_proto(loop_proto));
        }
        for loop_proto in proto.inner_loops {
            ts.m_inner_loops.push(NurbsCurve::from_proto(loop_proto));
        }
        if let Some(color) = proto.surfacecolor {
            ts.surfacecolor = Color::with_name(color.r, color.g, color.b, color.a, &color.name);
        }
        Ok(ts)
    }

    pub fn pb_dump(&self, filepath: &str) {
        let _ = std::fs::write(filepath, self.pb_dumps());
    }

    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).unwrap_or_default();
        Self::pb_loads(&data).unwrap_or_default()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════

    /// "NurbsSurfaceTrimmed(name=..., trimmed=..., holes=...)"
    pub fn str(&self) -> String {
        format!(
            "NurbsSurfaceTrimmed(name={}, trimmed={}, holes={})",
            self.name,
            self.is_trimmed(),
            self.inner_loop_count()
        )
    }

    /// Multi-line form with the surface
    pub fn repr(&self) -> String {
        format!(
            "NurbsSurfaceTrimmed(\n  name={},\n  trimmed={},\n  holes={},\n  surface={}\n)",
            self.name,
            self.is_trimmed(),
            self.inner_loop_count(),
            self.m_surface.str()
        )
    }
}

impl Default for NurbsSurfaceTrimmed {
    fn default() -> Self {
        Self::new()
    }
}

impl PartialEq for NurbsSurfaceTrimmed {
    fn eq(&self, other: &Self) -> bool {
        if self.name != other.name {
            return false;
        }
        if self.width != other.width {
            return false;
        }
        if self.surfacecolor != other.surfacecolor {
            return false;
        }
        if self.m_surface != other.m_surface {
            return false;
        }
        true
    }
}

impl std::fmt::Display for NurbsSurfaceTrimmed {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.str())
    }
}
