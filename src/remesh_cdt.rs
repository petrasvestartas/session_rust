// Constrained Delaunay Triangulation via sweep-line.
use std::collections::HashMap;
use crate::mesh::Mesh;
use crate::point::Point;
use crate::polyline::Polyline;

const LOOSE: u8 = 0;
const ASCEND: u8 = 1;
const DESCEND: u8 = 2;
const IX_NONE: u8 = 0;
const IX_COLLINEAR: u8 = 1;
const IX_INTERSECT: u8 = 2;
const EC_NEITHER: u8 = 0;
const EC_LEFT: u8 = 1;
const EC_RIGHT: u8 = 2;
const NULL: usize = usize::MAX;

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct P64 {
    x: i64,
    y: i64,
}

struct V2 {
    pt: P64,
    edges: Vec<usize>,
    inner_lm: bool,
}

struct Edge {
    vl: usize,
    vr: usize,
    vb: usize,
    vt: usize,
    kind: u8,
    tri_a: usize,
    tri_b: usize,
    is_active: bool,
    next_e: usize,
    prev_e: usize,
}

struct CdtTri {
    edges: [usize; 3],
}

fn cps(p1: P64, p2: P64, p3: P64) -> i32 {
    let cp = (p2.x - p1.x) as f64 * (p3.y - p2.y) as f64
           - (p2.y - p1.y) as f64 * (p3.x - p2.x) as f64;
    if cp > 0.0 { 1 } else if cp < 0.0 { -1 } else { 0 }
}

fn sqr(x: f64) -> f64 { x * x }

fn dist_sqr(a: P64, b: P64) -> f64 {
    sqr((a.x - b.x) as f64) + sqr((a.y - b.y) as f64)
}

fn left_turning(p1: P64, p2: P64, p3: P64) -> bool { cps(p1, p2, p3) < 0 }
fn right_turning(p1: P64, p2: P64, p3: P64) -> bool { cps(p1, p2, p3) > 0 }

fn is_horiz_e(e: &Edge, vs: &[V2]) -> bool { vs[e.vb].pt.y == vs[e.vt].pt.y }
fn is_loose_e(e: &Edge) -> bool { e.kind == LOOSE }
fn is_left_e(e: &Edge) -> bool { e.kind == ASCEND }
fn is_right_e(e: &Edge) -> bool { e.kind == DESCEND }

fn edge_completed(e: &Edge) -> bool {
    if e.tri_a == NULL { return false; }
    if e.tri_b != NULL { return true; }
    e.kind != LOOSE
}

fn edge_contains(e: &Edge, v: usize) -> u8 {
    if e.vl == v { EC_LEFT } else if e.vr == v { EC_RIGHT } else { EC_NEITHER }
}

fn in_circle(pa: P64, pb: P64, pc: P64, pd: P64) -> f64 {
    let m00 = (pa.x - pd.x) as f64; let m01 = (pa.y - pd.y) as f64; let m02 = sqr(m00) + sqr(m01);
    let m10 = (pb.x - pd.x) as f64; let m11 = (pb.y - pd.y) as f64; let m12 = sqr(m10) + sqr(m11);
    let m20 = (pc.x - pd.x) as f64; let m21 = (pc.y - pd.y) as f64; let m22 = sqr(m20) + sqr(m21);
    m00*(m11*m22-m21*m12) - m10*(m01*m22-m21*m02) + m20*(m01*m12-m11*m02)
}

fn shortest_dist_seg(pt: P64, sp1: P64, sp2: P64) -> f64 {
    let dx = (sp2.x - sp1.x) as f64; let dy = (sp2.y - sp1.y) as f64;
    let ax = (pt.x - sp1.x) as f64; let ay = (pt.y - sp1.y) as f64;
    let qnum = ax*dx + ay*dy;
    if qnum < 0.0 { return dist_sqr(pt, sp1); }
    let dd = dx*dx + dy*dy;
    if qnum > dd { return dist_sqr(pt, sp2); }
    sqr(ax*dy - dx*ay) / dd
}

fn segs_intersect(s1a: P64, s1b: P64, s2a: P64, s2b: P64) -> u8 {
    if s1a == s2a || s1b == s2a || s1b == s2b || s1a == s2b { return IX_NONE; }
    let dy1 = (s1b.y - s1a.y) as f64; let dx1 = (s1b.x - s1a.x) as f64;
    let dy2 = (s2b.y - s2a.y) as f64; let dx2 = (s2b.x - s2a.x) as f64;
    let cp = dy1*dx2 - dy2*dx1;
    if cp == 0.0 { return IX_COLLINEAR; }
    let t = (s1a.x - s2a.x) as f64 * dy2 - (s1a.y - s2a.y) as f64 * dx2;
    if t >= 0.0 { if cp < 0.0 || t >= cp { return IX_NONE; } }
    else        { if cp > 0.0 || t <= cp { return IX_NONE; } }
    let t2 = (s1a.x - s2a.x) as f64 * dy1 - (s1a.y - s2a.y) as f64 * dx1;
    if t2 >= 0.0 { if cp > 0.0 && t2 < cp { return IX_INTERSECT; } }
    else         { if cp < 0.0 && t2 > cp { return IX_INTERSECT; } }
    IX_NONE
}

fn find_loc_min_idx(path: &[P64], length: usize, start: usize) -> Option<usize> {
    if length < 3 { return None; }
    let i0 = start;
    let mut idx = start;
    let mut n = (idx + 1) % length;
    while path[n].y <= path[idx].y {
        idx = n; n = (n + 1) % length;
        if idx == i0 { return None; }
    }
    while path[n].y >= path[idx].y {
        idx = n; n = (n + 1) % length;
    }
    Some(idx)
}

fn prev_idx(idx: usize, length: usize) -> usize {
    if idx == 0 { length - 1 } else { idx - 1 }
}

fn next_idx(idx: usize, length: usize) -> usize { (idx + 1) % length }

fn remove_edge_from_vert(vs: &mut [V2], v: usize, e: usize) {
    if let Some(pos) = vs[v].edges.iter().position(|&x| x == e) {
        vs[v].edges.remove(pos);
    }
}

fn find_linking_edge(vs: &[V2], es: &[Edge], vert1: usize, vert2: usize, prefer_asc: bool) -> usize {
    let mut res = NULL;
    for &e in &vs[vert1].edges {
        if es[e].vl == vert2 || es[e].vr == vert2 {
            if es[e].kind == LOOSE || ((es[e].kind == ASCEND) == prefer_asc) { return e; }
            res = e;
        }
    }
    res
}

fn path_from_tri(es: &[Edge], vs: &[V2], tri: &CdtTri) -> [P64; 3] {
    let e0 = &es[tri.edges[0]];
    let p0 = vs[e0.vl].pt;
    let p1 = vs[e0.vr].pt;
    let e1 = &es[tri.edges[1]];
    let p2 = if vs[e1.vl].pt == p0 || vs[e1.vl].pt == p1 { vs[e1.vr].pt } else { vs[e1.vl].pt };
    [p0, p1, p2]
}

struct Delaunay {
    vs: Vec<V2>,
    es: Vec<Edge>,
    ts: Vec<CdtTri>,
    pending: Vec<usize>,
    horz: Vec<usize>,
    loc_mins: Vec<usize>,
    use_del: bool,
    lowermost: usize,
    first_active: usize,
}

impl Delaunay {
    fn new(use_del: bool) -> Self {
        Delaunay {
            vs: Vec::new(), es: Vec::new(), ts: Vec::new(),
            pending: Vec::new(), horz: Vec::new(), loc_mins: Vec::new(),
            use_del, lowermost: NULL, first_active: NULL,
        }
    }

    fn add_active(&mut self, eid: usize) {
        if self.es[eid].is_active { return; }
        self.es[eid].prev_e = NULL;
        self.es[eid].next_e = self.first_active;
        self.es[eid].is_active = true;
        if self.first_active != NULL { self.es[self.first_active].prev_e = eid; }
        self.first_active = eid;
    }

    fn remove_active(&mut self, eid: usize) {
        let vb = self.es[eid].vb;
        let vt = self.es[eid].vt;
        remove_edge_from_vert(&mut self.vs, vb, eid);
        remove_edge_from_vert(&mut self.vs, vt, eid);
        let prev = self.es[eid].prev_e;
        let nxt  = self.es[eid].next_e;
        if nxt  != NULL { self.es[nxt].prev_e  = prev; }
        if prev != NULL { self.es[prev].next_e  = nxt;  }
        self.es[eid].is_active = false;
        if self.first_active == eid { self.first_active = nxt; }
    }

    fn create_edge(&mut self, v1: usize, v2: usize, kind: u8) -> usize {
        let eid = self.es.len();
        let p1 = self.vs[v1].pt; let p2 = self.vs[v2].pt;
        let (vb, vt) = if p1.y == p2.y { (v1, v2) } else if p1.y < p2.y { (v2, v1) } else { (v1, v2) };
        let (vl, vr) = if p1.x <= p2.x { (v1, v2) } else { (v2, v1) };
        self.es.push(Edge { vl, vr, vb, vt, kind, tri_a: NULL, tri_b: NULL,
                            is_active: false, next_e: NULL, prev_e: NULL });
        self.vs[v1].edges.push(eid);
        self.vs[v2].edges.push(eid);
        if kind == LOOSE {
            self.pending.push(eid);
            self.add_active(eid);
        }
        eid
    }

    fn create_tri(&mut self, e1: usize, e2: usize, e3: usize) -> usize {
        let tid = self.ts.len();
        self.ts.push(CdtTri { edges: [e1, e2, e3] });
        for i in 0..3 {
            let eid = self.ts[tid].edges[i];
            if self.es[eid].tri_a != NULL {
                self.es[eid].tri_b = tid;
                self.remove_active(eid);
            } else {
                self.es[eid].tri_a = tid;
                if !is_loose_e(&self.es[eid]) { self.remove_active(eid); }
            }
        }
        tid
    }

    fn split_edge(&mut self, long_e: usize, short_e: usize) {
        let old_t = self.es[long_e].vt;
        let new_t = self.es[short_e].vt;
        remove_edge_from_vert(&mut self.vs, old_t, long_e);
        self.es[long_e].vt = new_t;
        if self.es[long_e].vl == old_t { self.es[long_e].vl = new_t; }
        else                            { self.es[long_e].vr = new_t; }
        self.vs[new_t].edges.push(long_e);
        let kind = self.es[long_e].kind;
        self.create_edge(new_t, old_t, kind);
    }

    fn merge_dup_collinear(&mut self) {
        let mut iter1 = 0usize;
        let n = self.vs.len();
        let mut iter2 = 1usize;
        while iter2 < n {
            if self.vs[iter1].pt != self.vs[iter2].pt {
                iter1 = iter2; iter2 += 1; continue;
            }
            if !self.vs[iter1].inner_lm || !self.vs[iter2].inner_lm {
                self.vs[iter1].inner_lm = false;
            }
            let edges2: Vec<usize> = self.vs[iter2].edges.clone();
            for &e in &edges2 {
                if self.es[e].vb == iter2 { self.es[e].vb = iter1; } else { self.es[e].vt = iter1; }
                if self.es[e].vl == iter2 { self.es[e].vl = iter1; } else { self.es[e].vr = iter1; }
            }
            let mut combined = self.vs[iter1].edges.clone();
            combined.extend(edges2);
            self.vs[iter1].edges = combined;
            self.vs[iter2].edges.clear();
            let edges1: Vec<usize> = self.vs[iter1].edges.clone();
            'outer: for &e1 in &edges1 {
                if is_horiz_e(&self.es[e1], &self.vs) || self.es[e1].vb != iter1 { continue; }
                let edges1b: Vec<usize> = self.vs[iter1].edges.clone();
                for &e2 in &edges1b {
                    if e2 == e1 || self.es[e2].vb != iter1 { continue; }
                    let t1y = self.vs[self.es[e1].vt].pt.y;
                    let t2y = self.vs[self.es[e2].vt].pt.y;
                    if t1y == t2y { continue; }
                    let pt1 = self.vs[self.es[e1].vt].pt;
                    let pt2 = self.vs[self.es[e2].vt].pt;
                    let pv = self.vs[iter1].pt;
                    if cps(pt1, pv, pt2) != 0 { continue; }
                    if t1y < t2y { self.split_edge(e1, e2); } else { self.split_edge(e2, e1); }
                    break 'outer;
                }
            }
            iter2 += 1;
        }
    }

    fn inner_loc_min_edge(&mut self, v_above: usize) -> usize {
        if self.first_active == NULL { return NULL; }
        let xa = self.vs[v_above].pt.x;
        let ya = self.vs[v_above].pt.y;
        let mut e = self.first_active;
        let mut e_below = NULL;
        let mut best_d = -1.0f64;
        while e != NULL {
            let vl_x = self.vs[self.es[e].vl].pt.x;
            let vr_x = self.vs[self.es[e].vr].pt.x;
            let vb_y = self.vs[self.es[e].vb].pt.y;
            let vb = self.es[e].vb; let vt = self.es[e].vt;
            let vl_pt = self.vs[self.es[e].vl].pt;
            let vr_pt = self.vs[self.es[e].vr].pt;
            let va_pt = self.vs[v_above].pt;
            if vl_x <= xa && vr_x >= xa && vb_y >= ya && vb != v_above && vt != v_above
                && !left_turning(vl_pt, va_pt, vr_pt)
            {
                let d = shortest_dist_seg(va_pt, vl_pt, vr_pt);
                if e_below == NULL || d < best_d { e_below = e; best_d = d; }
            }
            e = self.es[e].next_e;
        }
        if e_below == NULL { return NULL; }
        let vt_eb = self.es[e_below].vt;
        let vb_eb = self.es[e_below].vb;
        let mut v_best = if self.vs[vt_eb].pt.y <= ya { vb_eb } else { vt_eb };
        let mut x_best = self.vs[v_best].pt.x;
        let mut y_best = self.vs[v_best].pt.y;
        let va_pt = self.vs[v_above].pt;
        e = self.first_active;
        if x_best < xa {
            while e != NULL {
                let vr_x = self.vs[self.es[e].vr].pt.x;
                let vl_x = self.vs[self.es[e].vl].pt.x;
                let vb_y = self.vs[self.es[e].vb].pt.y;
                let vt_y = self.vs[self.es[e].vt].pt.y;
                if vr_x > x_best && vl_x < xa && vb_y > ya && vt_y < y_best {
                    let vb_pt = self.vs[self.es[e].vb].pt;
                    let vt_pt = self.vs[self.es[e].vt].pt;
                    let vb2 = self.vs[v_best].pt;
                    if segs_intersect(vb_pt, vt_pt, vb2, va_pt) == IX_INTERSECT {
                        let et = self.es[e].vt; let eb = self.es[e].vb;
                        v_best = if self.vs[et].pt.y > ya { et } else { eb };
                        x_best = self.vs[v_best].pt.x;
                        y_best = self.vs[v_best].pt.y;
                    }
                }
                e = self.es[e].next_e;
            }
        } else {
            while e != NULL {
                let vr_x = self.vs[self.es[e].vr].pt.x;
                let vl_x = self.vs[self.es[e].vl].pt.x;
                let vb_y = self.vs[self.es[e].vb].pt.y;
                let vt_y = self.vs[self.es[e].vt].pt.y;
                if vr_x < x_best && vl_x > xa && vb_y > ya && vt_y < y_best {
                    let vb_pt = self.vs[self.es[e].vb].pt;
                    let vt_pt = self.vs[self.es[e].vt].pt;
                    let vb2 = self.vs[v_best].pt;
                    if segs_intersect(vb_pt, vt_pt, vb2, va_pt) == IX_INTERSECT {
                        let et = self.es[e].vt; let eb = self.es[e].vb;
                        v_best = if self.vs[et].pt.y > ya { et } else { eb };
                        x_best = self.vs[v_best].pt.x;
                        y_best = self.vs[v_best].pt.y;
                    }
                }
                e = self.es[e].next_e;
            }
        }
        let _ = (x_best, y_best);
        self.create_edge(v_best, v_above, LOOSE)
    }

    fn horiz_between(&self, v1: usize, v2: usize) -> bool {
        let y = self.vs[v1].pt.y;
        let (l, r) = if self.vs[v1].pt.x > self.vs[v2].pt.x {
            (self.vs[v2].pt.x, self.vs[v1].pt.x)
        } else {
            (self.vs[v1].pt.x, self.vs[v2].pt.x)
        };
        let mut e = self.first_active;
        while e != NULL {
            let vl = self.es[e].vl; let vr = self.es[e].vr;
            if self.vs[vl].pt.y == y && self.vs[vr].pt.y == y
                && self.vs[vl].pt.x >= l && self.vs[vr].pt.x <= r
                && (self.vs[vl].pt.x != l || self.vs[vl].pt.x != r)
            { return true; }
            e = self.es[e].next_e;
        }
        false
    }

    fn tri_left(&mut self, edge: usize, pivot: usize, min_y: i64) {
        let v = { let e = &self.es[edge]; if e.vb == pivot { e.vt } else { e.vb } };
        let mut v_alt = NULL;
        let mut e_alt = NULL;
        let pivot_edges: Vec<usize> = self.vs[pivot].edges.clone();
        for e in pivot_edges {
            if e == edge || !self.es[e].is_active { continue; }
            let vx = { let ep = &self.es[e]; if ep.vt == pivot { ep.vb } else { ep.vt } };
            if vx == v { continue; }
            let c = cps(self.vs[v].pt, self.vs[pivot].pt, self.vs[vx].pt);
            if c == 0 {
                let vx_px = self.vs[vx].pt.x;
                let v_px = self.vs[v].pt.x;
                let pv_px = self.vs[pivot].pt.x;
                if (v_px > pv_px) == (pv_px > vx_px) { continue; }
            } else if c > 0 || (v_alt != NULL && !left_turning(self.vs[vx].pt, self.vs[pivot].pt, self.vs[v_alt].pt)) {
                continue;
            }
            v_alt = vx; e_alt = e;
        }
        if v_alt == NULL || self.vs[v_alt].pt.y < min_y { return; }
        if self.vs[v_alt].pt.y < self.vs[pivot].pt.y { if is_left_e(&self.es[e_alt]) { return; } }
        else if self.vs[v_alt].pt.y > self.vs[pivot].pt.y { if is_right_e(&self.es[e_alt]) { return; } }
        let prefer = self.vs[v_alt].pt.y < self.vs[v].pt.y;
        let ex = find_linking_edge(&self.vs, &self.es, v_alt, v, prefer);
        let ex = if ex == NULL {
            if self.vs[v_alt].pt.y == self.vs[v].pt.y && self.vs[v].pt.y == min_y && self.horiz_between(v_alt, v) { return; }
            self.create_edge(v_alt, v, LOOSE)
        } else { ex };
        self.create_tri(edge, e_alt, ex);
        if !edge_completed(&self.es[ex]) { self.tri_left(ex, v_alt, min_y); }
    }

    fn tri_right(&mut self, edge: usize, pivot: usize, min_y: i64) {
        let v = { let e = &self.es[edge]; if e.vb == pivot { e.vt } else { e.vb } };
        let mut v_alt = NULL;
        let mut e_alt = NULL;
        let pivot_edges: Vec<usize> = self.vs[pivot].edges.clone();
        for e in pivot_edges {
            if e == edge || !self.es[e].is_active { continue; }
            let vx = { let ep = &self.es[e]; if ep.vt == pivot { ep.vb } else { ep.vt } };
            if vx == v { continue; }
            let c = cps(self.vs[v].pt, self.vs[pivot].pt, self.vs[vx].pt);
            if c == 0 {
                let vx_px = self.vs[vx].pt.x;
                let v_px = self.vs[v].pt.x;
                let pv_px = self.vs[pivot].pt.x;
                if (v_px > pv_px) == (pv_px > vx_px) { continue; }
            } else if c < 0 || (v_alt != NULL && !right_turning(self.vs[vx].pt, self.vs[pivot].pt, self.vs[v_alt].pt)) {
                continue;
            }
            v_alt = vx; e_alt = e;
        }
        if v_alt == NULL || self.vs[v_alt].pt.y < min_y { return; }
        if self.vs[v_alt].pt.y < self.vs[pivot].pt.y { if is_right_e(&self.es[e_alt]) { return; } }
        else if self.vs[v_alt].pt.y > self.vs[pivot].pt.y { if is_left_e(&self.es[e_alt]) { return; } }
        let prefer = self.vs[v_alt].pt.y > self.vs[v].pt.y;
        let ex = find_linking_edge(&self.vs, &self.es, v_alt, v, prefer);
        let ex = if ex == NULL {
            if self.vs[v_alt].pt.y == self.vs[v].pt.y && self.vs[v].pt.y == min_y && self.horiz_between(v_alt, v) { return; }
            self.create_edge(v_alt, v, LOOSE)
        } else { ex };
        self.create_tri(edge, ex, e_alt);
        if !edge_completed(&self.es[ex]) { self.tri_right(ex, v_alt, min_y); }
    }

    fn force_legal(&mut self, edge: usize) {
        if self.es[edge].tri_a == NULL || self.es[edge].tri_b == NULL { return; }
        let mut vert_a = NULL; let mut vert_b = NULL;
        let mut edges_a = [NULL; 3]; let mut edges_b = [NULL; 3];
        let vl = self.es[edge].vl;
        let ta = self.es[edge].tri_a;
        let tb = self.es[edge].tri_b;
        for i in 0..3 {
            let eid = self.ts[ta].edges[i];
            if eid == edge { continue; }
            let ec = edge_contains(&self.es[eid], vl);
            if ec == EC_LEFT   { edges_a[1] = eid; vert_a = self.es[eid].vr; }
            else if ec == EC_RIGHT { edges_a[1] = eid; vert_a = self.es[eid].vl; }
            else                   { edges_b[1] = eid; }
        }
        for i in 0..3 {
            let eid = self.ts[tb].edges[i];
            if eid == edge { continue; }
            let ec = edge_contains(&self.es[eid], vl);
            if ec == EC_LEFT   { edges_a[2] = eid; vert_b = self.es[eid].vr; }
            else if ec == EC_RIGHT { edges_a[2] = eid; vert_b = self.es[eid].vl; }
            else                   { edges_b[2] = eid; }
        }
        if vert_a == NULL || vert_b == NULL { return; }
        let vl_pt = self.vs[vl].pt;
        let vr_pt = self.vs[self.es[edge].vr].pt;
        let va_pt = self.vs[vert_a].pt;
        let vb_pt = self.vs[vert_b].pt;
        if cps(va_pt, vl_pt, vr_pt) == 0 { return; }
        let ict = in_circle(va_pt, vl_pt, vr_pt, vb_pt);
        if ict == 0.0 || (right_turning(va_pt, vl_pt, vr_pt) == (ict < 0.0)) { return; }
        self.es[edge].vl = vert_a;
        self.es[edge].vr = vert_b;
        self.ts[ta].edges[0] = edge;
        for i in 1..3 {
            let ea = edges_a[i];
            self.ts[ta].edges[i] = ea;
            if is_loose_e(&self.es[ea]) { self.pending.push(ea); }
            if self.es[ea].tri_a == ta || self.es[ea].tri_b == ta { continue; }
            if self.es[ea].tri_a == tb      { self.es[ea].tri_a = ta; }
            else if self.es[ea].tri_b == tb { self.es[ea].tri_b = ta; }
        }
        self.ts[tb].edges[0] = edge;
        for i in 1..3 {
            let eb = edges_b[i];
            self.ts[tb].edges[i] = eb;
            if is_loose_e(&self.es[eb]) { self.pending.push(eb); }
            if self.es[eb].tri_a == tb || self.es[eb].tri_b == tb { continue; }
            if self.es[eb].tri_a == ta      { self.es[eb].tri_a = tb; }
            else if self.es[eb].tri_b == ta { self.es[eb].tri_b = tb; }
        }
    }

    fn add_path(&mut self, path: &[P64]) {
        let length = path.len();
        let i0 = match find_loc_min_idx(path, length, 0) { Some(x) => x, None => return };
        let mut i_prev = prev_idx(i0, length);
        while path[i_prev] == path[i0] { i_prev = prev_idx(i_prev, length); }
        let mut i_next = next_idx(i0, length);
        let mut i = i0;
        while cps(path[i_prev], path[i], path[i_next]) == 0 {
            i = match find_loc_min_idx(path, length, i) { Some(x) => x, None => return };
            if i == i0 { return; }
            i_prev = prev_idx(i, length);
            while path[i_prev] == path[i] { i_prev = prev_idx(i_prev, length); }
            i_next = next_idx(i, length);
        }
        let vert_cnt = self.vs.len();
        let v0 = self.vs.len();
        self.vs.push(V2 { pt: path[i], edges: Vec::new(), inner_lm: false });
        if left_turning(path[i_prev], path[i], path[i_next]) { self.vs[v0].inner_lm = true; }
        let mut v_prev = v0;
        i = i_next;
        // Degeneracy guard: a valid simple path is walked in O(len) advances. A degenerate or
        // self-intersecting path (e.g. an inexact conic pcurve that collapses to a collinear/looping
        // run after integer quantization) can spin these sweep loops forever -> bound the total work
        // and discard the path if the budget is blown, so the CDT can never hang the kernel.
        let mut steps: usize = 0;
        let budget = 16 * length + 256;
        let mut bailed = false;
        'walk: loop {
            steps += 1;
            if steps > budget { bailed = true; break 'walk; }
            self.loc_mins.push(v_prev);
            let vp_pt = self.vs[v_prev].pt;
            if self.lowermost == NULL
                || vp_pt.y > self.vs[self.lowermost].pt.y
                || (vp_pt.y == self.vs[self.lowermost].pt.y && vp_pt.x < self.vs[self.lowermost].pt.x)
            { self.lowermost = v_prev; }
            i_next = next_idx(i, length);
            if cps(self.vs[v_prev].pt, path[i], path[i_next]) == 0 { i = i_next; continue; }
            while path[i].y <= self.vs[v_prev].pt.y {
                steps += 1;
                if steps > budget { bailed = true; break 'walk; }
                let vn = self.vs.len();
                self.vs.push(V2 { pt: path[i], edges: Vec::new(), inner_lm: false });
                self.create_edge(v_prev, vn, ASCEND);
                v_prev = vn;
                i = i_next; i_next = next_idx(i, length);
                while cps(self.vs[v_prev].pt, path[i], path[i_next]) == 0 {
                    steps += 1;
                    if steps > budget { bailed = true; break 'walk; }
                    i = i_next; i_next = next_idx(i, length);
                }
            }
            let mut v_prev_prev = v_prev;
            while i != i0 && path[i].y >= self.vs[v_prev].pt.y {
                steps += 1;
                if steps > budget { bailed = true; break 'walk; }
                let vn = self.vs.len();
                self.vs.push(V2 { pt: path[i], edges: Vec::new(), inner_lm: false });
                self.create_edge(vn, v_prev, DESCEND);
                v_prev_prev = v_prev; v_prev = vn;
                i = i_next; i_next = next_idx(i, length);
                while cps(self.vs[v_prev].pt, path[i], path[i_next]) == 0 {
                    steps += 1;
                    if steps > budget { bailed = true; break 'walk; }
                    i = i_next; i_next = next_idx(i, length);
                }
            }
            if i == i0 { break; }
            if left_turning(self.vs[v_prev_prev].pt, self.vs[v_prev].pt, path[i]) {
                self.vs[v_prev].inner_lm = true;
            }
        }
        if bailed {
            // Sweep budget blown -> degenerate/self-intersecting path; discard its partial edges.
            for j in vert_cnt..self.vs.len() { self.vs[j].edges.clear(); }
            return;
        }
        self.create_edge(v0, v_prev, DESCEND);
        let n_new = self.vs.len() - vert_cnt;
        if n_new < 3 || (n_new == 3 && (
            dist_sqr(self.vs[vert_cnt].pt, self.vs[vert_cnt+1].pt) <= 1.0 ||
            dist_sqr(self.vs[vert_cnt+1].pt, self.vs[vert_cnt+2].pt) <= 1.0 ||
            dist_sqr(self.vs[vert_cnt+2].pt, self.vs[vert_cnt].pt) <= 1.0))
        {
            for j in vert_cnt..self.vs.len() { self.vs[j].edges.clear(); }
        }
    }

    fn execute(mut self, paths: &[Vec<P64>]) -> Vec<[P64; 3]> {
        let total: usize = paths.iter().map(|p| p.len()).sum();
        if total == 0 { return Vec::new(); }
        for path in paths { self.add_path(path); }
        if self.vs.len() <= 2 { return Vec::new(); }
        if self.lowermost != NULL && self.vs[self.lowermost].inner_lm {
            for lm in &self.loc_mins { self.vs[*lm].inner_lm = !self.vs[*lm].inner_lm; }
            for e in &mut self.es {
                if e.kind == ASCEND { e.kind = DESCEND; } else if e.kind == DESCEND { e.kind = ASCEND; }
            }
        }
        self.loc_mins.clear();
        let n_vs = self.vs.len();
        let mut order: Vec<usize> = (0..n_vs).collect();
        order.sort_by(|&a, &b| self.vs[b].pt.y.cmp(&self.vs[a].pt.y).then(self.vs[a].pt.x.cmp(&self.vs[b].pt.x)));
        let mut remap: Vec<usize> = vec![0; n_vs];
        for (new_idx, &old_idx) in order.iter().enumerate() { remap[old_idx] = new_idx; }
        let mut old_vs: Vec<V2> = std::mem::take(&mut self.vs);
        self.vs = order.iter().map(|&old_idx| {
            let v = &mut old_vs[old_idx];
            V2 { pt: v.pt, edges: std::mem::take(&mut v.edges), inner_lm: v.inner_lm }
        }).collect();
        for e in &mut self.es {
            e.vb = remap[e.vb]; e.vt = remap[e.vt]; e.vl = remap[e.vl]; e.vr = remap[e.vr];
        }
        if self.lowermost != NULL { self.lowermost = remap[self.lowermost]; }
        self.merge_dup_collinear();
        let mut curr_y = self.vs[0].pt.y;
        let nv = self.vs.len();
        for vi in 0..nv {
            if self.vs[vi].edges.is_empty() { continue; }
            if self.vs[vi].pt.y != curr_y {
                while let Some(lm) = self.loc_mins.pop() {
                    let e = self.inner_loc_min_edge(lm);
                    if e == NULL { return Vec::new(); }
                    if is_horiz_e(&self.es[e], &self.vs) {
                        let vb = self.es[e].vb; let vl = self.es[e].vl;
                        if vl == vb { self.tri_left(e, vb, curr_y); }
                        else        { self.tri_right(e, vb, curr_y); }
                    } else {
                        let vb = self.es[e].vb;
                        self.tri_left(e, vb, curr_y);
                        if !edge_completed(&self.es[e]) { self.tri_right(e, vb, curr_y); }
                    }
                    let lm_e0 = self.vs[lm].edges[0];
                    let lm_e1 = self.vs[lm].edges[1];
                    self.add_active(lm_e0);
                    self.add_active(lm_e1);
                }
                while let Some(e) = self.horz.pop() {
                    if edge_completed(&self.es[e]) { continue; }
                    let vb = self.es[e].vb; let vl = self.es[e].vl;
                    if vb == vl {
                        if is_left_e(&self.es[e]) { self.tri_left(e, vb, curr_y); }
                    } else {
                        if is_right_e(&self.es[e]) { self.tri_right(e, vb, curr_y); }
                    }
                }
                curr_y = self.vs[vi].pt.y;
            }
            let vi_edges: Vec<usize> = self.vs[vi].edges.clone();
            for i in (0..vi_edges.len()).rev() {
                if i >= self.vs[vi].edges.len() { continue; }
                let e = vi_edges[i];
                if edge_completed(&self.es[e]) || is_loose_e(&self.es[e]) { continue; }
                let vb = self.es[e].vb;
                if vi == vb {
                    if is_horiz_e(&self.es[e], &self.vs) { self.horz.push(e); }
                    if !self.vs[vi].inner_lm { self.add_active(e); }
                } else {
                    if is_horiz_e(&self.es[e], &self.vs) { self.horz.push(e); }
                    else if is_left_e(&self.es[e]) { self.tri_left(e, vb, self.vs[vi].pt.y); }
                    else                           { self.tri_right(e, vb, self.vs[vi].pt.y); }
                }
            }
            if self.vs[vi].inner_lm { self.loc_mins.push(vi); }
        }
        while let Some(e) = self.horz.pop() {
            if !edge_completed(&self.es[e]) {
                let vb = self.es[e].vb; let vl = self.es[e].vl;
                if vb == vl { self.tri_left(e, vb, curr_y); }
            }
        }
        // Legalize all interior diagonal edges. force_legal re-pushes up to 4 neighbours per flip, so
        // on degenerate / near-cocircular integer points the InCircle vs turn signs can disagree and two
        // edges flip-flop forever. Bound the total flips: a valid triangulation legalizes in O(n) flips,
        // far under this budget; a degenerate one stops early with a still-valid (non-optimal) mesh.
        if self.use_del {
            let mut flips: usize = 0;
            let flip_budget = 64 * self.vs.len() + 4096;
            while !self.pending.is_empty() {
                flips += 1;
                if flips > flip_budget { break; }
                let e = self.pending.pop().unwrap();
                self.force_legal(e);
            }
        }
        let mut res = Vec::new();
        for ti in 0..self.ts.len() {
            let p = path_from_tri(&self.es, &self.vs, &self.ts[ti]);
            let c = cps(p[0], p[1], p[2]);
            if c == 0 { continue; }
            if c < 0 { res.push([p[2], p[1], p[0]]); } else { res.push(p); }
        }
        res
    }
}

pub(crate) fn cdt_triangulate(border_2d: &[Point], holes_2d: &[Vec<Point>]) -> Vec<(usize, usize, usize)> {
    let mut max_coord = 1.0f64;
    for p in border_2d {
        max_coord = max_coord.max(p[0].abs()).max(p[1].abs());
    }
    for h in holes_2d {
        for p in h {
            max_coord = max_coord.max(p[0].abs()).max(p[1].abs());
        }
    }
    let mut precision = 6i32;
    while precision > 0 && max_coord * 10f64.powi(precision) > 9e17 {
        precision -= 1;
    }
    let scale = 10f64.powi(precision);

    // Break y-collinearity between hole vertices and border vertices.
    // The sweep-line CDT fails when a hole vertex shares the same int64
    // y-coordinate as a border vertex (purely y-collinear constraint segments
    // at different x positions break event ordering → 0 adjacent triangles for
    // that hole). Fix: shift each conflicting hole vertex by -1 int64 unit in y
    // (≈ 1/scale metres), which is imperceptible in practice.
    let border_ys: std::collections::HashSet<i64> = border_2d.iter()
        .map(|p| (p[1] * scale).round() as i64)
        .collect();
    let holes_adj: Vec<Vec<Point>> = holes_2d.iter().map(|hole| {
        hole.iter().map(|p| {
            let iy = (p[1] * scale).round() as i64;
            if border_ys.contains(&iy) {
                let mut adj = p.clone();
                adj[1] = (iy - 1) as f64 / scale;
                adj
            } else {
                p.clone()
            }
        }).collect()
    }).collect();

    let mut flat: Vec<Point> = border_2d.to_vec();
    for h in &holes_adj { flat.extend(h.iter().cloned()); }
    let mut pt_map: HashMap<(i64, i64), usize> = HashMap::new();
    for (i, p) in flat.iter().enumerate() {
        let key = ((p[0] * scale).round() as i64, (p[1] * scale).round() as i64);
        pt_map.entry(key).or_insert(i);
    }
    let make_path = |pts: &[Point]| -> Vec<P64> {
        let mut path: Vec<P64> = pts.iter().map(|p| P64 {
            x: (p[0] * scale).round() as i64, y: (p[1] * scale).round() as i64
        }).collect();
        if path.len() > 1 && path[0] == *path.last().unwrap() { path.pop(); }
        path
    };
    let mut paths: Vec<Vec<P64>> = vec![make_path(border_2d)];
    for h in &holes_adj { paths.push(make_path(h)); }
    let d = Delaunay::new(true);
    let tris = d.execute(&paths);

    // Post-process: remove triangles inside holes.
    // Two tests (centroid-only — edge midpoints excluded because valid triangles
    // adjacent to a hole share an edge with the hole boundary, landing their
    // midpoints exactly on the boundary which pt_in_poly counts as inside):
    //   1. Vertex-set: all 3 vertices belong to the same hole → remove.
    //   2. Centroid outside outer boundary or inside any hole → remove.
    let tris = if !holes_2d.is_empty() {
        let hole_vsets: Vec<std::collections::HashSet<(i64, i64)>> = paths[1..]
            .iter()
            .map(|h| h.iter().map(|p| (p.x, p.y)).collect())
            .collect();

        let pt_in_poly = |px: i64, py: i64, poly: &[P64]| -> bool {
            let mut inside = false;
            let n = poly.len();
            let mut j = n - 1;
            for i in 0..n {
                let (xi, yi) = (poly[i].x, poly[i].y);
                let (xj, yj) = (poly[j].x, poly[j].y);
                if (yi > py) != (yj > py) {
                    let xi_cross = xi as f64 + (py - yi) as f64 * (xj - xi) as f64 / (yj - yi) as f64;
                    if (px as f64) < xi_cross { inside = !inside; }
                }
                j = i;
            }
            inside
        };

        let pt_invalid = |px: i64, py: i64| -> bool {
            if !pt_in_poly(px, py, &paths[0]) { return true; }
            for h_path in &paths[1..] {
                if pt_in_poly(px, py, h_path) { return true; }
            }
            false
        };

        tris.into_iter().filter(|tri| {
            let t = [(tri[0].x, tri[0].y), (tri[1].x, tri[1].y), (tri[2].x, tri[2].y)];
            // Test 1: all 3 verts in same hole
            for vs in &hole_vsets {
                if vs.contains(&t[0]) && vs.contains(&t[1]) && vs.contains(&t[2]) {
                    return false;
                }
            }
            // Test 2: centroid
            let cx = (tri[0].x + tri[1].x + tri[2].x) / 3;
            let cy = (tri[0].y + tri[1].y + tri[2].y) / 3;
            !pt_invalid(cx, cy)
        }).collect()
    } else {
        tris
    };

    let mut out = Vec::new();
    for tri in tris {
        let mut f = [0usize; 3];
        let mut ok = true;
        for k in 0..3 {
            let key = (tri[k].x, tri[k].y);
            match pt_map.get(&key) { Some(&i) => f[k] = i, None => { ok = false; break; } }
        }
        if ok { out.push((f[0], f[1], f[2])); }
    }
    out
}

pub(crate) fn from_polygon_with_holes_impl(polylines: &[Polyline], is_2d: bool, is_first_boundary: bool) -> Mesh {
    use crate::session_config::SESSION_CONFIG;
    fn strip_close(mut pts: Vec<Point>) -> Vec<Point> {
        if pts.len() > 1 {
            let same = { let f = &pts[0]; let b = &pts[pts.len()-1];
                (f[0]-b[0]).abs() < 1e-12 && (f[1]-b[1]).abs() < 1e-12 && (f[2]-b[2]).abs() < 1e-12 };
            if same { pts.pop(); }
        }
        pts
    }
    if polylines.is_empty() { return Mesh::new(); }
    let mut border_idx = 0usize;
    if !is_first_boundary && polylines.len() > 1 {
        let mut max_diag = 0.0_f64;
        for (i, poly) in polylines.iter().enumerate() {
            let pts = poly.get_points();
            if pts.len() < 3 { continue; }
            let (mut minx, mut miny, mut minz) = (pts[0][0], pts[0][1], pts[0][2]);
            let (mut maxx, mut maxy, mut maxz) = (minx, miny, minz);
            for p in &pts {
                if p[0] < minx { minx = p[0]; } if p[0] > maxx { maxx = p[0]; }
                if p[1] < miny { miny = p[1]; } if p[1] > maxy { maxy = p[1]; }
                if p[2] < minz { minz = p[2]; } if p[2] > maxz { maxz = p[2]; }
            }
            let (dx, dy, dz) = (maxx - minx, maxy - miny, maxz - minz);
            let diag = (dx*dx + dy*dy + dz*dz).sqrt();
            if diag > max_diag { max_diag = diag; border_idx = i; }
        }
    }
    let mut border = strip_close(polylines[border_idx].get_points());
    if border.len() < 3 { return Mesh::new(); }
    let mut hole_pts_3d: Vec<Vec<Point>> = polylines.iter().enumerate()
        .filter(|(i, _)| *i != border_idx)
        .map(|(_, h)| strip_close(h.get_points())).filter(|h| h.len() >= 3).collect();
    let signed_area = |pts: &[Point]| -> f64 {
        let n = pts.len(); let mut a = 0.0;
        for i in 0..n { let j = (i+1)%n; a += pts[i][0]*pts[j][1] - pts[j][0]*pts[i][1]; }
        a * 0.5
    };
    let (mut boundary_2d, mut holes_2d) = if is_2d {
        let b2d: Vec<Point> = border.iter().map(|p| Point::new(p[0], p[1], 0.0)).collect();
        let h2d: Vec<Vec<Point>> = hole_pts_3d.iter().map(|h| h.iter().map(|p| Point::new(p[0], p[1], 0.0)).collect()).collect();
        (b2d, h2d)
    } else {
        let all_pts_for_plane: Vec<Point> = border.iter().chain(hole_pts_3d.iter().flatten()).cloned().collect();
        let (origin, xaxis, yaxis, _) = crate::polyline::Polyline::new(all_pts_for_plane).get_average_plane();
        let project_2d = |p: &Point| -> Point {
            let dx = p[0]-origin[0]; let dy = p[1]-origin[1]; let dz = p[2]-origin[2];
            Point::new(dx*xaxis[0]+dy*xaxis[1]+dz*xaxis[2], dx*yaxis[0]+dy*yaxis[1]+dz*yaxis[2], 0.0)
        };
        let b2d: Vec<Point> = border.iter().map(|p| project_2d(p)).collect();
        let h2d: Vec<Vec<Point>> = hole_pts_3d.iter().map(|h| h.iter().map(|p| project_2d(p)).collect()).collect();
        (b2d, h2d)
    };
    if signed_area(&boundary_2d) < 0.0 { border.reverse(); boundary_2d.reverse(); }
    for (hole, h2d) in hole_pts_3d.iter_mut().zip(holes_2d.iter_mut()) {
        if signed_area(h2d) > 0.0 { hole.reverse(); h2d.reverse(); }
    }
    let tris = cdt_triangulate(&boundary_2d, &holes_2d);
    let all_pts: Vec<Point> = border.iter().chain(hole_pts_3d.iter().flatten()).cloned().collect();
    let mut m = Mesh::new();
    let vkeys: Vec<usize> = all_pts.iter().map(|p| m.add_vertex(p.clone(), None)).collect();
    if SESSION_CONFIG.explode_mesh_faces() {
        for &(a, b, c) in &tris {
            m.add_face(vec![vkeys[a], vkeys[b], vkeys[c]], None);
        }
        return m;
    }
    let bvk: Vec<usize> = vkeys[..border.len()].to_vec();
    if let Some(fkey) = m.add_face(bvk, None) {
        if hole_pts_3d.is_empty() {
            let mut tri_list: Vec<[usize; 3]> = tris.iter()
                .filter(|&&(a, b, c)| vkeys[a] != vkeys[b] && vkeys[b] != vkeys[c] && vkeys[c] != vkeys[a])
                .map(|&(a, b, c)| [vkeys[a], vkeys[b], vkeys[c]])
                .collect();
            let covered: std::collections::HashSet<usize> = tri_list.iter().flatten().cloned().collect();
            let n_vk = border.len();
            for i in 0..n_vk {
                if !covered.contains(&vkeys[i]) {
                    tri_list.push([vkeys[(i+n_vk-1)%n_vk], vkeys[i], vkeys[(i+1)%n_vk]]);
                }
            }
            m.set_face_triangulation(fkey, tri_list);
        } else {
            let mut hole_rings: Vec<Vec<usize>> = Vec::new();
            let mut off = border.len();
            for h in &hole_pts_3d {
                hole_rings.push(vkeys[off..off+h.len()].to_vec());
                off += h.len();
            }
            m.face_holes.insert(fkey, hole_rings);
            let tri_list: Vec<[usize; 3]> = tris.iter()
                .filter(|&&(a, b, c)| vkeys[a] != vkeys[b] && vkeys[b] != vkeys[c] && vkeys[c] != vkeys[a])
                .map(|&(a, b, c)| [vkeys[a], vkeys[b], vkeys[c]])
                .collect();
            m.set_face_triangulation(fkey, tri_list);
        }
    }
    m
}

pub struct RemeshCDT;

impl RemeshCDT {
    /// polylines[0]=border, rest=holes (x,y used; z ignored). Strips closing duplicate.
    /// Returns (i,j,k) index triples into flat vertex array [border..., hole0..., hole1...].
    /// To build a Mesh from the result:
    /// ```ignore
    /// let border = Polyline::new(vec![Point::new(0.0,0.0,0.0), Point::new(4.0,0.0,0.0), Point::new(4.0,4.0,0.0), Point::new(0.0,4.0,0.0)]);
    /// let hole   = Polyline::new(vec![Point::new(1.0,1.0,0.0), Point::new(1.0,3.0,0.0), Point::new(3.0,3.0,0.0), Point::new(3.0,1.0,0.0)]);
    /// let tris = RemeshCDT::triangulate(&[border.clone(), hole.clone()]);
    /// let flat: Vec<Point> = border.get_points().into_iter().chain(hole.get_points()).collect();
    /// let mut m = Mesh::new();
    /// let vkeys: Vec<usize> = flat.iter().map(|p| m.add_vertex(p.clone(), None)).collect();
    /// for &(a, b, c) in &tris { m.add_face(vec![vkeys[a], vkeys[b], vkeys[c]], None); }
    /// ```
    pub fn triangulate(polylines: &[Polyline]) -> Vec<(usize, usize, usize)> {
        fn strip(pts: Vec<Point>) -> Vec<Point> {
            if pts.len() > 1 {
                let f = &pts[0]; let b = &pts[pts.len()-1];
                if (f[0]-b[0]).abs() < 1e-12 && (f[1]-b[1]).abs() < 1e-12 && (f[2]-b[2]).abs() < 1e-12 {
                    return pts[..pts.len()-1].to_vec();
                }
            }
            pts
        }
        if polylines.is_empty() { return vec![]; }
        let bpts = strip(polylines[0].get_points());
        let hpts_list: Vec<Vec<Point>> = polylines[1..].iter().map(|h| strip(h.get_points())).collect();
        cdt_triangulate(&bpts, &hpts_list)
    }

    /// Polylines → Mesh. is_2d=true skips plane projection. is_first_boundary=false detects border by largest bbox diagonal.
    pub fn from_polylines(polylines: &[Polyline], is_2d: bool, is_first_boundary: bool) -> Mesh {
        from_polygon_with_holes_impl(polylines, is_2d, is_first_boundary)
    }
}

