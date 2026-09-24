use crate::mesh::Mesh;
use crate::point::Point;
use crate::polyline::Polyline;
use crate::session_config::SESSION_CONFIG;
use crate::vector::Vector;
use std::cmp::Ordering;
use std::collections::HashMap;
use std::collections::HashSet;

// ═══════════════════════════════════════════════════════════════════════════
// Integer geometry
// ═══════════════════════════════════════════════════════════════════════════

const NULL_IDX: usize = usize::MAX;
const MAX_COORD64: f64 = 9e17;
const MAX_PRECISION: i32 = 6;

/// Round to the nearest int64.
fn to_int64(x: f64) -> i64 {
    x.round() as i64
}

/// Scale a 2D point to integer coordinates.
fn to_point64(p: &Point, scale: f64) -> [i64; 2] {
    [to_int64(p[0] * scale), to_int64(p[1] * scale)]
}

/// Sign of the turn p1 -> p2 -> p3.
fn cross_sign(p1: [i64; 2], p2: [i64; 2], p3: [i64; 2]) -> i32 {
    let cp = (p2[0] - p1[0]) as f64 * (p3[1] - p2[1]) as f64
        - (p2[1] - p1[1]) as f64 * (p3[0] - p2[0]) as f64;

    if cp > 0.0 {
        return 1;
    }

    if cp < 0.0 {
        return -1;
    }

    0
}

/// True when p1 -> p2 -> p3 turns left.
fn left_turning(p1: [i64; 2], p2: [i64; 2], p3: [i64; 2]) -> bool {
    cross_sign(p1, p2, p3) < 0
}

/// True when p1 -> p2 -> p3 turns right.
fn right_turning(p1: [i64; 2], p2: [i64; 2], p3: [i64; 2]) -> bool {
    cross_sign(p1, p2, p3) > 0
}

/// True when a is swept before b: higher y first, then lower x.
fn sweep_before(a: [i64; 2], b: [i64; 2]) -> bool {
    if a[1] == b[1] {
        return a[0] < b[0];
    }

    a[1] > b[1]
}

/// Squared distance between two integer points.
fn dist_sqr(a: [i64; 2], b: [i64; 2]) -> f64 {
    let dx = (a[0] - b[0]) as f64;
    let dy = (a[1] - b[1]) as f64;

    dx * dx + dy * dy
}

/// Positive when d lies inside the circumcircle of the counter-clockwise triangle a, b, c.
fn in_circle(a: [i64; 2], b: [i64; 2], c: [i64; 2], d: [i64; 2]) -> f64 {
    let m00 = (a[0] - d[0]) as f64;
    let m01 = (a[1] - d[1]) as f64;
    let m02 = m00 * m00 + m01 * m01;
    let m10 = (b[0] - d[0]) as f64;
    let m11 = (b[1] - d[1]) as f64;
    let m12 = m10 * m10 + m11 * m11;
    let m20 = (c[0] - d[0]) as f64;
    let m21 = (c[1] - d[1]) as f64;
    let m22 = m20 * m20 + m21 * m21;

    m00 * (m11 * m22 - m21 * m12) - m10 * (m01 * m22 - m21 * m02) + m20 * (m01 * m12 - m11 * m02)
}

/// Squared distance from p to the segment a-b.
fn dist_sqr_segment(p: [i64; 2], a: [i64; 2], b: [i64; 2]) -> f64 {
    let dx = (b[0] - a[0]) as f64;
    let dy = (b[1] - a[1]) as f64;
    let ax = (p[0] - a[0]) as f64;
    let ay = (p[1] - a[1]) as f64;
    let q = ax * dx + ay * dy;

    if q < 0.0 {
        return dist_sqr(p, a);
    }

    if q > dx * dx + dy * dy {
        return dist_sqr(p, b);
    }

    (ax * dy - dx * ay) * (ax * dy - dx * ay) / (dx * dx + dy * dy)
}

/// True when a1-a2 and b1-b2 cross strictly inside both segments.
fn segments_intersect(a1: [i64; 2], a2: [i64; 2], b1: [i64; 2], b2: [i64; 2]) -> bool {
    if a1 == b1 || a2 == b1 || a2 == b2 || a1 == b2 {
        return false;
    }

    let dy1 = (a2[1] - a1[1]) as f64;
    let dx1 = (a2[0] - a1[0]) as f64;
    let dy2 = (b2[1] - b1[1]) as f64;
    let dx2 = (b2[0] - b1[0]) as f64;
    let cp = dy1 * dx2 - dy2 * dx1;

    if cp == 0.0 {
        return false;
    }

    let t = (a1[0] - b1[0]) as f64 * dy2 - (a1[1] - b1[1]) as f64 * dx2;

    if t >= 0.0 && (cp < 0.0 || t >= cp) {
        return false;
    }

    if t < 0.0 && (cp > 0.0 || t <= cp) {
        return false;
    }

    let u = (a1[0] - b1[0]) as f64 * dy1 - (a1[1] - b1[1]) as f64 * dx1;

    if u >= 0.0 {
        return cp > 0.0 && u < cp;
    }

    cp < 0.0 && u > cp
}

/// Even-odd test of an integer point against an integer ring.
fn inside_path64(p: [i64; 2], poly: &[[i64; 2]]) -> bool {
    let mut inside = false;
    let n = poly.len();
    let mut j = n - 1;

    for i in 0..n {
        if (poly[i][1] > p[1]) != (poly[j][1] > p[1]) {
            let x = poly[i][0] as f64
                + (p[1] - poly[i][1]) as f64 * (poly[j][0] - poly[i][0]) as f64
                    / (poly[j][1] - poly[i][1]) as f64;

            if (p[0] as f64) < x {
                inside = !inside;
            }
        }

        j = i;
    }

    inside
}

/// Index before i on a ring of n.
fn prev_index(i: usize, n: usize) -> usize {
    if i == 0 {
        n - 1
    } else {
        i - 1
    }
}

/// Index after i on a ring of n.
fn next_index(i: usize, n: usize) -> usize {
    (i + 1) % n
}

/// Advance i to the next vertex that ends a rising run and starts a falling one; false when the path is flat.
fn find_loc_min(path: &[[i64; 2]], i: &mut usize) -> bool {
    let n = path.len();

    if n < 3 {
        return false;
    }

    let i0 = *i;
    let mut k = next_index(*i, n);

    while path[k][1] <= path[*i][1] {
        *i = k;
        k = next_index(k, n);

        if *i == i0 {
            return false;
        }
    }

    while path[k][1] >= path[*i][1] {
        *i = k;
        k = next_index(k, n);
    }

    true
}

// ═══════════════════════════════════════════════════════════════════════════
// Sweep graph
// ═══════════════════════════════════════════════════════════════════════════

/// Boundary side of an edge, or loose for a diagonal.
#[derive(Clone, Copy, PartialEq)]
enum EdgeKind {
    Loose,   // Diagonal between two boundary edges.
    Ascend,  // Boundary edge on the left side.
    Descend, // Boundary edge on the right side.
}

/// Sweep vertex with its incident edges.
struct Vertex {
    pt: [i64; 2],      // Integer position.
    edges: Vec<usize>, // Edges touching the vertex.
    inner_lm: bool,    // True at a local minimum of a hole.
}

/// Sweep edge with its endpoints, triangles and active-list links.
struct Edge {
    vl: usize,      // Left vertex.
    vr: usize,      // Right vertex.
    vb: usize,      // Bottom vertex.
    vt: usize,      // Top vertex.
    kind: EdgeKind, // Boundary side or loose diagonal.
    tri_a: usize,   // First triangle.
    tri_b: usize,   // Second triangle.
    active: bool,   // True while on the active list.
    next: usize,    // Next active edge.
    prev: usize,    // Previous active edge.
}

/// Triangle on three edges.
struct Tri {
    edges: [usize; 3], // Edge indices.
}

/// Sweep-line constrained Delaunay: boundary edges ascend on the left and descend on the right, diagonals are loose.
struct Delaunay {
    vs: Vec<Vertex>,      // Vertices.
    es: Vec<Edge>,        // Edges.
    ts: Vec<Tri>,         // Triangles.
    pending: Vec<usize>,  // Loose edges waiting to be legalized.
    horz: Vec<usize>,     // Horizontal edges deferred from the current row.
    loc_mins: Vec<usize>, // Hole local minima on the current row.
    lowermost: usize,     // Lowest vertex of the outer path.
    first_active: usize,  // Head of the active edge list.
}

impl Delaunay {
    /// Construct an empty sweep graph.
    fn new() -> Self {
        Delaunay {
            vs: Vec::new(),
            es: Vec::new(),
            ts: Vec::new(),
            pending: Vec::new(),
            horz: Vec::new(),
            loc_mins: Vec::new(),
            lowermost: NULL_IDX,
            first_active: NULL_IDX,
        }
    }

    /// True when both ends of e share a row.
    fn is_horizontal(&self, e: usize) -> bool {
        self.vs[self.es[e].vb].pt[1] == self.vs[self.es[e].vt].pt[1]
    }

    /// An edge is done with two triangles, or with one when it is a boundary edge.
    fn completed(&self, e: usize) -> bool {
        if self.es[e].tri_a == NULL_IDX {
            return false;
        }

        if self.es[e].tri_b != NULL_IDX {
            return true;
        }

        self.es[e].kind != EdgeKind::Loose
    }

    /// The endpoint of e that is not v.
    fn other(&self, e: usize, v: usize) -> usize {
        if self.es[e].vb == v {
            self.es[e].vt
        } else {
            self.es[e].vb
        }
    }

    /// Append a vertex and return its index.
    fn add_vertex(&mut self, p: [i64; 2]) -> usize {
        self.vs.push(Vertex {
            pt: p,
            edges: Vec::new(),
            inner_lm: false,
        });

        self.vs.len() - 1
    }

    /// Prepend e to the doubly-linked active list.
    fn add_active(&mut self, e: usize) {
        if self.es[e].active {
            return;
        }

        self.es[e].prev = NULL_IDX;
        self.es[e].next = self.first_active;
        self.es[e].active = true;

        if self.first_active != NULL_IDX {
            self.es[self.first_active].prev = e;
        }

        self.first_active = e;
    }

    /// Unlink e from the active list and from both endpoint edge lists.
    fn remove_active(&mut self, e: usize) {
        self.remove_from_vertex(self.es[e].vb, e);
        self.remove_from_vertex(self.es[e].vt, e);

        let prev = self.es[e].prev;
        let next = self.es[e].next;

        if next != NULL_IDX {
            self.es[next].prev = prev;
        }

        if prev != NULL_IDX {
            self.es[prev].next = next;
        }

        self.es[e].active = false;

        if self.first_active == e {
            self.first_active = next;
        }
    }

    /// Drop e from the edge list of v.
    fn remove_from_vertex(&mut self, v: usize, e: usize) {
        let edges = &mut self.vs[v].edges;

        if let Some(at) = edges.iter().position(|&x| x == e) {
            edges.remove(at);
        }
    }

    /// New edge between v1 and v2; loose edges go straight to the active list and the legalize queue.
    fn create_edge(&mut self, v1: usize, v2: usize, kind: EdgeKind) -> usize {
        let e = self.es.len();
        let p1 = self.vs[v1].pt;
        let p2 = self.vs[v2].pt;

        self.es.push(Edge {
            vl: if p1[0] <= p2[0] { v1 } else { v2 },
            vr: if p1[0] <= p2[0] { v2 } else { v1 },
            vb: if p1[1] < p2[1] { v2 } else { v1 },
            vt: if p1[1] < p2[1] { v1 } else { v2 },
            kind,
            tri_a: NULL_IDX,
            tri_b: NULL_IDX,
            active: false,
            next: NULL_IDX,
            prev: NULL_IDX,
        });

        self.vs[v1].edges.push(e);
        self.vs[v2].edges.push(e);

        if kind == EdgeKind::Loose {
            self.pending.push(e);
            self.add_active(e);
        }

        e
    }

    /// New triangle on three edges; an edge leaves the active list when it is completed.
    fn create_tri(&mut self, e1: usize, e2: usize, e3: usize) -> usize {
        let t = self.ts.len();

        self.ts.push(Tri {
            edges: [e1, e2, e3],
        });

        for e in [e1, e2, e3] {
            if self.es[e].tri_a != NULL_IDX {
                self.es[e].tri_b = t;
                self.remove_active(e);
            } else {
                self.es[e].tri_a = t;

                if self.es[e].kind != EdgeKind::Loose {
                    self.remove_active(e);
                }
            }
        }

        t
    }

    /// Shorten long_e to end at short_e's top and continue it with a new edge to the old top.
    fn split_edge(&mut self, long_e: usize, short_e: usize) {
        let old_t = self.es[long_e].vt;
        let new_t = self.es[short_e].vt;

        self.remove_from_vertex(old_t, long_e);
        self.es[long_e].vt = new_t;

        if self.es[long_e].vl == old_t {
            self.es[long_e].vl = new_t;
        } else {
            self.es[long_e].vr = new_t;
        }

        self.vs[new_t].edges.push(long_e);
        self.create_edge(new_t, old_t, self.es[long_e].kind);
    }

    /// Split the longer of two collinear non-horizontal edges leaving v downwards.
    fn split_collinear(&mut self, v: usize) {
        let snapshot = self.vs[v].edges.clone();

        for &e1 in &snapshot {
            if self.is_horizontal(e1) || self.es[e1].vb != v {
                continue;
            }

            for &e2 in &snapshot {
                if e2 == e1 || self.es[e2].vb != v {
                    continue;
                }

                let t1 = self.vs[self.es[e1].vt].pt;
                let t2 = self.vs[self.es[e2].vt].pt;

                if t1[1] == t2[1] || cross_sign(t1, self.vs[v].pt, t2) != 0 {
                    continue;
                }

                if t1[1] < t2[1] {
                    self.split_edge(e1, e2);
                } else {
                    self.split_edge(e2, e1);
                }

                break;
            }
        }
    }

    /// Merge coincident vertices that are neighbours in sweep order into the first one.
    fn merge_duplicates(&mut self, order: &[usize]) {
        let mut v1 = order[0];

        for &v2 in &order[1..] {
            if self.vs[v1].pt != self.vs[v2].pt {
                v1 = v2;
                continue;
            }

            if !self.vs[v1].inner_lm || !self.vs[v2].inner_lm {
                self.vs[v1].inner_lm = false;
            }

            let moved = std::mem::take(&mut self.vs[v2].edges);

            for &e in &moved {
                if self.es[e].vb == v2 {
                    self.es[e].vb = v1;
                } else {
                    self.es[e].vt = v1;
                }

                if self.es[e].vl == v2 {
                    self.es[e].vl = v1;
                } else {
                    self.es[e].vr = v1;
                }
            }

            self.vs[v1].edges.extend(moved);
            self.split_collinear(v1);
        }
    }

    /// Edge of v1 that reaches v2, a loose one or one of the preferred kind first.
    fn find_linking_edge(&self, v1: usize, v2: usize, prefer_ascend: bool) -> usize {
        let mut res = NULL_IDX;

        for &e in &self.vs[v1].edges {
            if self.es[e].vl != v2 && self.es[e].vr != v2 {
                continue;
            }

            if self.es[e].kind == EdgeKind::Loose
                || (self.es[e].kind == EdgeKind::Ascend) == prefer_ascend
            {
                return e;
            }

            res = e;
        }

        res
    }

    /// True when an active horizontal edge lies on the row of v1 between v1 and v2.
    fn horizontal_between(&self, v1: usize, v2: usize) -> bool {
        let y = self.vs[v1].pt[1];
        let lo = self.vs[v1].pt[0].min(self.vs[v2].pt[0]);
        let hi = self.vs[v1].pt[0].max(self.vs[v2].pt[0]);
        let mut e = self.first_active;

        while e != NULL_IDX {
            let pl = self.vs[self.es[e].vl].pt;
            let pr = self.vs[self.es[e].vr].pt;

            if pl[1] == y
                && pr[1] == y
                && pl[0] >= lo
                && pr[0] <= hi
                && (pl[0] != lo || pl[0] != hi)
            {
                return true;
            }

            e = self.es[e].next;
        }

        false
    }

    /// Nearest active edge spanning the x of v_above below it, NULL_IDX when there is none.
    fn edge_below(&self, v_above: usize) -> usize {
        let pa = self.vs[v_above].pt;
        let mut best = NULL_IDX;
        let mut best_d = -1.0;
        let mut e = self.first_active;

        while e != NULL_IDX {
            let pl = self.vs[self.es[e].vl].pt;
            let pr = self.vs[self.es[e].vr].pt;
            let spans = pl[0] <= pa[0] && pr[0] >= pa[0] && self.vs[self.es[e].vb].pt[1] >= pa[1];

            if spans
                && self.es[e].vb != v_above
                && self.es[e].vt != v_above
                && !left_turning(pl, pa, pr)
            {
                let d = dist_sqr_segment(pa, pl, pr);

                if best == NULL_IDX || d < best_d {
                    best = e;
                    best_d = d;
                }
            }

            e = self.es[e].next;
        }

        best
    }

    /// Endpoint of e_below visible from v_above, moved past every active edge crossing the connection.
    fn visible_vertex(&self, e_below: usize, v_above: usize) -> usize {
        let pa = self.vs[v_above].pt;
        let mut best = if self.vs[self.es[e_below].vt].pt[1] <= pa[1] {
            self.es[e_below].vb
        } else {
            self.es[e_below].vt
        };
        let left = self.vs[best].pt[0] < pa[0];
        let mut e = self.first_active;

        while e != NULL_IDX {
            let pb = self.vs[best].pt;
            let pl = self.vs[self.es[e].vl].pt;
            let pr = self.vs[self.es[e].vr].pt;
            let eb = self.vs[self.es[e].vb].pt;
            let et = self.vs[self.es[e].vt].pt;
            let spans = if left {
                pr[0] > pb[0] && pl[0] < pa[0]
            } else {
                pr[0] < pb[0] && pl[0] > pa[0]
            };

            if spans && eb[1] > pa[1] && et[1] < pb[1] && segments_intersect(eb, et, pb, pa) {
                best = if et[1] > pa[1] {
                    self.es[e].vt
                } else {
                    self.es[e].vb
                };
            }

            e = self.es[e].next;
        }

        best
    }

    /// Connect a hole local minimum to the visible vertex of the nearest active edge below it.
    fn create_loc_min_edge(&mut self, v_above: usize) -> usize {
        let below = self.edge_below(v_above);

        if below == NULL_IDX {
            return NULL_IDX;
        }

        let visible = self.visible_vertex(below, v_above);

        self.create_edge(visible, v_above, EdgeKind::Loose)
    }

    /// Tightest active fan candidate around pivot on the left (or right) side of edge, turns read with the side as sign; NULL_IDX when there is none.
    fn fan_vertex(&self, edge: usize, pivot: usize, left: bool) -> (usize, usize) {
        let v = self.other(edge, pivot);
        let side = if left { 1 } else { -1 };
        let mut v_alt = NULL_IDX;
        let mut e_alt = NULL_IDX;

        for &e in &self.vs[pivot].edges {
            if e == edge || !self.es[e].active {
                continue;
            }

            let vx = self.other(e, pivot);

            if vx == v {
                continue;
            }

            let sign = side * cross_sign(self.vs[v].pt, self.vs[pivot].pt, self.vs[vx].pt);

            if sign == 0 {
                if (self.vs[v].pt[0] > self.vs[pivot].pt[0])
                    == (self.vs[pivot].pt[0] > self.vs[vx].pt[0])
                {
                    continue;
                }
            } else if sign > 0
                || (v_alt != NULL_IDX
                    && side * cross_sign(self.vs[vx].pt, self.vs[pivot].pt, self.vs[v_alt].pt) >= 0)
            {
                continue;
            }

            v_alt = vx;
            e_alt = e;
        }

        (v_alt, e_alt)
    }

    /// Fan triangles around pivot on one side of edge, walking onto each new diagonal, never below min_y.
    fn triangulate_fan(&mut self, edge: usize, pivot: usize, min_y: i64, left: bool) {
        let mut edge = edge;
        let mut pivot = pivot;
        let max_fan = 2 * self.vs.len() + 2;

        for _step in 0..max_fan {
            let (v_alt, e_alt) = self.fan_vertex(edge, pivot, left);

            if v_alt == NULL_IDX || self.vs[v_alt].pt[1] < min_y {
                return;
            }

            let kind_below = if left {
                EdgeKind::Ascend
            } else {
                EdgeKind::Descend
            };
            let kind_above = if left {
                EdgeKind::Descend
            } else {
                EdgeKind::Ascend
            };

            if self.vs[v_alt].pt[1] < self.vs[pivot].pt[1] && self.es[e_alt].kind == kind_below {
                return;
            }

            if self.vs[v_alt].pt[1] > self.vs[pivot].pt[1] && self.es[e_alt].kind == kind_above {
                return;
            }

            let v = self.other(edge, pivot);
            let prefer_ascend = if left {
                self.vs[v_alt].pt[1] < self.vs[v].pt[1]
            } else {
                self.vs[v_alt].pt[1] > self.vs[v].pt[1]
            };
            let mut ex = self.find_linking_edge(v_alt, v, prefer_ascend);

            if ex == NULL_IDX {
                if self.vs[v_alt].pt[1] == self.vs[v].pt[1]
                    && self.vs[v].pt[1] == min_y
                    && self.horizontal_between(v_alt, v)
                {
                    return;
                }

                ex = self.create_edge(v_alt, v, EdgeKind::Loose);
            }

            if left {
                self.create_tri(edge, e_alt, ex);
            } else {
                self.create_tri(edge, ex, e_alt);
            }

            if self.completed(ex) {
                return;
            }

            edge = ex;
            pivot = v_alt;
        }
    }

    /// Of the two edges of tri other than edge, a gets the one touching vl and b the other; returns the far vertex.
    fn opposite(&self, tri: usize, edge: usize, vl: usize) -> (usize, usize, usize) {
        let mut far = NULL_IDX;
        let mut a = NULL_IDX;
        let mut b = NULL_IDX;

        for e in self.ts[tri].edges {
            if e == edge {
                continue;
            }

            if self.es[e].vl == vl {
                a = e;
                far = self.es[e].vr;
            } else if self.es[e].vr == vl {
                a = e;
                far = self.es[e].vl;
            } else {
                b = e;
            }
        }

        (far, a, b)
    }

    /// Give tri the edges (edge, e1, e2) and move e1/e2 from the other triangle onto it.
    fn rewire(&mut self, tri: usize, other: usize, edge: usize, e1: usize, e2: usize) {
        self.ts[tri].edges = [edge, e1, e2];

        for e in [e1, e2] {
            if self.es[e].kind == EdgeKind::Loose {
                self.pending.push(e);
            }

            if self.es[e].tri_a == tri || self.es[e].tri_b == tri {
                continue;
            }

            if self.es[e].tri_a == other {
                self.es[e].tri_a = tri;
            } else if self.es[e].tri_b == other {
                self.es[e].tri_b = tri;
            }
        }
    }

    /// Flip edge when the far vertex of one triangle lies inside the circumcircle of the other.
    fn force_legal(&mut self, edge: usize) {
        let ta = self.es[edge].tri_a;
        let tb = self.es[edge].tri_b;

        if ta == NULL_IDX || tb == NULL_IDX {
            return;
        }

        let vl = self.es[edge].vl;
        let vr = self.es[edge].vr;
        let (va, a1, b1) = self.opposite(ta, edge, vl);
        let (vb, a2, b2) = self.opposite(tb, edge, vl);

        if va == NULL_IDX || vb == NULL_IDX || b1 == NULL_IDX || b2 == NULL_IDX {
            return;
        }

        if cross_sign(self.vs[va].pt, self.vs[vl].pt, self.vs[vr].pt) == 0 {
            return;
        }

        let ict = in_circle(
            self.vs[va].pt,
            self.vs[vl].pt,
            self.vs[vr].pt,
            self.vs[vb].pt,
        );

        if ict == 0.0
            || right_turning(self.vs[va].pt, self.vs[vl].pt, self.vs[vr].pt) == (ict < 0.0)
        {
            return;
        }

        self.es[edge].vl = va;
        self.es[edge].vr = vb;
        self.rewire(ta, tb, edge, a1, a2);
        self.rewire(tb, ta, edge, b1, b2);
    }

    /// Walk the path from i back round to i0 creating boundary edges; false when the step budget of a degenerate path is blown.
    fn walk_path(&mut self, path: &[[i64; 2]], i0: usize, i: usize, v0: usize) -> bool {
        let n = path.len();
        let budget = 16 * n + 256;
        let mut steps = 0;
        let mut v_prev = v0;
        let mut i = i;

        while steps < budget {
            steps += 1;

            self.loc_mins.push(v_prev);

            if self.lowermost == NULL_IDX
                || sweep_before(self.vs[v_prev].pt, self.vs[self.lowermost].pt)
            {
                self.lowermost = v_prev;
            }

            let mut i_next = next_index(i, n);

            if cross_sign(self.vs[v_prev].pt, path[i], path[i_next]) == 0 {
                i = i_next;
                continue;
            }

            while path[i][1] <= self.vs[v_prev].pt[1] {
                steps += 1;

                if steps > budget {
                    return false;
                }

                let v = self.add_vertex(path[i]);

                self.create_edge(v_prev, v, EdgeKind::Ascend);
                v_prev = v;
                i = i_next;
                i_next = next_index(i, n);

                while cross_sign(self.vs[v_prev].pt, path[i], path[i_next]) == 0 {
                    steps += 1;

                    if steps > budget {
                        return false;
                    }

                    i = i_next;
                    i_next = next_index(i, n);
                }
            }

            let mut v_prev_prev = v_prev;

            while i != i0 && path[i][1] >= self.vs[v_prev].pt[1] {
                steps += 1;

                if steps > budget {
                    return false;
                }

                let v = self.add_vertex(path[i]);

                self.create_edge(v, v_prev, EdgeKind::Descend);
                v_prev_prev = v_prev;
                v_prev = v;
                i = i_next;
                i_next = next_index(i, n);

                while cross_sign(self.vs[v_prev].pt, path[i], path[i_next]) == 0 {
                    steps += 1;

                    if steps > budget {
                        return false;
                    }

                    i = i_next;
                    i_next = next_index(i, n);
                }
            }

            if i == i0 {
                self.create_edge(v0, v_prev, EdgeKind::Descend);

                return true;
            }

            if left_turning(self.vs[v_prev_prev].pt, self.vs[v_prev].pt, path[i]) {
                self.vs[v_prev].inner_lm = true;
            }
        }

        false
    }

    /// Detach the edges of every vertex added since start.
    fn discard(&mut self, start: usize) {
        for v in start..self.vs.len() {
            self.vs[v].edges.clear();
        }
    }

    /// Register one closed path; paths that are flat, degenerate or too tiny to hold a triangle are dropped.
    fn add_path(&mut self, path: &[[i64; 2]]) {
        let n = path.len();
        let mut i = 0;

        if !find_loc_min(path, &mut i) {
            return;
        }

        let i0 = i;
        let mut i_prev = prev_index(i, n);

        while path[i_prev] == path[i] {
            i_prev = prev_index(i_prev, n);
        }

        let mut i_next = next_index(i, n);

        while cross_sign(path[i_prev], path[i], path[i_next]) == 0 {
            if !find_loc_min(path, &mut i) || i == i0 {
                return;
            }

            i_prev = prev_index(i, n);

            while path[i_prev] == path[i] {
                i_prev = prev_index(i_prev, n);
            }

            i_next = next_index(i, n);
        }

        let start = self.vs.len();
        let v0 = self.add_vertex(path[i]);

        if left_turning(path[i_prev], path[i], path[i_next]) {
            self.vs[v0].inner_lm = true;
        }

        if !self.walk_path(path, i0, i_next, v0) {
            self.discard(start);

            return;
        }

        let count = self.vs.len() - start;
        let tiny = count == 3
            && (dist_sqr(self.vs[start].pt, self.vs[start + 1].pt) <= 1.0
                || dist_sqr(self.vs[start + 1].pt, self.vs[start + 2].pt) <= 1.0
                || dist_sqr(self.vs[start + 2].pt, self.vs[start].pt) <= 1.0);

        if count < 3 || tiny {
            self.discard(start);
        }
    }

    /// Register every path; false when none survives.
    fn add_paths(&mut self, paths: &[Vec<[i64; 2]>]) -> bool {
        let mut total = 0;

        for path in paths {
            total += path.len();
        }

        if total == 0 {
            return false;
        }

        self.vs.reserve(total);
        self.es.reserve(total);

        for path in paths {
            self.add_path(path);
        }

        self.vs.len() > 2
    }

    /// The outer path was wound clockwise: swap the hole flags and the boundary sides.
    fn flip_winding(&mut self) {
        for &v in &self.loc_mins {
            self.vs[v].inner_lm = !self.vs[v].inner_lm;
        }

        for e in &mut self.es {
            if e.kind == EdgeKind::Ascend {
                e.kind = EdgeKind::Descend;
            } else if e.kind == EdgeKind::Descend {
                e.kind = EdgeKind::Ascend;
            }
        }
    }

    /// Connect and fan the hole local minima collected on the finished row; false when one cannot be reached.
    fn sweep_loc_mins(&mut self, curr_y: i64) -> bool {
        while let Some(lm) = self.loc_mins.pop() {
            let e = self.create_loc_min_edge(lm);

            if e == NULL_IDX {
                return false;
            }

            let vb = self.es[e].vb;

            if self.is_horizontal(e) {
                self.triangulate_fan(e, vb, curr_y, self.es[e].vl == vb);
            } else {
                self.triangulate_fan(e, vb, curr_y, true);

                if !self.completed(e) {
                    self.triangulate_fan(e, vb, curr_y, false);
                }
            }

            if self.vs[lm].edges.len() < 2 {
                continue;
            }

            self.add_active(self.vs[lm].edges[0]);
            self.add_active(self.vs[lm].edges[1]);
        }

        true
    }

    /// Fan the horizontal edges deferred from the finished row.
    fn sweep_horizontals(&mut self, curr_y: i64) {
        while let Some(e) = self.horz.pop() {
            if self.completed(e) {
                continue;
            }

            if self.es[e].vb == self.es[e].vl {
                if self.es[e].kind == EdgeKind::Ascend {
                    self.triangulate_fan(e, self.es[e].vb, curr_y, true);
                }
            } else if self.es[e].kind == EdgeKind::Descend {
                self.triangulate_fan(e, self.es[e].vb, curr_y, false);
            }
        }
    }

    /// Activate the boundary edges starting at v and fan the ones ending at it.
    fn sweep_vertex(&mut self, v: usize) {
        for i in (0..self.vs[v].edges.len()).rev() {
            if i >= self.vs[v].edges.len() {
                continue;
            }

            let e = self.vs[v].edges[i];

            if self.completed(e) || self.es[e].kind == EdgeKind::Loose {
                continue;
            }

            if self.is_horizontal(e) {
                self.horz.push(e);
            }

            if v == self.es[e].vb {
                if !self.vs[v].inner_lm {
                    self.add_active(e);
                }
            } else if !self.is_horizontal(e) {
                self.triangulate_fan(
                    e,
                    self.es[e].vb,
                    self.vs[v].pt[1],
                    self.es[e].kind == EdgeKind::Ascend,
                );
            }
        }
    }

    /// Sweep the vertices top to bottom filling triangles row by row; false when a hole cannot be connected.
    fn sweep(&mut self, order: &[usize]) -> bool {
        let mut curr_y = self.vs[order[0]].pt[1];

        for &v in order {
            if self.vs[v].edges.is_empty() {
                continue;
            }

            if self.vs[v].pt[1] != curr_y {
                if !self.sweep_loc_mins(curr_y) {
                    return false;
                }

                self.sweep_horizontals(curr_y);
                curr_y = self.vs[v].pt[1];
            }

            self.sweep_vertex(v);

            if self.vs[v].inner_lm {
                self.loc_mins.push(v);
            }
        }

        while let Some(e) = self.horz.pop() {
            if !self.completed(e) && self.es[e].vb == self.es[e].vl {
                self.triangulate_fan(e, self.es[e].vb, curr_y, true);
            }
        }

        true
    }

    /// Flip loose edges until Delaunay, capped so near-cocircular integer points cannot flip-flop forever.
    fn legalize(&mut self) {
        let max_flips = 64 * self.vs.len() + 4096;

        for _flips in 0..max_flips {
            let Some(e) = self.pending.pop() else {
                return;
            };

            self.force_legal(e);
        }
    }

    /// Both ends of edge 0 and the far end of edge 1.
    fn tri_points(&self, t: &Tri) -> [[i64; 2]; 3] {
        let e0 = &self.es[t.edges[0]];
        let e1 = &self.es[t.edges[1]];
        let p0 = self.vs[e0.vl].pt;
        let p1 = self.vs[e0.vr].pt;
        let p2 = if self.vs[e1.vl].pt == p0 || self.vs[e1.vl].pt == p1 {
            self.vs[e1.vr].pt
        } else {
            self.vs[e1.vl].pt
        };

        [p0, p1, p2]
    }

    /// Counter-clockwise triangles, flat ones dropped.
    fn triangles(&self) -> Vec<[[i64; 2]; 3]> {
        let mut res = Vec::with_capacity(self.ts.len());

        for t in &self.ts {
            let mut p = self.tri_points(t);
            let sign = cross_sign(p[0], p[1], p[2]);

            if sign == 0 {
                continue;
            }

            if sign < 0 {
                p.swap(0, 2);
            }

            res.push(p);
        }

        res
    }

    /// Triangles of the paths, empty when they hold no polygon or a hole cannot be connected.
    fn execute(&mut self, paths: &[Vec<[i64; 2]>]) -> Vec<[[i64; 2]; 3]> {
        if !self.add_paths(paths) {
            return Vec::new();
        }

        if self.vs[self.lowermost].inner_lm {
            self.flip_winding();
        }

        self.loc_mins.clear();

        let mut order: Vec<usize> = (0..self.vs.len()).collect();
        order.sort_by(|&a, &b| {
            if sweep_before(self.vs[a].pt, self.vs[b].pt) {
                Ordering::Less
            } else if sweep_before(self.vs[b].pt, self.vs[a].pt) {
                Ordering::Greater
            } else {
                Ordering::Equal
            }
        });

        self.merge_duplicates(&order);

        if !self.sweep(&order) {
            return Vec::new();
        }

        self.legalize();

        self.triangles()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Triangulation
// ═══════════════════════════════════════════════════════════════════════════

/// Power of ten keeping the largest coordinate inside int64 headroom.
fn cdt_scale(border_2d: &[Point], holes_2d: &[Vec<Point>]) -> f64 {
    let mut max_coord = 1.0f64;

    for p in border_2d {
        max_coord = max_coord.max(p[0].abs()).max(p[1].abs());
    }

    for hole in holes_2d {
        for p in hole {
            max_coord = max_coord.max(p[0].abs()).max(p[1].abs());
        }
    }

    let mut precision = MAX_PRECISION;

    while precision > 0 && max_coord * 10f64.powi(precision) > MAX_COORD64 {
        precision -= 1;
    }

    10f64.powi(precision)
}

/// Hole rows sharing an integer y with a border row move one unit down so the sweep never sees a collinear constraint.
fn shift_hole_rows(border_2d: &[Point], holes_2d: &[Vec<Point>], scale: f64) -> Vec<Vec<Point>> {
    let mut border_ys: HashSet<i64> = HashSet::new();

    for p in border_2d {
        border_ys.insert(to_int64(p[1] * scale));
    }

    let mut holes = holes_2d.to_vec();

    for hole in &mut holes {
        for p in hole.iter_mut() {
            let iy = to_int64(p[1] * scale);

            if border_ys.contains(&iy) {
                p[1] = (iy - 1) as f64 / scale;
            }
        }
    }

    holes
}

/// Integer ring, closing duplicate dropped.
fn to_path64(pts: &[Point], scale: f64) -> Vec<[i64; 2]> {
    let mut path = Vec::with_capacity(pts.len());

    for p in pts {
        path.push(to_point64(p, scale));
    }

    if path.len() > 1 && path[0] == path[path.len() - 1] {
        path.pop();
    }

    path
}

/// Index of every integer point in the flat list [border..., hole0..., hole1...], first occurrence wins.
fn index_map(border_2d: &[Point], holes_2d: &[Vec<Point>], scale: f64) -> HashMap<[i64; 2], usize> {
    let mut indices: HashMap<[i64; 2], usize> = HashMap::new();
    let mut index = 0;

    for p in border_2d {
        indices.entry(to_point64(p, scale)).or_insert(index);
        index += 1;
    }

    for hole in holes_2d {
        for p in hole {
            indices.entry(to_point64(p, scale)).or_insert(index);
            index += 1;
        }
    }

    indices
}

/// A triangle lies in a hole when all its corners are on one hole ring or its centroid is outside the border or inside a hole.
fn inside_hole(
    tri: &[[i64; 2]; 3],
    paths: &[Vec<[i64; 2]>],
    hole_sets: &[HashSet<[i64; 2]>],
) -> bool {
    for set in hole_sets {
        if set.contains(&tri[0]) && set.contains(&tri[1]) && set.contains(&tri[2]) {
            return true;
        }
    }

    let c = [
        (tri[0][0] + tri[1][0] + tri[2][0]) / 3,
        (tri[0][1] + tri[1][1] + tri[2][1]) / 3,
    ];

    if !inside_path64(c, &paths[0]) {
        return true;
    }

    for path in &paths[1..] {
        if inside_path64(c, path) {
            return true;
        }
    }

    false
}

/// Drop the triangles the sweep filled inside the holes; edge midpoints are not tested because valid triangles touch the hole rings.
fn remove_hole_triangles(tris: &mut Vec<[[i64; 2]; 3]>, paths: &[Vec<[i64; 2]>]) {
    let mut hole_sets: Vec<HashSet<[i64; 2]>> = Vec::new();

    for path in &paths[1..] {
        hole_sets.push(path.iter().copied().collect());
    }

    let mut kept = Vec::with_capacity(tris.len());

    for tri in tris.iter() {
        if !inside_hole(tri, paths, &hole_sets) {
            kept.push(*tri);
        }
    }

    *tris = kept;
}

/// Corner indices into the flat list, triangles with an unknown corner dropped.
fn to_indices(
    tris: &[[[i64; 2]; 3]],
    indices: &HashMap<[i64; 2], usize>,
) -> Vec<(usize, usize, usize)> {
    let mut out = Vec::with_capacity(tris.len());

    for tri in tris {
        let mut f = [0usize; 3];
        let mut known = true;

        for k in 0..3 {
            match indices.get(&tri[k]) {
                Some(&i) => f[k] = i,
                None => known = false,
            }
        }

        if known {
            out.push((f[0], f[1], f[2]));
        }
    }

    out
}

// ═══════════════════════════════════════════════════════════════════════════
// Mesh assembly
// ═══════════════════════════════════════════════════════════════════════════

/// Polyline points without the closing duplicate.
fn strip_close(polyline: &Polyline) -> Vec<Point> {
    let mut pts = polyline.get_points();

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

/// Signed area of a 2D ring, positive when counter-clockwise.
pub(crate) fn signed_area(pts: &[Point]) -> f64 {
    let mut area = 0.0;
    let n = pts.len();

    for i in 0..n {
        let j = (i + 1) % n;

        area += pts[i][0] * pts[j][1] - pts[j][0] * pts[i][1];
    }

    area * 0.5
}

/// Index of the polyline with the largest bounding-box diagonal.
fn border_index(polylines: &[Polyline]) -> usize {
    let mut border = 0;
    let mut max_diag = 0.0;

    for (i, polyline) in polylines.iter().enumerate() {
        let pts = polyline.get_points();

        if pts.len() < 3 {
            continue;
        }

        let mut lo = pts[0].clone();
        let mut hi = pts[0].clone();

        for p in &pts {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }

        let diag = lo.distance(&hi, None);

        if diag > max_diag {
            max_diag = diag;
            border = i;
        }
    }

    border
}

/// Plane coordinates of the points in the frame (origin, xaxis, yaxis).
pub(crate) fn project_2d(
    pts: &[Point],
    origin: &Point,
    xaxis: &Vector,
    yaxis: &Vector,
) -> Vec<Point> {
    let mut out = Vec::with_capacity(pts.len());

    for p in pts {
        let dx = p[0] - origin[0];
        let dy = p[1] - origin[1];
        let dz = p[2] - origin[2];

        out.push(Point::new(
            dx * xaxis[0] + dy * xaxis[1] + dz * xaxis[2],
            dx * yaxis[0] + dy * yaxis[1] + dz * yaxis[2],
            0.0,
        ));
    }

    out
}

/// Ear triangles for border vertices no triangle touches, so every vertex is drawn.
fn cover_missing(tri_list: &mut Vec<[usize; 3]>, vkeys: &[usize], n: usize) {
    let mut covered: HashSet<usize> = HashSet::new();

    for t in tri_list.iter() {
        covered.extend(t);
    }

    for m in 0..n {
        if !covered.contains(&vkeys[m]) {
            tri_list.push([vkeys[(m + n - 1) % n], vkeys[m], vkeys[(m + 1) % n]]);
        }
    }
}

/// One face over the border with the holes as face holes, or one face per triangle under SESSION_CONFIG.explode_mesh_faces.
fn build_mesh(border: &[Point], holes: &[Vec<Point>], tris: &[(usize, usize, usize)]) -> Mesh {
    let mut mesh = Mesh::new();
    let mut vkeys = Vec::new();

    for p in border {
        vkeys.push(mesh.add_vertex(p.clone(), None));
    }

    for hole in holes {
        for p in hole {
            vkeys.push(mesh.add_vertex(p.clone(), None));
        }
    }

    if SESSION_CONFIG.read().explode_mesh_faces {
        for &(a, b, c) in tris {
            mesh.add_face(vec![vkeys[a], vkeys[b], vkeys[c]], None);
        }

        return mesh;
    }

    let ring = vkeys[..border.len()].to_vec();
    let Some(fkey) = mesh.add_face(ring, None) else {
        return mesh;
    };

    let mut tri_list: Vec<[usize; 3]> = Vec::new();

    for &(a, b, c) in tris {
        let f = [vkeys[a], vkeys[b], vkeys[c]];

        if f[0] != f[1] && f[1] != f[2] && f[2] != f[0] {
            tri_list.push(f);
        }
    }

    if holes.is_empty() {
        cover_missing(&mut tri_list, &vkeys, border.len());
    } else {
        let mut hole_rings: Vec<Vec<usize>> = Vec::new();
        let mut off = border.len();

        for hole in holes {
            hole_rings.push(vkeys[off..off + hole.len()].to_vec());
            off += hole.len();
        }

        mesh.set_face_holes(fkey, hole_rings);
    }

    mesh.set_face_triangulation(fkey, tri_list);

    mesh
}

// ═══════════════════════════════════════════════════════════════════════════
// RemeshCDT
// ═══════════════════════════════════════════════════════════════════════════

/// Triangle index triples of a counter-clockwise 2D border with clockwise holes into the flat list [border..., hole0..., hole1...].
pub fn cdt_triangulate(border_2d: &[Point], holes_2d: &[Vec<Point>]) -> Vec<(usize, usize, usize)> {
    let scale = cdt_scale(border_2d, holes_2d);
    let holes = shift_hole_rows(border_2d, holes_2d, scale);
    let mut paths = vec![to_path64(border_2d, scale)];

    for hole in &holes {
        paths.push(to_path64(hole, scale));
    }

    let mut delaunay = Delaunay::new();
    let mut tris = delaunay.execute(&paths);

    if !holes.is_empty() {
        remove_hole_triangles(&mut tris, &paths);
    }

    to_indices(&tris, &index_map(border_2d, &holes, scale))
}

/// Constrained Delaunay triangulation of a border polyline with hole polylines.
pub struct RemeshCDT;

impl RemeshCDT {
    // ═══════════════════════════════════════════════════════════════════════════
    // Triangulation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Triangle index triples into the flat list [border..., hole0..., hole1...], closing duplicates stripped.
    pub fn triangulate(polylines: &[Polyline]) -> Vec<(usize, usize, usize)> {
        if polylines.is_empty() {
            return Vec::new();
        }

        let border = strip_close(&polylines[0]);

        if border.len() < 3 {
            return Vec::new();
        }

        let mut border_2d = Vec::new();

        for p in &border {
            border_2d.push(Point::new(p[0], p[1], 0.0));
        }

        let mut holes_2d = Vec::new();

        for polyline in &polylines[1..] {
            let mut hole_2d = Vec::new();

            for p in strip_close(polyline) {
                hole_2d.push(Point::new(p[0], p[1], 0.0));
            }

            holes_2d.push(hole_2d);
        }

        cdt_triangulate(&border_2d, &holes_2d)
    }

    /// Mesh of one face with holes, or one face per triangle under SESSION_CONFIG.explode_mesh_faces; is_2d skips the plane projection, is_first_boundary=false picks the border by largest bbox diagonal.
    pub fn from_polylines(polylines: &[Polyline], is_2d: bool, is_first_boundary: bool) -> Mesh {
        if polylines.is_empty() {
            return Mesh::new();
        }

        let border_idx = if is_first_boundary || polylines.len() == 1 {
            0
        } else {
            border_index(polylines)
        };
        let mut border = strip_close(&polylines[border_idx]);

        if border.len() < 3 {
            return Mesh::new();
        }

        let mut holes: Vec<Vec<Point>> = Vec::new();

        for (i, polyline) in polylines.iter().enumerate() {
            if i == border_idx {
                continue;
            }

            let hole = strip_close(polyline);

            if hole.len() >= 3 {
                holes.push(hole);
            }
        }

        let mut origin = Point::new(0.0, 0.0, 0.0);
        let mut xaxis = Vector::new(1.0, 0.0, 0.0);
        let mut yaxis = Vector::new(0.0, 1.0, 0.0);

        if !is_2d {
            let mut all_pts = border.clone();

            for hole in &holes {
                all_pts.extend(hole.iter().cloned());
            }

            (origin, xaxis, yaxis, _) = Polyline::new(all_pts).get_average_plane();
        }

        let mut border_2d = project_2d(&border, &origin, &xaxis, &yaxis);

        if signed_area(&border_2d) < 0.0 {
            border.reverse();
            border_2d.reverse();
        }

        let mut holes_2d = Vec::new();

        for hole in &mut holes {
            let mut hole_2d = project_2d(hole, &origin, &xaxis, &yaxis);

            if signed_area(&hole_2d) > 0.0 {
                hole.reverse();
                hole_2d.reverse();
            }

            holes_2d.push(hole_2d);
        }

        build_mesh(&border, &holes, &cdt_triangulate(&border_2d, &holes_2d))
    }
}
