use crate::tolerance::Tolerance;
use crate::Polyline;

// ═══════════════════════════════════════════════════════════════════════════
// Sweep structures
// ═══════════════════════════════════════════════════════════════════════════
const VF_NONE: u32 = 0; // Plain vertex.
const VF_LOCAL_MAX: u32 = 4; // Local maximum in y.
const VF_LOCAL_MIN: u32 = 8; // Local minimum in y.

#[derive(Clone, Copy, Default, PartialEq, Eq)]
struct BIVec2 {
    x: i64, // Scaled integer x.
    y: i64, // Scaled integer y.
}

fn v_cvt_to_i64(p: &[f64], scale: f64) -> BIVec2 {
    BIVec2 {
        x: (p[0] * scale).round_ties_even() as i64,
        y: (p[1] * scale).round_ties_even() as i64,
    }
}

fn v_cvt_to_dbl(dst: &mut Vec<f64>, pt: BIVec2, inv_scale: f64) {
    dst.push(pt.x as f64 * inv_scale);
    dst.push(pt.y as f64 * inv_scale);
    dst.push(0.0);
}

#[derive(Clone, Default)]
struct VVertex {
    pt: BIVec2,          // Scaled position.
    next: Option<usize>, // Next vertex of the ring.
    prev: Option<usize>, // Previous vertex of the ring.
    flags: u32,          // Local extremum flags.
}

#[derive(Clone)]
struct VLocalMinima {
    vertex: usize, // Vertex at the local minimum.
    polytype: i8,  // 0 subject, 1 clip.
}

#[derive(Clone, Default)]
struct VOutPt {
    pt: BIVec2,    // Scaled output position.
    next: usize,   // Next point of the output ring.
    prev: usize,   // Previous point of the output ring.
    outrec: usize, // Owning output ring.
    horz: bool,    // Starts a horizontal segment.
}

#[derive(Clone, Default)]
struct VOutRec {
    idx: usize,                // Index in the output list.
    front_edge: Option<usize>, // Edge adding points to the front.
    back_edge: Option<usize>,  // Edge adding points to the back.
    pts: Option<usize>,        // Entry point of the ring.
    owner: Option<usize>,      // Ring this one was merged into.
}

#[derive(Clone, Default)]
struct VActive {
    bot: BIVec2,                // Bottom of the edge.
    top: BIVec2,                // Top of the edge.
    curr_x: i64,                // x at the current scanline.
    dx: f64,                    // Inverse slope.
    wind_dx: i32,               // Winding direction, 1 or -1.
    wind_cnt: i32,              // Winding count of its own polytype.
    wind_cnt2: i32,             // Winding count of the other polytype.
    outrec: Option<usize>,      // Output ring the edge contributes to.
    prev_in_ael: Option<usize>, // Previous edge in the active edge list.
    next_in_ael: Option<usize>, // Next edge in the active edge list.
    prev_in_sel: Option<usize>, // Previous edge in the sorted edge list.
    next_in_sel: Option<usize>, // Next edge in the sorted edge list.
    jump: Option<usize>,        // Merge sort run boundary.
    vertex_top: Option<usize>,  // Vertex at the top of the edge.
    local_min: Option<usize>,   // Local minimum the bound starts from.
    is_left_bound: bool,        // Left or right bound of its minimum.
    join_with: i8,              // 0 none, 1 left, 2 right.
}

struct VIntersectNode {
    pt: BIVec2,   // Intersection point.
    edge1: usize, // Left edge.
    edge2: usize, // Right edge.
}

#[derive(Clone)]
struct VHorzSeg {
    left_op: usize,          // Left end of the segment.
    right_op: Option<usize>, // Right end of the segment.
    left_to_right: bool,     // Direction of the output ring.
}

struct VHorzJoin {
    op1: usize, // First point to join.
    op2: usize, // Second point to join.
}

struct ScanlineHeap {
    buf: Vec<i64>, // Max heap storage.
}

impl ScanlineHeap {
    fn clear(&mut self) {
        self.buf.clear();
    }

    fn empty(&self) -> bool {
        self.buf.is_empty()
    }

    fn push(&mut self, y: i64) {
        self.buf.push(y);
        let mut i = self.buf.len() - 1;

        while i > 0 {
            let p = (i - 1) / 2;

            if self.buf[p] >= self.buf[i] {
                break;
            }

            self.buf.swap(p, i);
            i = p;
        }
    }

    fn top(&self) -> i64 {
        self.buf[0]
    }

    fn pop(&mut self) {
        let last = self.buf.len() - 1;
        self.buf.swap(0, last);
        self.buf.pop();
        let sz = self.buf.len();
        let mut i = 0;

        loop {
            let l = 2 * i + 1;
            let r = l + 1;
            let mut m = i;

            if l < sz && self.buf[l] > self.buf[m] {
                m = l;
            }

            if r < sz && self.buf[r] > self.buf[m] {
                m = r;
            }

            if m == i {
                break;
            }

            self.buf.swap(i, m);
            i = m;
        }
    }
}

/// Arena of the sweep: every pointer of the C++ engine is an index into one of these pools.
struct VattiScratch {
    vtx_pool: Vec<VVertex>,               // Vertices of both inputs.
    act_pool: Vec<VActive>,               // Active edges.
    opt_pool: Vec<VOutPt>,                // Output points.
    orc_pool: Vec<VOutRec>,               // Output rings.
    locmin_list: Vec<VLocalMinima>,       // Local minima of both inputs.
    intersect_nodes: Vec<VIntersectNode>, // Intersections of the current scanbeam.
    horz_seg_list: Vec<VHorzSeg>,         // Horizontal output segments of the current scanline.
    horz_join_list: Vec<VHorzJoin>,       // Pending horizontal joins.
    outrec_list: Vec<usize>,              // Output rings in creation order.
    scanline_list: ScanlineHeap,          // Pending scanlines.
    actives: Option<usize>,               // Head of the active edge list.
    sel: Option<usize>,                   // Head of the sorted edge list.
    bot_y: i64,                           // Bottom of the current scanbeam.
    locmin_idx: usize,                    // Next local minimum to insert.
    succeeded: bool,                      // False once the sweep failed.
}

impl VattiScratch {
    fn new() -> Self {
        Self {
            vtx_pool: Vec::new(),
            act_pool: Vec::new(),
            opt_pool: Vec::new(),
            orc_pool: Vec::new(),
            locmin_list: Vec::new(),
            intersect_nodes: Vec::new(),
            horz_seg_list: Vec::new(),
            horz_join_list: Vec::new(),
            outrec_list: Vec::new(),
            scanline_list: ScanlineHeap { buf: Vec::new() },
            actives: None,
            sel: None,
            bot_y: 0,
            locmin_idx: 0,
            succeeded: true,
        }
    }

    fn reset(&mut self, total: usize) {
        self.vtx_pool.clear();
        self.act_pool.clear();
        self.opt_pool.clear();
        self.orc_pool.clear();
        self.locmin_list.clear();
        self.intersect_nodes.clear();
        self.horz_seg_list.clear();
        self.horz_join_list.clear();
        self.outrec_list.clear();
        self.scanline_list.clear();
        self.actives = None;
        self.sel = None;
        self.bot_y = 0;
        self.locmin_idx = 0;
        self.succeeded = true;

        self.vtx_pool.reserve(total + 4);
        self.act_pool.reserve(total * 2 + 4);
        self.opt_pool.reserve(total * 4);
        self.orc_pool.reserve(total);
        self.locmin_list.reserve(total);
        self.outrec_list.reserve(total);
        self.scanline_list.buf.reserve(total * 2);
    }

    fn new_active(&mut self) -> usize {
        self.act_pool.push(VActive::default());

        self.act_pool.len() - 1
    }

    fn new_outpt(&mut self, pt: BIVec2, outrec: usize) -> usize {
        let i = self.opt_pool.len();
        self.opt_pool.push(VOutPt {
            pt,
            next: i,
            prev: i,
            outrec,
            horz: false,
        });

        i
    }

    fn new_outrec(&mut self) -> usize {
        let i = self.orc_pool.len();
        self.orc_pool.push(VOutRec {
            idx: i,
            ..Default::default()
        });
        self.outrec_list.push(i);

        i
    }
}

thread_local! {
    static SCRATCH: std::cell::RefCell<VattiScratch> = std::cell::RefCell::new(VattiScratch::new());
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry helpers
// ═══════════════════════════════════════════════════════════════════════════
fn v_get_dx(p1: BIVec2, p2: BIVec2) -> f64 {
    let dy = (p2.y - p1.y) as f64;

    if dy != 0.0 {
        (p2.x - p1.x) as f64 / dy
    } else if p2.x > p1.x {
        -f64::MAX
    } else {
        f64::MAX
    }
}

fn v_top_x(sc: &VattiScratch, e: usize, y: i64) -> i64 {
    let a = &sc.act_pool[e];

    if y == a.top.y || a.top.x == a.bot.x {
        a.top.x
    } else if y == a.bot.y {
        a.bot.x
    } else {
        a.bot.x + (a.dx * (y - a.bot.y) as f64).round_ties_even() as i64
    }
}

fn v_is_horizontal(sc: &VattiScratch, e: usize) -> bool {
    sc.act_pool[e].top.y == sc.act_pool[e].bot.y
}

fn v_is_hot(sc: &VattiScratch, e: usize) -> bool {
    sc.act_pool[e].outrec.is_some()
}

fn v_is_maxima_v(sc: &VattiScratch, v: usize) -> bool {
    sc.vtx_pool[v].flags & VF_LOCAL_MAX != 0
}

fn v_is_maxima_e(sc: &VattiScratch, e: usize) -> bool {
    sc.act_pool[e]
        .vertex_top
        .is_some_and(|v| v_is_maxima_v(sc, v))
}

fn v_is_front(sc: &VattiScratch, e: usize) -> bool {
    sc.act_pool[e]
        .outrec
        .is_some_and(|o| sc.orc_pool[o].front_edge == Some(e))
}

fn v_is_joined(sc: &VattiScratch, e: usize) -> bool {
    sc.act_pool[e].join_with != 0
}

fn v_polytype(sc: &VattiScratch, e: usize) -> i8 {
    sc.act_pool[e]
        .local_min
        .map_or(0, |lm| sc.locmin_list[lm].polytype)
}

fn v_same_polytype(sc: &VattiScratch, e1: usize, e2: usize) -> bool {
    v_polytype(sc, e1) == v_polytype(sc, e2)
}

fn v_set_dx(sc: &mut VattiScratch, e: usize) {
    sc.act_pool[e].dx = v_get_dx(sc.act_pool[e].bot, sc.act_pool[e].top);
}

fn v_next_vertex(sc: &VattiScratch, e: usize) -> Option<usize> {
    let a = &sc.act_pool[e];
    let vt = a.vertex_top?;

    if a.wind_dx > 0 {
        sc.vtx_pool[vt].next
    } else {
        sc.vtx_pool[vt].prev
    }
}

fn v_prev_prev_vertex(sc: &VattiScratch, e: usize) -> Option<usize> {
    let vt = sc.act_pool[e].vertex_top?;

    if sc.act_pool[e].wind_dx > 0 {
        sc.vtx_pool[sc.vtx_pool[vt].prev?].prev
    } else {
        sc.vtx_pool[sc.vtx_pool[vt].next?].next
    }
}

fn v_cross_product(p1: BIVec2, p2: BIVec2, p3: BIVec2) -> f64 {
    (p2.x - p1.x) as f64 * (p3.y - p2.y) as f64 - (p2.y - p1.y) as f64 * (p3.x - p2.x) as f64
}

fn v_is_collinear(p1: BIVec2, s: BIVec2, p2: BIVec2) -> bool {
    (s.x - p1.x) as i128 * (p2.y - s.y) as i128 == (s.y - p1.y) as i128 * (p2.x - s.x) as i128
}

fn v_perpendic_dist_sq(pt: BIVec2, l1: BIVec2, l2: BIVec2) -> f64 {
    let (a, b, c, d) = (
        (pt.x - l1.x) as f64,
        (pt.y - l1.y) as f64,
        (l2.x - l1.x) as f64,
        (l2.y - l1.y) as f64,
    );

    if c == 0.0 && d == 0.0 {
        return 0.0;
    }

    let e = a * d - c * b;

    (e * e) / (c * c + d * d)
}

fn v_get_seg_isect_pt(a: BIVec2, b: BIVec2, c: BIVec2, d: BIVec2) -> Option<BIVec2> {
    let (dx1, dy1) = ((b.x - a.x) as f64, (b.y - a.y) as f64);
    let (dx2, dy2) = ((d.x - c.x) as f64, (d.y - c.y) as f64);
    let det = dy1 * dx2 - dy2 * dx1;

    if det == 0.0 {
        return None;
    }

    let t = ((a.x - c.x) as f64 * dy2 - (a.y - c.y) as f64 * dx2) / det;

    if t <= 0.0 {
        Some(a)
    } else if t >= 1.0 {
        Some(b)
    } else {
        Some(BIVec2 {
            x: a.x + (t * dx1).round_ties_even() as i64,
            y: a.y + (t * dy1).round_ties_even() as i64,
        })
    }
}

fn v_closest_pt_on_seg(pt: BIVec2, s1: BIVec2, s2: BIVec2) -> BIVec2 {
    if s1 == s2 {
        return s1;
    }

    let (dx, dy) = ((s2.x - s1.x) as f64, (s2.y - s1.y) as f64);
    let q = (((pt.x - s1.x) as f64 * dx + (pt.y - s1.y) as f64 * dy) / (dx * dx + dy * dy))
        .clamp(0.0, 1.0);

    BIVec2 {
        x: s1.x + (q * dx).round_ties_even() as i64,
        y: s1.y + (q * dy).round_ties_even() as i64,
    }
}

fn v_sign_d(v: f64) -> i32 {
    (v > 0.0) as i32 - (v < 0.0) as i32
}

fn v_segs_intersect(a: BIVec2, b: BIVec2, c: BIVec2, d: BIVec2) -> bool {
    (v_sign_d(v_cross_product(a, c, d)) * v_sign_d(v_cross_product(b, c, d)) < 0)
        && (v_sign_d(v_cross_product(c, a, b)) * v_sign_d(v_cross_product(d, a, b)) < 0)
}

fn v_segs_touch(a: BIVec2, b: BIVec2, c: BIVec2, d: BIVec2) -> bool {
    (v_sign_d(v_cross_product(a, c, d)) * v_sign_d(v_cross_product(b, c, d)) <= 0)
        && (v_sign_d(v_cross_product(c, a, b)) * v_sign_d(v_cross_product(d, a, b)) <= 0)
}

fn v_area_outpt(sc: &VattiScratch, start: usize) -> f64 {
    let mut r = 0.0;
    let mut o = start;

    loop {
        let prev = sc.opt_pool[o].prev;
        let pp = sc.opt_pool[prev].pt;
        let cp = sc.opt_pool[o].pt;
        r += (pp.y + cp.y) as f64 * (pp.x - cp.x) as f64;
        o = sc.opt_pool[o].next;

        if o == start {
            break;
        }
    }

    r * 0.5
}

/// Number of points in the output ring through op.
fn v_ring_size(sc: &VattiScratch, op: usize) -> usize {
    let mut count = 0;
    let mut o = op;

    loop {
        count += 1;
        o = sc.opt_pool[o].next;

        if o == op {
            break;
        }
    }

    count
}

fn v_area_tri(p1: BIVec2, p2: BIVec2, p3: BIVec2) -> f64 {
    (p3.y + p1.y) as f64 * (p3.x - p1.x) as f64
        + (p1.y + p2.y) as f64 * (p1.x - p2.x) as f64
        + (p2.y + p3.y) as f64 * (p2.x - p3.x) as f64
}

fn v_pts_close(a: BIVec2, b: BIVec2) -> bool {
    (a.x - b.x).abs() < 2 && (a.y - b.y).abs() < 2
}

fn v_very_small_tri(sc: &VattiScratch, op: usize) -> bool {
    let n = sc.opt_pool[op].next;
    let nn = sc.opt_pool[n].next;
    let p = sc.opt_pool[op].prev;
    nn == p
        && (v_pts_close(sc.opt_pool[p].pt, sc.opt_pool[n].pt)
            || v_pts_close(sc.opt_pool[op].pt, sc.opt_pool[n].pt)
            || v_pts_close(sc.opt_pool[op].pt, sc.opt_pool[p].pt))
}

fn v_valid_closed(sc: &VattiScratch, op: usize) -> bool {
    let n = sc.opt_pool[op].next;

    n != op && n != sc.opt_pool[op].prev && !v_very_small_tri(sc, op)
}

fn v_winding_step(pt: BIVec2, a: BIVec2, b: BIVec2) -> i32 {
    let cross =
        (b.x - a.x) as i128 * (pt.y - a.y) as i128 - (b.y - a.y) as i128 * (pt.x - a.x) as i128;

    if a.y <= pt.y {
        return if b.y > pt.y && cross > 0 { 1 } else { 0 };
    }

    if b.y <= pt.y && cross < 0 {
        -1
    } else {
        0
    }
}

fn pip_i(pt: BIVec2, poly: &[BIVec2]) -> bool {
    let mut winding = 0;
    let n = poly.len();

    for i in 0..n {
        winding += v_winding_step(pt, poly[i], poly[(i + 1) % n]);
    }

    winding != 0
}

fn pip_vertex(sc: &VattiScratch, pt: BIVec2, head: usize) -> bool {
    let mut winding = 0;
    let mut v = head;

    loop {
        let next = sc.vtx_pool[v].next.unwrap();
        winding += v_winding_step(pt, sc.vtx_pool[v].pt, sc.vtx_pool[next].pt);
        v = next;

        if v == head {
            break;
        }
    }

    winding != 0
}

// ═══════════════════════════════════════════════════════════════════════════
// Vertex building and local minima detection
// ═══════════════════════════════════════════════════════════════════════════
fn v_find_local_minima(sc: &mut VattiScratch, head: usize, polytype: i8) {
    let mut pv = sc.vtx_pool[head].prev.unwrap();

    while pv != head && sc.vtx_pool[pv].pt.y == sc.vtx_pool[head].pt.y {
        pv = sc.vtx_pool[pv].prev.unwrap();
    }

    if pv == head {
        return;
    }

    let mut going_up = sc.vtx_pool[pv].pt.y > sc.vtx_pool[head].pt.y;
    let going_up0 = going_up;
    pv = head;
    let mut cv = sc.vtx_pool[head].next.unwrap();

    while cv != head {
        if sc.vtx_pool[cv].pt.y > sc.vtx_pool[pv].pt.y && going_up {
            sc.vtx_pool[pv].flags |= VF_LOCAL_MAX;
            going_up = false;
        } else if sc.vtx_pool[cv].pt.y < sc.vtx_pool[pv].pt.y && !going_up {
            going_up = true;
            sc.vtx_pool[pv].flags |= VF_LOCAL_MIN;
            sc.locmin_list.push(VLocalMinima {
                vertex: pv,
                polytype,
            });
        }

        pv = cv;
        cv = sc.vtx_pool[cv].next.unwrap();
    }

    if going_up != going_up0 {
        if going_up0 {
            sc.vtx_pool[pv].flags |= VF_LOCAL_MIN;
            sc.locmin_list.push(VLocalMinima {
                vertex: pv,
                polytype,
            });
        } else {
            sc.vtx_pool[pv].flags |= VF_LOCAL_MAX;
        }
    }
}

/// Link n scaled points into a circular vertex list and return its head, or None if degenerate.
fn v_link_path(sc: &mut VattiScratch, pts: &[BIVec2], n: usize, polytype: i8) -> Option<usize> {
    let base = sc.vtx_pool.len();
    let vertex = VVertex {
        flags: VF_NONE,
        ..Default::default()
    };
    sc.vtx_pool.resize(base + n, vertex);
    sc.vtx_pool[base].pt = pts[0];
    let mut prev_v = base;
    let mut cnt = 1;

    for pt in &pts[1..n] {
        if *pt == sc.vtx_pool[prev_v].pt {
            continue;
        }

        let cv = base + cnt;
        sc.vtx_pool[cv].pt = *pt;
        sc.vtx_pool[cv].prev = Some(prev_v);
        sc.vtx_pool[prev_v].next = Some(cv);
        prev_v = cv;
        cnt += 1;
    }

    if cnt >= 3 && sc.vtx_pool[prev_v].pt == sc.vtx_pool[base].pt {
        prev_v = sc.vtx_pool[prev_v].prev.unwrap();
        cnt -= 1;
    }

    if cnt < 3 {
        sc.vtx_pool.truncate(base);

        return None;
    }

    sc.vtx_pool.truncate(base + cnt);
    sc.vtx_pool[prev_v].next = Some(base);
    sc.vtx_pool[base].prev = Some(prev_v);
    v_find_local_minima(sc, base, polytype);

    Some(base)
}

fn v_add_path_from_doubles(
    sc: &mut VattiScratch,
    coords: &[f64],
    n: usize,
    polytype: i8,
    bool_scale: f64,
) -> (Option<usize>, i64, i64, i64, i64) {
    if n < 3 {
        return (None, 0, 0, 0, 0);
    }

    let mut pts = Vec::with_capacity(n);

    for i in 0..n {
        pts.push(v_cvt_to_i64(&coords[i * 3..], bool_scale));
    }

    let (min_x, max_x, min_y, max_y) = v_bounds(&pts);

    (
        v_link_path(sc, &pts, n, polytype),
        min_x,
        max_x,
        min_y,
        max_y,
    )
}

fn v_add_path(sc: &mut VattiScratch, pts: &[BIVec2], n: usize, polytype: i8) {
    if n < 3 {
        return;
    }

    v_link_path(sc, pts, n, polytype);
}

// ═══════════════════════════════════════════════════════════════════════════
// AEL operations
// ═══════════════════════════════════════════════════════════════════════════
fn v_get_maxima_pair(sc: &VattiScratch, e: usize) -> Option<usize> {
    let vt = sc.act_pool[e].vertex_top?;
    let mut e2 = sc.act_pool[e].next_in_ael;

    while let Some(ei) = e2 {
        if sc.act_pool[ei].vertex_top == Some(vt) {
            return Some(ei);
        }

        e2 = sc.act_pool[ei].next_in_ael;
    }

    None
}

fn v_get_curr_y_maxima(sc: &VattiScratch, e: usize) -> Option<usize> {
    let mut r = sc.act_pool[e].vertex_top?;

    if sc.act_pool[e].wind_dx > 0 {
        while sc.vtx_pool[sc.vtx_pool[r].next?].pt.y == sc.vtx_pool[r].pt.y {
            r = sc.vtx_pool[r].next?;
        }
    } else {
        while sc.vtx_pool[sc.vtx_pool[r].prev?].pt.y == sc.vtx_pool[r].pt.y {
            r = sc.vtx_pool[r].prev?;
        }
    }

    if v_is_maxima_v(sc, r) {
        Some(r)
    } else {
        None
    }
}

fn v_get_prev_hot(sc: &VattiScratch, e: usize) -> Option<usize> {
    let mut p = sc.act_pool[e].prev_in_ael;

    while let Some(pi) = p {
        if v_is_hot(sc, pi) {
            return Some(pi);
        }

        p = sc.act_pool[pi].prev_in_ael;
    }

    None
}

fn v_is_valid_ael_order(sc: &VattiScratch, resident: usize, newcomer: usize) -> bool {
    if sc.act_pool[newcomer].curr_x != sc.act_pool[resident].curr_x {
        return sc.act_pool[newcomer].curr_x > sc.act_pool[resident].curr_x;
    }

    let d = v_cross_product(
        sc.act_pool[resident].top,
        sc.act_pool[newcomer].bot,
        sc.act_pool[newcomer].top,
    );

    if d != 0.0 {
        return d < 0.0;
    }

    if !v_is_maxima_e(sc, resident) && sc.act_pool[resident].top.y > sc.act_pool[newcomer].top.y {
        return v_cross_product(
            sc.act_pool[newcomer].bot,
            sc.act_pool[resident].top,
            sc.vtx_pool[v_next_vertex(sc, resident).unwrap()].pt,
        ) <= 0.0;
    }

    if !v_is_maxima_e(sc, newcomer) && sc.act_pool[newcomer].top.y > sc.act_pool[resident].top.y {
        return v_cross_product(
            sc.act_pool[newcomer].bot,
            sc.act_pool[newcomer].top,
            sc.vtx_pool[v_next_vertex(sc, newcomer).unwrap()].pt,
        ) >= 0.0;
    }

    let y = sc.act_pool[newcomer].bot.y;

    if sc.act_pool[resident].bot.y != y
        || sc.vtx_pool[sc.locmin_list[sc.act_pool[resident].local_min.unwrap()].vertex]
            .pt
            .y
            != y
    {
        return sc.act_pool[newcomer].is_left_bound;
    }

    if sc.act_pool[resident].is_left_bound != sc.act_pool[newcomer].is_left_bound {
        return sc.act_pool[newcomer].is_left_bound;
    }

    let pp_res = v_prev_prev_vertex(sc, resident);

    if pp_res.is_some()
        && v_is_collinear(
            sc.vtx_pool[pp_res.unwrap()].pt,
            sc.act_pool[resident].bot,
            sc.act_pool[resident].top,
        )
    {
        return true;
    }

    let pp_new = v_prev_prev_vertex(sc, newcomer);

    match (pp_res, pp_new) {
        (Some(pp_res), Some(pp_new)) => {
            (v_cross_product(
                sc.vtx_pool[pp_res].pt,
                sc.act_pool[newcomer].bot,
                sc.vtx_pool[pp_new].pt,
            ) > 0.0)
                == sc.act_pool[newcomer].is_left_bound
        }

        _ => sc.act_pool[newcomer].is_left_bound,
    }
}

fn v_insert_left_edge(sc: &mut VattiScratch, e: usize) {
    if sc.actives.is_none() {
        sc.act_pool[e].prev_in_ael = None;
        sc.act_pool[e].next_in_ael = None;
        sc.actives = Some(e);
    } else if !v_is_valid_ael_order(sc, sc.actives.unwrap(), e) {
        sc.act_pool[e].prev_in_ael = None;
        sc.act_pool[e].next_in_ael = sc.actives;
        sc.act_pool[sc.actives.unwrap()].prev_in_ael = Some(e);
        sc.actives = Some(e);
    } else {
        let mut e2 = sc.actives.unwrap();

        while sc.act_pool[e2].next_in_ael.is_some()
            && v_is_valid_ael_order(sc, sc.act_pool[e2].next_in_ael.unwrap(), e)
        {
            e2 = sc.act_pool[e2].next_in_ael.unwrap();
        }

        if sc.act_pool[e2].join_with == 2 {
            e2 = sc.act_pool[e2].next_in_ael.unwrap();
        }

        let next = sc.act_pool[e2].next_in_ael;
        sc.act_pool[e].next_in_ael = next;

        if let Some(n) = next {
            sc.act_pool[n].prev_in_ael = Some(e);
        }

        sc.act_pool[e].prev_in_ael = Some(e2);
        sc.act_pool[e2].next_in_ael = Some(e);
    }
}

fn v_insert_right_edge(sc: &mut VattiScratch, e: usize, e2: usize) {
    let next = sc.act_pool[e].next_in_ael;
    sc.act_pool[e2].next_in_ael = next;

    if let Some(n) = next {
        sc.act_pool[n].prev_in_ael = Some(e2);
    }

    sc.act_pool[e2].prev_in_ael = Some(e);
    sc.act_pool[e].next_in_ael = Some(e2);
}

fn v_swap_positions_in_ael(sc: &mut VattiScratch, e1: usize, e2: usize) {
    let next = sc.act_pool[e2].next_in_ael;

    if let Some(n) = next {
        sc.act_pool[n].prev_in_ael = Some(e1);
    }

    let prev = sc.act_pool[e1].prev_in_ael;

    if let Some(p) = prev {
        sc.act_pool[p].next_in_ael = Some(e2);
    }

    sc.act_pool[e2].prev_in_ael = prev;
    sc.act_pool[e2].next_in_ael = Some(e1);
    sc.act_pool[e1].prev_in_ael = Some(e2);
    sc.act_pool[e1].next_in_ael = next;

    if sc.act_pool[e2].prev_in_ael.is_none() {
        sc.actives = Some(e2);
    }
}

fn v_delete_from_ael(sc: &mut VattiScratch, e: usize) {
    let prev = sc.act_pool[e].prev_in_ael;
    let next = sc.act_pool[e].next_in_ael;

    if prev.is_none() && next.is_none() && sc.actives != Some(e) {
        return;
    }

    if let Some(p) = prev {
        sc.act_pool[p].next_in_ael = next;
    } else {
        sc.actives = next;
    }

    if let Some(n) = next {
        sc.act_pool[n].prev_in_ael = prev;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Scanline
// ═══════════════════════════════════════════════════════════════════════════
fn v_insert_scanline(sc: &mut VattiScratch, y: i64) {
    sc.scanline_list.push(y);
}

fn v_pop_scanline(sc: &mut VattiScratch) -> Option<i64> {
    let sl = &mut sc.scanline_list;

    if sl.empty() {
        return None;
    }

    let y = sl.top();
    sl.pop();

    while !sl.empty() && y == sl.top() {
        sl.pop();
    }

    Some(y)
}

fn v_pop_locmin(sc: &mut VattiScratch, y: i64) -> Option<usize> {
    if sc.locmin_idx >= sc.locmin_list.len()
        || sc.vtx_pool[sc.locmin_list[sc.locmin_idx].vertex].pt.y != y
    {
        return None;
    }

    sc.locmin_idx += 1;

    Some(sc.locmin_idx - 1)
}

fn v_push_horz(sc: &mut VattiScratch, e: usize) {
    sc.act_pool[e].next_in_sel = sc.sel;
    sc.sel = Some(e);
}

fn v_pop_horz(sc: &mut VattiScratch) -> Option<usize> {
    let e = sc.sel?;
    sc.sel = sc.act_pool[e].next_in_sel;

    Some(e)
}

// ═══════════════════════════════════════════════════════════════════════════
// Winding and contribution
// ═══════════════════════════════════════════════════════════════════════════
fn v_set_wind_count(sc: &mut VattiScratch, e: usize) {
    let pt = v_polytype(sc, e);
    let mut e2 = sc.act_pool[e].prev_in_ael;

    while e2.is_some() && v_polytype(sc, e2.unwrap()) != pt {
        e2 = sc.act_pool[e2.unwrap()].prev_in_ael;
    }

    if e2.is_none() {
        sc.act_pool[e].wind_cnt = sc.act_pool[e].wind_dx;
        e2 = sc.actives;
    } else {
        let ei = e2.unwrap();
        let wc = sc.act_pool[ei].wind_cnt;
        let wd = sc.act_pool[ei].wind_dx;
        let ewd = sc.act_pool[e].wind_dx;

        if wc * wd < 0 {
            sc.act_pool[e].wind_cnt = if wc.abs() > 1 {
                if wd * ewd < 0 {
                    wc
                } else {
                    wc + ewd
                }
            } else {
                ewd
            };
        } else {
            sc.act_pool[e].wind_cnt = if wd * ewd < 0 { wc } else { wc + ewd };
        }

        sc.act_pool[e].wind_cnt2 = sc.act_pool[ei].wind_cnt2;
        e2 = sc.act_pool[ei].next_in_ael;
    }

    while e2.is_some() && e2.unwrap() != e {
        let ei = e2.unwrap();

        if v_polytype(sc, ei) != pt {
            sc.act_pool[e].wind_cnt2 += sc.act_pool[ei].wind_dx;
        }

        e2 = sc.act_pool[ei].next_in_ael;
    }
}

fn v_is_contributing(sc: &VattiScratch, e: usize, cliptype: i32) -> bool {
    if sc.act_pool[e].wind_cnt.abs() != 1 {
        return false;
    }

    let wc2 = sc.act_pool[e].wind_cnt2.abs();

    if cliptype == 0 {
        wc2 != 0
    } else if cliptype == 1 {
        wc2 == 0
    } else {
        let r = wc2 == 0;

        if v_polytype(sc, e) == 0 {
            r
        } else {
            !r
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Output operations
// ═══════════════════════════════════════════════════════════════════════════
fn v_set_sides(sc: &mut VattiScratch, or: usize, f: usize, b: usize) {
    sc.orc_pool[or].front_edge = Some(f);
    sc.orc_pool[or].back_edge = Some(b);
}

fn v_swap_outrecs(sc: &mut VattiScratch, e1: usize, e2: usize) {
    let (or1, or2) = (sc.act_pool[e1].outrec, sc.act_pool[e2].outrec);

    if or1 == or2 {
        if let Some(o) = or1 {
            let fe = sc.orc_pool[o].front_edge;
            sc.orc_pool[o].front_edge = sc.orc_pool[o].back_edge;
            sc.orc_pool[o].back_edge = fe;
        }

        return;
    }

    if let Some(o) = or1 {
        if sc.orc_pool[o].front_edge == Some(e1) {
            sc.orc_pool[o].front_edge = Some(e2);
        } else {
            sc.orc_pool[o].back_edge = Some(e2);
        }
    }

    if let Some(o) = or2 {
        if sc.orc_pool[o].front_edge == Some(e2) {
            sc.orc_pool[o].front_edge = Some(e1);
        } else {
            sc.orc_pool[o].back_edge = Some(e1);
        }
    }

    sc.act_pool[e1].outrec = or2;
    sc.act_pool[e2].outrec = or1;
}

fn v_add_outpt(sc: &mut VattiScratch, e: usize, pt: BIVec2) -> usize {
    let or = sc.act_pool[e].outrec.unwrap();
    let to_front = v_is_front(sc, e);
    let op_front = sc.orc_pool[or].pts.unwrap();
    let op_back = sc.opt_pool[op_front].next;

    if to_front && pt == sc.opt_pool[op_front].pt {
        return op_front;
    }

    if !to_front && pt == sc.opt_pool[op_back].pt {
        return op_back;
    }

    let nop = sc.new_outpt(pt, or);
    sc.opt_pool[op_back].prev = nop;
    sc.opt_pool[nop].prev = op_front;
    sc.opt_pool[nop].next = op_back;
    sc.opt_pool[op_front].next = nop;

    if to_front {
        sc.orc_pool[or].pts = Some(nop);
    }

    nop
}

fn v_add_local_min_poly(
    sc: &mut VattiScratch,
    e1: usize,
    e2: usize,
    pt: BIVec2,
    is_new: bool,
) -> usize {
    let or = sc.new_outrec();
    sc.act_pool[e1].outrec = Some(or);
    sc.act_pool[e2].outrec = Some(or);
    let prev_hot = v_get_prev_hot(sc, e1);

    if let Some(ph) = prev_hot {
        if v_is_front(sc, ph) == is_new {
            v_set_sides(sc, or, e2, e1);
        } else {
            v_set_sides(sc, or, e1, e2);
        }
    } else {
        sc.orc_pool[or].owner = None;

        if is_new {
            v_set_sides(sc, or, e1, e2);
        } else {
            v_set_sides(sc, or, e2, e1);
        }
    }

    let op = sc.new_outpt(pt, or);
    sc.orc_pool[or].pts = Some(op);

    op
}

fn v_uncouple(sc: &mut VattiScratch, e: usize) {
    if let Some(or) = sc.act_pool[e].outrec {
        if let Some(fe) = sc.orc_pool[or].front_edge {
            sc.act_pool[fe].outrec = None;
        }

        if let Some(be) = sc.orc_pool[or].back_edge {
            sc.act_pool[be].outrec = None;
        }

        sc.orc_pool[or].front_edge = None;
        sc.orc_pool[or].back_edge = None;
    }
}

fn v_join_outrec_paths(sc: &mut VattiScratch, e1: usize, e2: usize) {
    let or1 = sc.act_pool[e1].outrec.unwrap();
    let or2 = sc.act_pool[e2].outrec.unwrap();
    let p1_st = sc.orc_pool[or1].pts.unwrap();
    let p2_st = sc.orc_pool[or2].pts.unwrap();
    let p1_end = sc.opt_pool[p1_st].next;
    let p2_end = sc.opt_pool[p2_st].next;

    if v_is_front(sc, e1) {
        sc.opt_pool[p2_end].prev = p1_st;
        sc.opt_pool[p1_st].next = p2_end;
        sc.opt_pool[p2_st].next = p1_end;
        sc.opt_pool[p1_end].prev = p2_st;
        sc.orc_pool[or1].pts = Some(p2_st);
        sc.orc_pool[or1].front_edge = sc.orc_pool[or2].front_edge;

        if let Some(fe) = sc.orc_pool[or1].front_edge {
            sc.act_pool[fe].outrec = Some(or1);
        }
    } else {
        sc.opt_pool[p1_end].prev = p2_st;
        sc.opt_pool[p2_st].next = p1_end;
        sc.opt_pool[p1_st].next = p2_end;
        sc.opt_pool[p2_end].prev = p1_st;
        sc.orc_pool[or1].back_edge = sc.orc_pool[or2].back_edge;

        if let Some(be) = sc.orc_pool[or1].back_edge {
            sc.act_pool[be].outrec = Some(or1);
        }
    }

    sc.orc_pool[or2].front_edge = None;
    sc.orc_pool[or2].back_edge = None;
    sc.orc_pool[or2].pts = None;
    sc.orc_pool[or2].owner = Some(or1);
    sc.act_pool[e1].outrec = None;
    sc.act_pool[e2].outrec = None;
}

fn v_split(sc: &mut VattiScratch, e: usize, pt: BIVec2) {
    if sc.act_pool[e].join_with == 2 {
        let next = sc.act_pool[e].next_in_ael.unwrap();
        sc.act_pool[e].join_with = 0;
        sc.act_pool[next].join_with = 0;
        v_add_local_min_poly(sc, e, next, pt, true);
    } else {
        let prev = sc.act_pool[e].prev_in_ael.unwrap();
        sc.act_pool[e].join_with = 0;
        sc.act_pool[prev].join_with = 0;
        v_add_local_min_poly(sc, prev, e, pt, true);
    }
}

fn v_add_local_max_poly(sc: &mut VattiScratch, e1: usize, e2: usize, pt: BIVec2) -> Option<usize> {
    if v_is_joined(sc, e1) {
        v_split(sc, e1, pt);
    }

    if v_is_joined(sc, e2) {
        v_split(sc, e2, pt);
    }

    if v_is_front(sc, e1) == v_is_front(sc, e2) {
        sc.succeeded = false;

        return None;
    }

    let result = v_add_outpt(sc, e1, pt);

    if sc.act_pool[e1].outrec == sc.act_pool[e2].outrec {
        let or = sc.act_pool[e1].outrec.unwrap();
        sc.orc_pool[or].pts = Some(result);
        v_uncouple(sc, e1);
    } else if sc.act_pool[e1].outrec.map_or(0, |o| sc.orc_pool[o].idx)
        < sc.act_pool[e2].outrec.map_or(0, |o| sc.orc_pool[o].idx)
    {
        v_join_outrec_paths(sc, e1, e2);
    } else {
        v_join_outrec_paths(sc, e2, e1);
    }

    Some(result)
}

// ═══════════════════════════════════════════════════════════════════════════
// Split and check join
// ═══════════════════════════════════════════════════════════════════════════
fn v_check_join_left(sc: &mut VattiScratch, e: usize, pt: BIVec2, check_curr_x: bool) {
    let prev = match sc.act_pool[e].prev_in_ael {
        Some(p) => p,
        None => return,
    };

    if !v_is_hot(sc, e)
        || !v_is_hot(sc, prev)
        || v_is_horizontal(sc, e)
        || v_is_horizontal(sc, prev)
    {
        return;
    }

    if (pt.y < sc.act_pool[e].top.y + 2 || pt.y < sc.act_pool[prev].top.y + 2)
        && (sc.act_pool[e].bot.y > pt.y || sc.act_pool[prev].bot.y > pt.y)
    {
        return;
    }

    if check_curr_x {
        if v_perpendic_dist_sq(pt, sc.act_pool[prev].bot, sc.act_pool[prev].top) > 0.25 {
            return;
        }
    } else if sc.act_pool[e].curr_x != sc.act_pool[prev].curr_x {
        return;
    }

    if !v_is_collinear(sc.act_pool[e].top, pt, sc.act_pool[prev].top) {
        return;
    }

    let or_e = sc.act_pool[e]
        .outrec
        .map(|o| sc.orc_pool[o].idx)
        .unwrap_or(usize::MAX);

    let or_p = sc.act_pool[prev]
        .outrec
        .map(|o| sc.orc_pool[o].idx)
        .unwrap_or(usize::MAX);

    if or_e == or_p {
        v_add_local_max_poly(sc, prev, e, pt);
    } else if or_e < or_p {
        v_join_outrec_paths(sc, e, prev);
    } else {
        v_join_outrec_paths(sc, prev, e);
    }

    sc.act_pool[prev].join_with = 2;
    sc.act_pool[e].join_with = 1;
}

fn v_check_join_right(sc: &mut VattiScratch, e: usize, pt: BIVec2, check_curr_x: bool) {
    let next = match sc.act_pool[e].next_in_ael {
        Some(n) => n,
        None => return,
    };

    if !v_is_hot(sc, e)
        || !v_is_hot(sc, next)
        || v_is_horizontal(sc, e)
        || v_is_horizontal(sc, next)
    {
        return;
    }

    if (pt.y < sc.act_pool[e].top.y + 2 || pt.y < sc.act_pool[next].top.y + 2)
        && (sc.act_pool[e].bot.y > pt.y || sc.act_pool[next].bot.y > pt.y)
    {
        return;
    }

    if check_curr_x {
        if v_perpendic_dist_sq(pt, sc.act_pool[next].bot, sc.act_pool[next].top) > 0.35 {
            return;
        }
    } else if sc.act_pool[e].curr_x != sc.act_pool[next].curr_x {
        return;
    }

    if !v_is_collinear(sc.act_pool[e].top, pt, sc.act_pool[next].top) {
        return;
    }

    let or_e = sc.act_pool[e]
        .outrec
        .map(|o| sc.orc_pool[o].idx)
        .unwrap_or(usize::MAX);

    let or_n = sc.act_pool[next]
        .outrec
        .map(|o| sc.orc_pool[o].idx)
        .unwrap_or(usize::MAX);

    if or_e == or_n {
        v_add_local_max_poly(sc, e, next, pt);
    } else if or_e < or_n {
        v_join_outrec_paths(sc, e, next);
    } else {
        v_join_outrec_paths(sc, next, e);
    }

    sc.act_pool[e].join_with = 2;
    sc.act_pool[next].join_with = 1;
}

// ═══════════════════════════════════════════════════════════════════════════
// Intersect edges
// ═══════════════════════════════════════════════════════════════════════════
/// Update the winding counts of two edges that cross.
fn v_update_wind_counts(sc: &mut VattiScratch, e1: usize, e2: usize) {
    if v_polytype(sc, e1) == v_polytype(sc, e2) {
        if sc.act_pool[e1].wind_cnt + sc.act_pool[e2].wind_dx == 0 {
            sc.act_pool[e1].wind_cnt = -sc.act_pool[e1].wind_cnt;
        } else {
            sc.act_pool[e1].wind_cnt += sc.act_pool[e2].wind_dx;
        }

        if sc.act_pool[e2].wind_cnt - sc.act_pool[e1].wind_dx == 0 {
            sc.act_pool[e2].wind_cnt = -sc.act_pool[e2].wind_cnt;
        } else {
            sc.act_pool[e2].wind_cnt -= sc.act_pool[e1].wind_dx;
        }
    } else {
        sc.act_pool[e1].wind_cnt2 += sc.act_pool[e2].wind_dx;
        sc.act_pool[e2].wind_cnt2 -= sc.act_pool[e1].wind_dx;
    }
}

fn v_intersect_edges(sc: &mut VattiScratch, e1: usize, e2: usize, pt: BIVec2, cliptype: i32) {
    if v_is_joined(sc, e1) {
        v_split(sc, e1, pt);
    }

    if v_is_joined(sc, e2) {
        v_split(sc, e2, pt);
    }

    v_update_wind_counts(sc, e1, e2);

    let old_e1_wc = sc.act_pool[e1].wind_cnt.abs();
    let old_e2_wc = sc.act_pool[e2].wind_cnt.abs();
    let e1_in01 = old_e1_wc == 0 || old_e1_wc == 1;
    let e2_in01 = old_e2_wc == 0 || old_e2_wc == 1;

    if (!v_is_hot(sc, e1) && !e1_in01) || (!v_is_hot(sc, e2) && !e2_in01) {
        return;
    }

    if v_is_hot(sc, e1) && v_is_hot(sc, e2) {
        if (old_e1_wc != 0 && old_e1_wc != 1)
            || (old_e2_wc != 0 && old_e2_wc != 1)
            || v_polytype(sc, e1) != v_polytype(sc, e2)
        {
            v_add_local_max_poly(sc, e1, e2, pt);
        } else if v_is_front(sc, e1) || sc.act_pool[e1].outrec == sc.act_pool[e2].outrec {
            v_add_local_max_poly(sc, e1, e2, pt);
            v_add_local_min_poly(sc, e1, e2, pt, false);
        } else {
            v_add_outpt(sc, e1, pt);
            v_add_outpt(sc, e2, pt);
            v_swap_outrecs(sc, e1, e2);
        }
    } else if v_is_hot(sc, e1) {
        v_add_outpt(sc, e1, pt);
        v_swap_outrecs(sc, e1, e2);
    } else if v_is_hot(sc, e2) {
        v_add_outpt(sc, e2, pt);
        v_swap_outrecs(sc, e1, e2);
    } else {
        let e1_wc2 = sc.act_pool[e1].wind_cnt2.abs() as i64;
        let e2_wc2 = sc.act_pool[e2].wind_cnt2.abs() as i64;

        if !v_same_polytype(sc, e1, e2) {
            v_add_local_min_poly(sc, e1, e2, pt, false);
        } else if old_e1_wc == 1 && old_e2_wc == 1 {
            match cliptype {
                0 => {
                    if e1_wc2 > 0 && e2_wc2 > 0 {
                        v_add_local_min_poly(sc, e1, e2, pt, false);
                    }
                }

                1 => {
                    if e1_wc2 <= 0 && e2_wc2 <= 0 {
                        v_add_local_min_poly(sc, e1, e2, pt, false);
                    }
                }

                _ => {
                    if (v_polytype(sc, e1) == 1 && e1_wc2 > 0 && e2_wc2 > 0)
                        || (v_polytype(sc, e1) == 0 && e1_wc2 <= 0 && e2_wc2 <= 0)
                    {
                        v_add_local_min_poly(sc, e1, e2, pt, false);
                    }
                }
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Horizontal edges
// ═══════════════════════════════════════════════════════════════════════════
fn v_add_trial_horz_join(sc: &mut VattiScratch, op: usize) {
    sc.horz_seg_list.push(VHorzSeg {
        left_op: op,
        right_op: None,
        left_to_right: true,
    });
}

fn v_get_last_op(sc: &VattiScratch, e: usize) -> usize {
    let or = sc.act_pool[e].outrec.unwrap();
    let pts = sc.orc_pool[or].pts.unwrap();

    if sc.orc_pool[or].front_edge == Some(e) {
        pts
    } else {
        sc.opt_pool[pts].next
    }
}

fn v_update_edge_into_ael(sc: &mut VattiScratch, e: usize) {
    let nv = v_next_vertex(sc, e).unwrap();
    sc.act_pool[e].bot = sc.act_pool[e].top;
    sc.act_pool[e].vertex_top = Some(nv);
    sc.act_pool[e].top = sc.vtx_pool[nv].pt;
    sc.act_pool[e].curr_x = sc.act_pool[e].bot.x;
    v_set_dx(sc, e);

    if v_is_joined(sc, e) {
        v_split(sc, e, sc.act_pool[e].bot);
    }

    if v_is_horizontal(sc, e) {
        let mut pt = sc.vtx_pool[v_next_vertex(sc, e).unwrap()].pt;

        while pt.y == sc.act_pool[e].top.y {
            if (pt.x < sc.act_pool[e].top.x) != (sc.act_pool[e].bot.x < sc.act_pool[e].top.x) {
                break;
            }

            sc.act_pool[e].vertex_top = v_next_vertex(sc, e);
            sc.act_pool[e].top = pt;

            if v_is_maxima_e(sc, e) {
                break;
            }

            pt = sc.vtx_pool[v_next_vertex(sc, e).unwrap()].pt;
        }

        v_set_dx(sc, e);

        return;
    }

    v_insert_scanline(sc, sc.act_pool[e].top.y);
    v_check_join_left(sc, e, sc.act_pool[e].bot, false);
    v_check_join_right(sc, e, sc.act_pool[e].bot, true);
}

fn v_reset_horz_dir(sc: &VattiScratch, e: usize, max_v: Option<usize>) -> (bool, i64, i64) {
    if sc.act_pool[e].bot.x == sc.act_pool[e].top.x {
        let mut ae = sc.act_pool[e].next_in_ael;

        while ae.is_some() && sc.act_pool[ae.unwrap()].vertex_top != max_v {
            ae = sc.act_pool[ae.unwrap()].next_in_ael;
        }

        (ae.is_some(), sc.act_pool[e].curr_x, sc.act_pool[e].curr_x)
    } else if sc.act_pool[e].curr_x < sc.act_pool[e].top.x {
        (true, sc.act_pool[e].curr_x, sc.act_pool[e].top.x)
    } else {
        (false, sc.act_pool[e].top.x, sc.act_pool[e].curr_x)
    }
}

/// Close a horizontal that reached the edge sharing its maximum vertex.
fn v_horz_meet_maxima(
    sc: &mut VattiScratch,
    horz: usize,
    e: usize,
    vertex_max: Option<usize>,
    is_ltr: bool,
) {
    if v_is_hot(sc, horz) && v_is_joined(sc, e) {
        v_split(sc, e, sc.act_pool[e].top);
    }

    if v_is_hot(sc, horz) {
        while sc.act_pool[horz].vertex_top != vertex_max {
            v_add_outpt(sc, horz, sc.act_pool[horz].top);
            v_update_edge_into_ael(sc, horz);
        }

        if is_ltr {
            v_add_local_max_poly(sc, horz, e, sc.act_pool[horz].top);
        } else {
            v_add_local_max_poly(sc, e, horz, sc.act_pool[horz].top);
        }
    }

    v_delete_from_ael(sc, e);
    v_delete_from_ael(sc, horz);
}

/// Return whether a horizontal stops before edge e.
fn v_horz_stops_at(
    sc: &VattiScratch,
    horz: usize,
    e: usize,
    vertex_max: Option<usize>,
    is_ltr: bool,
    horz_left: i64,
    horz_right: i64,
) -> bool {
    if vertex_max == sc.act_pool[horz].vertex_top {
        return false;
    }

    if (is_ltr && sc.act_pool[e].curr_x > horz_right)
        || (!is_ltr && sc.act_pool[e].curr_x < horz_left)
    {
        return true;
    }

    if sc.act_pool[e].curr_x != sc.act_pool[horz].top.x || v_is_horizontal(sc, e) {
        return false;
    }

    let pt2 = sc.vtx_pool[v_next_vertex(sc, horz).unwrap()].pt;

    if is_ltr {
        return v_top_x(sc, e, pt2.y) >= pt2.x;
    }

    v_top_x(sc, e, pt2.y) <= pt2.x
}

/// Pass a horizontal over edge e and return the next edge in its direction.
fn v_horz_cross_edge(
    sc: &mut VattiScratch,
    horz: usize,
    e: usize,
    is_ltr: bool,
    cliptype: i32,
) -> Option<usize> {
    let pt = BIVec2 {
        x: sc.act_pool[e].curr_x,
        y: sc.act_pool[horz].bot.y,
    };

    if is_ltr {
        v_intersect_edges(sc, horz, e, pt, cliptype);
        v_swap_positions_in_ael(sc, horz, e);
        v_check_join_left(sc, e, pt, false);
    } else {
        v_intersect_edges(sc, e, horz, pt, cliptype);
        v_swap_positions_in_ael(sc, e, horz);
        v_check_join_right(sc, e, pt, false);
    }

    sc.act_pool[horz].curr_x = sc.act_pool[e].curr_x;

    if sc.act_pool[horz].outrec.is_some() {
        let last_op = v_get_last_op(sc, horz);
        v_add_trial_horz_join(sc, last_op);
    }

    if is_ltr {
        sc.act_pool[horz].next_in_ael
    } else {
        sc.act_pool[horz].prev_in_ael
    }
}

fn v_do_horizontal(sc: &mut VattiScratch, horz: usize, cliptype: i32) {
    let y = sc.act_pool[horz].bot.y;
    let vertex_max = v_get_curr_y_maxima(sc, horz);
    let (mut is_ltr, mut horz_left, mut horz_right) = v_reset_horz_dir(sc, horz, vertex_max);

    if v_is_hot(sc, horz) {
        let op = v_add_outpt(
            sc,
            horz,
            BIVec2 {
                x: sc.act_pool[horz].curr_x,
                y,
            },
        );
        v_add_trial_horz_join(sc, op);
    }

    let max_iter = sc.vtx_pool.len();

    for _ in 0..max_iter {
        let mut ei = if is_ltr {
            sc.act_pool[horz].next_in_ael
        } else {
            sc.act_pool[horz].prev_in_ael
        };

        while let Some(e) = ei {
            if sc.act_pool[e].vertex_top == vertex_max {
                v_horz_meet_maxima(sc, horz, e, vertex_max, is_ltr);

                return;
            }

            if v_horz_stops_at(sc, horz, e, vertex_max, is_ltr, horz_left, horz_right) {
                break;
            }

            ei = v_horz_cross_edge(sc, horz, e, is_ltr, cliptype);
        }

        let nv = v_next_vertex(sc, horz);

        if nv.is_none() || sc.vtx_pool[nv.unwrap()].pt.y != sc.act_pool[horz].top.y {
            break;
        }

        if v_is_hot(sc, horz) {
            v_add_outpt(sc, horz, sc.act_pool[horz].top);
        }

        v_update_edge_into_ael(sc, horz);
        (is_ltr, horz_left, horz_right) = v_reset_horz_dir(sc, horz, vertex_max);
    }

    if v_is_hot(sc, horz) {
        let op = v_add_outpt(sc, horz, sc.act_pool[horz].top);
        v_add_trial_horz_join(sc, op);
    }

    v_update_edge_into_ael(sc, horz);
}

// ═══════════════════════════════════════════════════════════════════════════
// Horizontal joins
// ═══════════════════════════════════════════════════════════════════════════
fn v_dup_outpt(sc: &mut VattiScratch, op: usize, after: bool) -> usize {
    let r = sc.new_outpt(sc.opt_pool[op].pt, sc.opt_pool[op].outrec);

    if after {
        let next = sc.opt_pool[op].next;
        sc.opt_pool[r].next = next;
        sc.opt_pool[next].prev = r;
        sc.opt_pool[r].prev = op;
        sc.opt_pool[op].next = r;
    } else {
        let prev = sc.opt_pool[op].prev;
        sc.opt_pool[r].prev = prev;
        sc.opt_pool[prev].next = r;
        sc.opt_pool[r].next = op;
        sc.opt_pool[op].prev = r;
    }

    r
}

fn v_horz_seg_less(sc: &VattiScratch, a: &VHorzSeg, b: &VHorzSeg) -> std::cmp::Ordering {
    if a.right_op.is_none() || b.right_op.is_none() {
        return a.right_op.is_none().cmp(&b.right_op.is_none());
    }

    sc.opt_pool[a.left_op]
        .pt
        .x
        .cmp(&sc.opt_pool[b.left_op].pt.x)
}

/// Order a horizontal run into its segment and return whether it can join.
fn v_set_horz_segment(sc: &mut VattiScratch, i: usize, op_p: usize, op_n: usize) -> bool {
    let hs = &mut sc.horz_seg_list[i];

    if sc.opt_pool[op_p].pt.x == sc.opt_pool[op_n].pt.x {
        hs.right_op = None;

        return false;
    }

    if sc.opt_pool[op_p].pt.x < sc.opt_pool[op_n].pt.x {
        hs.left_op = op_p;
        hs.right_op = Some(op_n);
        hs.left_to_right = true;
    } else {
        hs.left_op = op_n;
        hs.right_op = Some(op_p);
        hs.left_to_right = false;
    }

    if sc.opt_pool[hs.left_op].horz {
        hs.right_op = None;

        return false;
    }

    sc.opt_pool[hs.left_op].horz = true;

    true
}

/// Extend a trial segment to its full horizontal run and return whether it can join.
fn v_update_horz_segment(sc: &mut VattiScratch, i: usize) -> bool {
    let op = sc.horz_seg_list[i].left_op;
    let mut outrec = Some(sc.opt_pool[op].outrec);

    while outrec.is_some_and(|o| sc.orc_pool[o].pts.is_none()) {
        outrec = sc.orc_pool[outrec.unwrap()].owner;
    }

    let Some(or) = outrec else {
        sc.horz_seg_list[i].right_op = None;

        return false;
    };

    let has_edges = sc.orc_pool[or].front_edge.is_some();
    let cy = sc.opt_pool[op].pt.y;
    let mut op_p = op;
    let mut op_n = op;

    if has_edges {
        let op_a = sc.orc_pool[or].pts.unwrap();
        let op_z = sc.opt_pool[op_a].next;

        while op_p != op_z && sc.opt_pool[sc.opt_pool[op_p].prev].pt.y == cy {
            op_p = sc.opt_pool[op_p].prev;
        }

        while op_n != op_a && sc.opt_pool[sc.opt_pool[op_n].next].pt.y == cy {
            op_n = sc.opt_pool[op_n].next;
        }
    } else {
        while sc.opt_pool[op_p].prev != op_n && sc.opt_pool[sc.opt_pool[op_p].prev].pt.y == cy {
            op_p = sc.opt_pool[op_p].prev;
        }

        while sc.opt_pool[op_n].next != op_p && sc.opt_pool[sc.opt_pool[op_n].next].pt.y == cy {
            op_n = sc.opt_pool[op_n].next;
        }
    }

    v_set_horz_segment(sc, i, op_p, op_n)
}

/// Join two overlapping horizontal segments of opposite direction.
fn v_add_horz_join(sc: &mut VattiScratch, i: usize, k: usize) {
    let cy = sc.opt_pool[sc.horz_seg_list[i].left_op].pt.y;
    let mut lo1 = sc.horz_seg_list[i].left_op;
    let mut lo2 = sc.horz_seg_list[k].left_op;

    if sc.horz_seg_list[i].left_to_right {
        while sc.opt_pool[sc.opt_pool[lo1].next].pt.y == cy
            && sc.opt_pool[sc.opt_pool[lo1].next].pt.x <= sc.opt_pool[lo2].pt.x
        {
            lo1 = sc.opt_pool[lo1].next;
        }

        while sc.opt_pool[sc.opt_pool[lo2].prev].pt.y == cy
            && sc.opt_pool[sc.opt_pool[lo2].prev].pt.x <= sc.opt_pool[lo1].pt.x
        {
            lo2 = sc.opt_pool[lo2].prev;
        }

        let op1 = v_dup_outpt(sc, lo1, true);
        let op2 = v_dup_outpt(sc, lo2, false);
        sc.horz_join_list.push(VHorzJoin { op1, op2 });
    } else {
        while sc.opt_pool[sc.opt_pool[lo1].prev].pt.y == cy
            && sc.opt_pool[sc.opt_pool[lo1].prev].pt.x <= sc.opt_pool[lo2].pt.x
        {
            lo1 = sc.opt_pool[lo1].prev;
        }

        while sc.opt_pool[sc.opt_pool[lo2].next].pt.y == cy
            && sc.opt_pool[sc.opt_pool[lo2].next].pt.x <= sc.opt_pool[lo1].pt.x
        {
            lo2 = sc.opt_pool[lo2].next;
        }

        let op1 = v_dup_outpt(sc, lo2, true);
        let op2 = v_dup_outpt(sc, lo1, false);
        sc.horz_join_list.push(VHorzJoin { op1, op2 });
    }

    sc.horz_seg_list[i].left_op = lo1;
    sc.horz_seg_list[k].left_op = lo2;
}

fn v_convert_horz_segs_to_joins(sc: &mut VattiScratch) {
    let mut valid = 0;

    for i in 0..sc.horz_seg_list.len() {
        if v_update_horz_segment(sc, i) {
            valid += 1;
        }
    }

    if valid < 2 {
        return;
    }

    let mut list = std::mem::take(&mut sc.horz_seg_list);
    list.sort_by(|a, b| v_horz_seg_less(sc, a, b));
    sc.horz_seg_list = list;

    for i in 0..valid - 1 {
        for k in (i + 1)..valid {
            let hs1 = &sc.horz_seg_list[i];
            let hs2 = &sc.horz_seg_list[k];

            if sc.opt_pool[hs2.left_op].pt.x >= sc.opt_pool[hs1.right_op.unwrap()].pt.x
                || hs2.left_to_right == hs1.left_to_right
                || sc.opt_pool[hs2.right_op.unwrap()].pt.x <= sc.opt_pool[hs1.left_op].pt.x
            {
                continue;
            }

            v_add_horz_join(sc, i, k);
        }
    }
}

fn v_fix_outrec_pts(sc: &mut VattiScratch, or: usize) {
    let start = sc.orc_pool[or].pts.unwrap();
    let mut o = start;

    loop {
        sc.opt_pool[o].outrec = or;
        o = sc.opt_pool[o].next;

        if o == start {
            break;
        }
    }
}

fn v_process_horz_joins(sc: &mut VattiScratch) {
    for ji in 0..sc.horz_join_list.len() {
        let op1 = sc.horz_join_list[ji].op1;
        let op2 = sc.horz_join_list[ji].op2;
        let mut or1 = sc.opt_pool[op1].outrec;

        while sc.orc_pool[or1].pts.is_none() {
            or1 = sc.orc_pool[or1].owner.unwrap_or(or1);
        }

        let mut or2 = sc.opt_pool[op2].outrec;

        while sc.orc_pool[or2].pts.is_none() {
            or2 = sc.orc_pool[or2].owner.unwrap_or(or2);
        }

        let op1b = sc.opt_pool[op1].next;
        let op2b = sc.opt_pool[op2].prev;
        sc.opt_pool[op1].next = op2;
        sc.opt_pool[op2].prev = op1;
        sc.opt_pool[op1b].prev = op2b;
        sc.opt_pool[op2b].next = op1b;

        if or1 == or2 {
            let nr = sc.new_outrec();
            sc.orc_pool[nr].pts = Some(op1b);
            v_fix_outrec_pts(sc, nr);

            if sc.opt_pool[sc.orc_pool[or1].pts.unwrap()].outrec == nr {
                sc.orc_pool[or1].pts = Some(op1);
                sc.opt_pool[op1].outrec = or1;
            }

            sc.orc_pool[nr].owner = Some(or1);
        } else {
            sc.orc_pool[or2].pts = None;
            sc.orc_pool[or2].owner = Some(or1);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Intersection detection
// ═══════════════════════════════════════════════════════════════════════════
fn v_adjust_curr_x_copy_to_sel(sc: &mut VattiScratch, top_y: i64) {
    let mut e_opt = sc.actives;
    sc.sel = e_opt;

    while let Some(e) = e_opt {
        sc.act_pool[e].prev_in_sel = sc.act_pool[e].prev_in_ael;
        sc.act_pool[e].next_in_sel = sc.act_pool[e].next_in_ael;
        sc.act_pool[e].jump = sc.act_pool[e].next_in_sel;

        if sc.act_pool[e].join_with == 1 {
            sc.act_pool[e].curr_x = sc.act_pool[sc.act_pool[e].prev_in_ael.unwrap()].curr_x;
        } else {
            sc.act_pool[e].curr_x = v_top_x(sc, e, top_y);
        }

        e_opt = sc.act_pool[e].next_in_ael;
    }
}

fn v_extract_from_sel(sc: &mut VattiScratch, ae: usize) -> Option<usize> {
    let res = sc.act_pool[ae].next_in_sel;

    if let Some(r) = res {
        sc.act_pool[r].prev_in_sel = sc.act_pool[ae].prev_in_sel;
    }

    let prev = sc.act_pool[ae].prev_in_sel.unwrap();
    sc.act_pool[prev].next_in_sel = res;

    res
}

fn v_insert1_before2_in_sel(sc: &mut VattiScratch, a1: usize, a2: usize) {
    sc.act_pool[a1].prev_in_sel = sc.act_pool[a2].prev_in_sel;

    if let Some(p) = sc.act_pool[a1].prev_in_sel {
        sc.act_pool[p].next_in_sel = Some(a1);
    }

    sc.act_pool[a1].next_in_sel = Some(a2);
    sc.act_pool[a2].prev_in_sel = Some(a1);
}

fn v_add_new_isect_node(sc: &mut VattiScratch, e1: usize, e2: usize, top_y: i64) {
    let ip = match v_get_seg_isect_pt(
        sc.act_pool[e1].bot,
        sc.act_pool[e1].top,
        sc.act_pool[e2].bot,
        sc.act_pool[e2].top,
    ) {
        Some(p) => p,
        None => BIVec2 {
            x: sc.act_pool[e1].curr_x,
            y: top_y,
        },
    };
    let mut ip = ip;

    if ip.y > sc.bot_y || ip.y < top_y {
        let (ad1, ad2) = (sc.act_pool[e1].dx.abs(), sc.act_pool[e2].dx.abs());

        if ad1 > 100.0 && ad2 > 100.0 {
            ip = if ad1 > ad2 {
                v_closest_pt_on_seg(ip, sc.act_pool[e1].bot, sc.act_pool[e1].top)
            } else {
                v_closest_pt_on_seg(ip, sc.act_pool[e2].bot, sc.act_pool[e2].top)
            };
        } else if ad1 > 100.0 {
            ip = v_closest_pt_on_seg(ip, sc.act_pool[e1].bot, sc.act_pool[e1].top);
        } else if ad2 > 100.0 {
            ip = v_closest_pt_on_seg(ip, sc.act_pool[e2].bot, sc.act_pool[e2].top);
        } else {
            if ip.y < top_y {
                ip.y = top_y;
            } else {
                ip.y = sc.bot_y;
            }

            ip.x = if ad1 < ad2 {
                v_top_x(sc, e1, ip.y)
            } else {
                v_top_x(sc, e2, ip.y)
            };
        }
    }

    sc.intersect_nodes.push(VIntersectNode {
        pt: ip,
        edge1: e1,
        edge2: e2,
    });
}

fn v_build_intersect_list(sc: &mut VattiScratch, top_y: i64) -> bool {
    let Some(first) = sc.actives else {
        return false;
    };

    if sc.act_pool[first].next_in_ael.is_none() {
        return false;
    }

    v_adjust_curr_x_copy_to_sel(sc, top_y);
    let mut left = sc.sel;

    while left.is_some_and(|l| sc.act_pool[l].jump.is_some()) {
        let mut prev_base: Option<usize> = None;

        while left.is_some_and(|l| sc.act_pool[l].jump.is_some()) {
            let mut curr_base = left.unwrap();
            let mut right = sc.act_pool[curr_base].jump;
            let mut l_end = right;
            let r_end = sc.act_pool[right.unwrap()].jump;
            sc.act_pool[curr_base].jump = r_end;

            while left != l_end && right != r_end {
                let li = left.unwrap();
                let ri = right.unwrap();

                if sc.act_pool[ri].curr_x < sc.act_pool[li].curr_x {
                    let mut tmp = sc.act_pool[ri].prev_in_sel.unwrap();
                    let max_iter = sc.vtx_pool.len();

                    for _ in 0..max_iter {
                        v_add_new_isect_node(sc, tmp, ri, top_y);

                        if tmp == li {
                            break;
                        }

                        tmp = sc.act_pool[tmp].prev_in_sel.unwrap();
                    }

                    tmp = ri;
                    right = v_extract_from_sel(sc, tmp);
                    l_end = right;
                    v_insert1_before2_in_sel(sc, tmp, li);

                    if li == curr_base {
                        curr_base = tmp;
                        sc.act_pool[curr_base].jump = r_end;

                        match prev_base {
                            None => sc.sel = Some(curr_base),
                            Some(pb) => sc.act_pool[pb].jump = Some(curr_base),
                        }
                    }
                } else {
                    left = sc.act_pool[li].next_in_sel;
                }
            }

            prev_base = Some(curr_base);
            left = r_end;
        }

        left = sc.sel;
    }

    !sc.intersect_nodes.is_empty()
}

fn v_intersect_node_less(a: &VIntersectNode, b: &VIntersectNode) -> std::cmp::Ordering {
    if a.pt.y == b.pt.y {
        a.pt.x.cmp(&b.pt.x)
    } else {
        b.pt.y.cmp(&a.pt.y)
    }
}

fn v_process_intersect_list(sc: &mut VattiScratch, cliptype: i32) {
    sc.intersect_nodes.sort_by(v_intersect_node_less);
    let n = sc.intersect_nodes.len();

    for i in 0..n {
        let e1 = sc.intersect_nodes[i].edge1;
        let e2 = sc.intersect_nodes[i].edge2;
        let adj =
            sc.act_pool[e1].next_in_ael == Some(e2) || sc.act_pool[e1].prev_in_ael == Some(e2);

        if !adj {
            for j in (i + 1)..n {
                let je1 = sc.intersect_nodes[j].edge1;
                let je2 = sc.intersect_nodes[j].edge2;

                if sc.act_pool[je1].next_in_ael == Some(je2)
                    || sc.act_pool[je1].prev_in_ael == Some(je2)
                {
                    sc.intersect_nodes.swap(i, j);
                    break;
                }
            }
        }

        let pt = sc.intersect_nodes[i].pt;
        let e1 = sc.intersect_nodes[i].edge1;
        let e2 = sc.intersect_nodes[i].edge2;
        v_intersect_edges(sc, e1, e2, pt, cliptype);
        v_swap_positions_in_ael(sc, e1, e2);
        sc.act_pool[e1].curr_x = pt.x;
        sc.act_pool[e2].curr_x = pt.x;
        v_check_join_left(sc, e2, pt, true);
        v_check_join_right(sc, e1, pt, true);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Local minima insertion
// ═══════════════════════════════════════════════════════════════════════════
/// New active edge leaving a local minimum, wind_dx -1 along prev and 1 along next.
fn v_new_bound(sc: &mut VattiScratch, lm: usize, wind_dx: i32) -> usize {
    let vertex = sc.locmin_list[lm].vertex;
    let b = sc.new_active();
    sc.act_pool[b].bot = sc.vtx_pool[vertex].pt;
    sc.act_pool[b].curr_x = sc.act_pool[b].bot.x;
    sc.act_pool[b].wind_dx = wind_dx;
    sc.act_pool[b].vertex_top = if wind_dx < 0 {
        sc.vtx_pool[vertex].prev
    } else {
        sc.vtx_pool[vertex].next
    };
    sc.act_pool[b].top = sc.vtx_pool[sc.act_pool[b].vertex_top.unwrap()].pt;
    sc.act_pool[b].local_min = Some(lm);
    v_set_dx(sc, b);

    b
}

fn v_insert_local_minima_into_ael(sc: &mut VattiScratch, bot_y: i64, cliptype: i32) {
    while let Some(lm) = v_pop_locmin(sc, bot_y) {
        let mut lb = v_new_bound(sc, lm, -1);
        let mut rb = v_new_bound(sc, lm, 1);

        if v_is_horizontal(sc, lb) {
            if sc.act_pool[lb].dx == -f64::MAX {
                std::mem::swap(&mut lb, &mut rb);
            }
        } else if v_is_horizontal(sc, rb) {
            if sc.act_pool[rb].dx == f64::MAX {
                std::mem::swap(&mut lb, &mut rb);
            }
        } else if sc.act_pool[lb].dx < sc.act_pool[rb].dx {
            std::mem::swap(&mut lb, &mut rb);
        }

        sc.act_pool[lb].is_left_bound = true;
        v_insert_left_edge(sc, lb);
        v_set_wind_count(sc, lb);
        let contributing = v_is_contributing(sc, lb, cliptype);

        sc.act_pool[rb].is_left_bound = false;
        sc.act_pool[rb].wind_cnt = sc.act_pool[lb].wind_cnt;
        sc.act_pool[rb].wind_cnt2 = sc.act_pool[lb].wind_cnt2;
        v_insert_right_edge(sc, lb, rb);

        if contributing {
            v_add_local_min_poly(sc, lb, rb, sc.act_pool[lb].bot, true);

            if !v_is_horizontal(sc, lb) {
                v_check_join_left(sc, lb, sc.act_pool[lb].bot, false);
            }
        }

        while sc.act_pool[rb].next_in_ael.is_some()
            && v_is_valid_ael_order(sc, sc.act_pool[rb].next_in_ael.unwrap(), rb)
        {
            let next = sc.act_pool[rb].next_in_ael.unwrap();
            v_intersect_edges(sc, rb, next, sc.act_pool[rb].bot, cliptype);
            v_swap_positions_in_ael(sc, rb, next);
        }

        if v_is_horizontal(sc, rb) {
            v_push_horz(sc, rb);
        } else {
            v_check_join_right(sc, rb, sc.act_pool[rb].bot, false);
            v_insert_scanline(sc, sc.act_pool[rb].top.y);
        }

        if v_is_horizontal(sc, lb) {
            v_push_horz(sc, lb);
        } else {
            v_insert_scanline(sc, sc.act_pool[lb].top.y);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Maxima
// ═══════════════════════════════════════════════════════════════════════════
fn v_do_maxima(sc: &mut VattiScratch, e: usize, cliptype: i32) -> Option<usize> {
    let prev_e = sc.act_pool[e].prev_in_ael;
    let max_pair = match v_get_maxima_pair(sc, e) {
        Some(mp) => mp,
        None => return sc.act_pool[e].next_in_ael,
    };

    if v_is_joined(sc, e) {
        v_split(sc, e, sc.act_pool[e].top);
    }

    if v_is_joined(sc, max_pair) {
        v_split(sc, max_pair, sc.act_pool[max_pair].top);
    }

    let mut next_e = sc.act_pool[e].next_in_ael;

    while next_e != Some(max_pair) {
        let ne = next_e.unwrap();
        v_intersect_edges(sc, e, ne, sc.act_pool[e].top, cliptype);
        v_swap_positions_in_ael(sc, e, ne);
        next_e = sc.act_pool[e].next_in_ael;
    }

    if v_is_hot(sc, e) {
        v_add_local_max_poly(sc, e, max_pair, sc.act_pool[e].top);
    }

    v_delete_from_ael(sc, max_pair);
    v_delete_from_ael(sc, e);

    if let Some(p) = prev_e {
        sc.act_pool[p].next_in_ael
    } else {
        sc.actives
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Top of scanbeam
// ═══════════════════════════════════════════════════════════════════════════
fn v_do_top_of_scanbeam(sc: &mut VattiScratch, y: i64, cliptype: i32) {
    sc.sel = None;
    let mut e_opt = sc.actives;

    while let Some(e) = e_opt {
        if sc.act_pool[e].top.y == y {
            sc.act_pool[e].curr_x = sc.act_pool[e].top.x;

            if v_is_maxima_e(sc, e) {
                e_opt = v_do_maxima(sc, e, cliptype);
                continue;
            }

            if v_is_hot(sc, e) {
                v_add_outpt(sc, e, sc.act_pool[e].top);
            }

            v_update_edge_into_ael(sc, e);

            if v_is_horizontal(sc, e) {
                v_push_horz(sc, e);
            }
        } else {
            sc.act_pool[e].curr_x = v_top_x(sc, e, y);
        }

        e_opt = sc.act_pool[e].next_in_ael;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Collinear cleanup and self intersections
// ═══════════════════════════════════════════════════════════════════════════
fn v_dispose_outpt(sc: &mut VattiScratch, op: usize) -> usize {
    let r = sc.opt_pool[op].next;
    let prev = sc.opt_pool[op].prev;
    sc.opt_pool[prev].next = r;
    sc.opt_pool[r].prev = prev;

    r
}

fn v_do_split_op(sc: &mut VattiScratch, or: usize, split_op: usize) {
    let prev_op = sc.opt_pool[split_op].prev;
    let nn_op = sc.opt_pool[sc.opt_pool[split_op].next].next;
    sc.orc_pool[or].pts = Some(prev_op);
    let ip = match v_get_seg_isect_pt(
        sc.opt_pool[prev_op].pt,
        sc.opt_pool[split_op].pt,
        sc.opt_pool[sc.opt_pool[split_op].next].pt,
        sc.opt_pool[nn_op].pt,
    ) {
        Some(p) => p,
        None => return,
    };
    let area1 = v_area_outpt(sc, sc.orc_pool[or].pts.unwrap());

    if area1.abs() < 2.0 {
        sc.orc_pool[or].pts = None;

        return;
    }

    let area2 = v_area_tri(
        ip,
        sc.opt_pool[split_op].pt,
        sc.opt_pool[sc.opt_pool[split_op].next].pt,
    );

    if ip == sc.opt_pool[prev_op].pt || ip == sc.opt_pool[nn_op].pt {
        sc.opt_pool[nn_op].prev = prev_op;
        sc.opt_pool[prev_op].next = nn_op;
    } else {
        let nop = sc.new_outpt(ip, sc.opt_pool[prev_op].outrec);
        sc.opt_pool[nop].prev = prev_op;
        sc.opt_pool[nop].next = nn_op;
        sc.opt_pool[nn_op].prev = nop;
        sc.opt_pool[prev_op].next = nop;
    }

    if area2.abs() >= 1.0 && (area2.abs() > area1.abs() || (area2 > 0.0) == (area1 > 0.0)) {
        let or_owner = sc.orc_pool[or].owner;
        let nr = sc.new_outrec();
        sc.orc_pool[nr].owner = or_owner;
        let split_next = sc.opt_pool[split_op].next;
        sc.opt_pool[split_op].outrec = nr;
        sc.opt_pool[split_next].outrec = nr;
        let nop = sc.new_outpt(ip, nr);
        sc.opt_pool[nop].prev = split_next;
        sc.opt_pool[nop].next = split_op;
        sc.orc_pool[nr].pts = Some(nop);
        sc.opt_pool[split_op].prev = nop;
        sc.opt_pool[split_next].next = nop;
    }
}

fn v_fix_self_intersects(sc: &mut VattiScratch, or: usize) {
    let mut op2 = match sc.orc_pool[or].pts {
        Some(p) => p,
        None => return,
    };
    let count = v_ring_size(sc, op2);
    let max_iter = (count + 1) * (count + 1);

    for _ in 0..max_iter {
        if sc.opt_pool[op2].prev == sc.opt_pool[sc.opt_pool[op2].next].next {
            break;
        }

        if v_segs_intersect(
            sc.opt_pool[sc.opt_pool[op2].prev].pt,
            sc.opt_pool[op2].pt,
            sc.opt_pool[sc.opt_pool[op2].next].pt,
            sc.opt_pool[sc.opt_pool[sc.opt_pool[op2].next].next].pt,
        ) {
            let pts = sc.orc_pool[or].pts.unwrap();

            if op2 == pts || sc.opt_pool[op2].next == pts {
                sc.orc_pool[or].pts = Some(sc.opt_pool[pts].prev);
            }

            v_do_split_op(sc, or, op2);

            if sc.orc_pool[or].pts.is_none() {
                break;
            }

            op2 = sc.orc_pool[or].pts.unwrap();
            continue;
        }

        op2 = sc.opt_pool[op2].next;

        if op2 == sc.orc_pool[or].pts.unwrap_or(usize::MAX) {
            break;
        }
    }
}

fn v_clean_collinear(sc: &mut VattiScratch, or_idx: usize) {
    let mut or = or_idx;

    while sc.orc_pool[or].pts.is_none() {
        match sc.orc_pool[or].owner {
            Some(o) => or = o,
            None => return,
        };
    }

    if !v_valid_closed(sc, sc.orc_pool[or].pts.unwrap()) {
        sc.orc_pool[or].pts = None;

        return;
    }

    let mut start_op = sc.orc_pool[or].pts.unwrap();
    let mut op2 = start_op;
    let count = v_ring_size(sc, start_op);
    let max_iter = (count + 1) * (count + 1);

    for _ in 0..max_iter {
        let prev = sc.opt_pool[op2].prev;
        let next = sc.opt_pool[op2].next;

        if v_is_collinear(
            sc.opt_pool[prev].pt,
            sc.opt_pool[op2].pt,
            sc.opt_pool[next].pt,
        ) {
            if op2 == sc.orc_pool[or].pts.unwrap() {
                sc.orc_pool[or].pts = Some(prev);
            }

            op2 = v_dispose_outpt(sc, op2);

            if !v_valid_closed(sc, op2) {
                sc.orc_pool[or].pts = None;

                return;
            }

            start_op = op2;
            continue;
        }

        op2 = sc.opt_pool[op2].next;

        if op2 == start_op {
            break;
        }
    }

    v_fix_self_intersects(sc, or);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sweep
// ═══════════════════════════════════════════════════════════════════════════
fn v_locmin_less(sc: &VattiScratch, a: &VLocalMinima, b: &VLocalMinima) -> std::cmp::Ordering {
    let pa = sc.vtx_pool[a.vertex].pt;
    let pb = sc.vtx_pool[b.vertex].pt;

    if pb.y != pa.y {
        return pb.y.cmp(&pa.y);
    }

    pa.x.cmp(&pb.x)
}

fn v_execute_internal(sc: &mut VattiScratch, cliptype: i32) -> bool {
    let mut list = std::mem::take(&mut sc.locmin_list);
    list.sort_by(|a, b| v_locmin_less(sc, a, b));
    sc.locmin_list = list;

    for i in 0..sc.locmin_list.len() {
        let y = sc.vtx_pool[sc.locmin_list[i].vertex].pt.y;
        v_insert_scanline(sc, y);
    }

    sc.locmin_idx = 0;

    let Some(mut y) = v_pop_scanline(sc) else {
        return true;
    };

    while sc.succeeded {
        v_insert_local_minima_into_ael(sc, y, cliptype);

        while let Some(e) = v_pop_horz(sc) {
            v_do_horizontal(sc, e, cliptype);
        }

        if !sc.horz_seg_list.is_empty() {
            v_convert_horz_segs_to_joins(sc);
            sc.horz_seg_list.clear();
        }

        sc.bot_y = y;

        match v_pop_scanline(sc) {
            Some(next) => y = next,
            None => break,
        }

        if sc.succeeded && v_build_intersect_list(sc, y) {
            v_process_intersect_list(sc, cliptype);
            sc.intersect_nodes.clear();
        }

        v_do_top_of_scanbeam(sc, y, cliptype);

        while let Some(e) = v_pop_horz(sc) {
            v_do_horizontal(sc, e, cliptype);
        }
    }

    if sc.succeeded {
        v_process_horz_joins(sc);
    }

    sc.succeeded
}

// ═══════════════════════════════════════════════════════════════════════════
// Fast paths and extraction
// ═══════════════════════════════════════════════════════════════════════════
/// Drop a closing point that repeats the first one.
fn v_strip_closing(c: &[f64], n: usize) -> usize {
    if n < 2 {
        return n;
    }

    let dx = c[(n - 1) * 3] - c[0];
    let dy = c[(n - 1) * 3 + 1] - c[1];

    if dx * dx + dy * dy < 1e-20 {
        return n - 1;
    }

    n
}

/// Integer scale so that (max_coord * scale)^2 fits in int64.
fn v_bool_scale(ca: &[f64], na: usize, cb: &[f64], nb: usize) -> f64 {
    let mut max_coord: f64 = 0.0;

    for i in 0..na {
        max_coord = max_coord.max(ca[i * 3].abs()).max(ca[i * 3 + 1].abs());
    }

    for i in 0..nb {
        max_coord = max_coord.max(cb[i * 3].abs()).max(cb[i * 3 + 1].abs());
    }

    if max_coord < 1e-12 {
        max_coord = 1.0;
    }

    ((i64::MAX as f64).sqrt() / (2.0 * max_coord)).floor()
}

/// Result of a boolean when one polygon contains the other or they are disjoint.
fn v_select(
    a: &Polyline,
    b: &Polyline,
    a_in_b: bool,
    b_in_a: bool,
    clip_type: i32,
) -> Vec<Polyline> {
    if clip_type == 0 {
        if a_in_b {
            return vec![a.clone()];
        }

        if b_in_a {
            return vec![b.clone()];
        }

        return vec![];
    }

    if clip_type == 1 {
        if a_in_b {
            return vec![b.clone()];
        }

        if b_in_a {
            return vec![a.clone()];
        }

        return vec![a.clone(), b.clone()];
    }

    if a_in_b {
        return vec![];
    }

    vec![a.clone()]
}

fn v_select_count(
    a_count: usize,
    b_count: usize,
    a_in_b: bool,
    b_in_a: bool,
    clip_type: i32,
) -> i32 {
    if clip_type == 0 {
        if a_in_b {
            return a_count as i32;
        }

        if b_in_a {
            return b_count as i32;
        }

        return 0;
    }

    if clip_type == 1 {
        if a_in_b {
            return b_count as i32;
        }

        if b_in_a {
            return a_count as i32;
        }

        return (a_count + b_count) as i32;
    }

    if a_in_b {
        return 0;
    }

    a_count as i32
}

fn v_bounds(v: &[BIVec2]) -> (i64, i64, i64, i64) {
    let (mut min_x, mut max_x, mut min_y, mut max_y) = (v[0].x, v[0].x, v[0].y, v[0].y);

    for p in &v[1..] {
        if p.x < min_x {
            min_x = p.x;
        } else if p.x > max_x {
            max_x = p.x;
        }

        if p.y < min_y {
            min_y = p.y;
        } else if p.y > max_y {
            max_y = p.y;
        }
    }

    (min_x, max_x, min_y, max_y)
}

fn v_any_touch(va: &[BIVec2], vb: &[BIVec2]) -> bool {
    let na = va.len();
    let nb = vb.len();

    for i in 0..na {
        let a1 = va[i];
        let a2 = va[(i + 1) % na];
        let axmin = a1.x.min(a2.x);
        let axmax = a1.x.max(a2.x);
        let aymin = a1.y.min(a2.y);
        let aymax = a1.y.max(a2.y);

        for j in 0..nb {
            let b1 = vb[j];
            let b2 = vb[(j + 1) % nb];

            if b1.x.max(b2.x) < axmin
                || b1.x.min(b2.x) > axmax
                || b1.y.max(b2.y) < aymin
                || b1.y.min(b2.y) > aymax
            {
                continue;
            }

            if v_segs_touch(a1, a2, b1, b2) {
                return true;
            }
        }
    }

    false
}

/// Add both inputs through an integer copy, or return the result when their boundaries do not touch.
fn v_add_small_paths(
    sc: &mut VattiScratch,
    a: &Polyline,
    b: &Polyline,
    na: usize,
    nb: usize,
    bool_scale: f64,
    clip_type: i32,
) -> Option<Vec<Polyline>> {
    let mut va = Vec::with_capacity(na);
    let mut vb = Vec::with_capacity(nb);

    for i in 0..na {
        va.push(v_cvt_to_i64(&a.coords[i * 3..], bool_scale));
    }

    for i in 0..nb {
        vb.push(v_cvt_to_i64(&b.coords[i * 3..], bool_scale));
    }

    let (a_min_x, a_max_x, a_min_y, a_max_y) = v_bounds(&va);
    let (b_min_x, b_max_x, b_min_y, b_max_y) = v_bounds(&vb);

    if a_max_x < b_min_x || b_max_x < a_min_x || a_max_y < b_min_y || b_max_y < a_min_y {
        return Some(v_select(
            a,
            b,
            pip_i(va[0], &vb),
            pip_i(vb[0], &va),
            clip_type,
        ));
    }

    if !v_any_touch(&va, &vb) {
        return Some(v_select(
            a,
            b,
            pip_i(va[0], &vb),
            pip_i(vb[0], &va),
            clip_type,
        ));
    }

    v_add_path(sc, &va, na, 0);
    v_add_path(sc, &vb, nb, 1);

    None
}

/// Add both inputs straight from doubles, or return the result when their bounds do not overlap.
fn v_add_large_paths(
    sc: &mut VattiScratch,
    a: &Polyline,
    b: &Polyline,
    na: usize,
    nb: usize,
    bool_scale: f64,
    clip_type: i32,
) -> Option<Vec<Polyline>> {
    let (va_head, a_min_x, a_max_x, a_min_y, a_max_y) =
        v_add_path_from_doubles(sc, &a.coords, na, 0, bool_scale);
    let (vb_head, b_min_x, b_max_x, b_min_y, b_max_y) =
        v_add_path_from_doubles(sc, &b.coords, nb, 1, bool_scale);

    let (Some(va_head), Some(vb_head)) = (va_head, vb_head) else {
        return Some(vec![]);
    };

    if a_max_x < b_min_x || b_max_x < a_min_x || a_max_y < b_min_y || b_max_y < a_min_y {
        let a_in_b = pip_vertex(sc, sc.vtx_pool[va_head].pt, vb_head);
        let b_in_a = pip_vertex(sc, sc.vtx_pool[vb_head].pt, va_head);

        return Some(v_select(a, b, a_in_b, b_in_a, clip_type));
    }

    None
}

/// First output point of a finished ring, or None when the ring is degenerate.
fn v_ring_start(sc: &mut VattiScratch, outrec: usize) -> Option<usize> {
    sc.orc_pool[outrec].pts?;
    v_clean_collinear(sc, outrec);
    let op = sc.orc_pool[outrec].pts?;

    if sc.opt_pool[op].next == op
        || sc.opt_pool[op].next == sc.opt_pool[op].prev
        || v_very_small_tri(sc, op)
    {
        return None;
    }

    Some(op)
}

fn v_extract(sc: &mut VattiScratch, inv_scale: f64) -> Vec<Polyline> {
    let mut out = Vec::new();
    let mut i = 0;

    while i < sc.outrec_list.len() {
        let outrec = sc.outrec_list[i];
        i += 1;

        let Some(op) = v_ring_start(sc, outrec) else {
            continue;
        };

        let mut coords = Vec::new();
        let mut o = sc.opt_pool[op].next;
        let mut last = sc.opt_pool[o].pt;
        v_cvt_to_dbl(&mut coords, last, inv_scale);
        o = sc.opt_pool[o].next;

        while o != sc.opt_pool[op].next {
            if sc.opt_pool[o].pt != last {
                last = sc.opt_pool[o].pt;
                v_cvt_to_dbl(&mut coords, last, inv_scale);
            }

            o = sc.opt_pool[o].next;
        }

        if coords.len() < 9 {
            continue;
        }

        out.push(Polyline::from_coords(coords));
    }

    out
}

/// Vatti boolean operations on closed planar polylines.
pub struct BooleanPolyline;

impl BooleanPolyline {
    // ═══════════════════════════════════════════════════════════════════════════
    // Boolean operations
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compute the Vatti boolean of two closed planar polylines with clip_type 0 intersection, 1 union, 2 a minus b.
    pub fn compute(a: &Polyline, b: &Polyline, clip_type: i32) -> Vec<Polyline> {
        let ca = &a.coords;
        let cb = &b.coords;
        let na = v_strip_closing(ca, ca.len() / 3);
        let nb = v_strip_closing(cb, cb.len() / 3);

        if na < 3 || nb < 3 {
            return vec![];
        }

        let bool_scale = v_bool_scale(ca, na, cb, nb);

        SCRATCH.with(|cell| {
            let sc = &mut *cell.borrow_mut();
            sc.reset(na + nb);

            let early = if na * nb <= 400 {
                v_add_small_paths(sc, a, b, na, nb, bool_scale, clip_type)
            } else {
                v_add_large_paths(sc, a, b, na, nb, bool_scale, clip_type)
            };

            if let Some(result) = early {
                return result;
            }

            if !v_execute_internal(sc, clip_type) {
                return vec![];
            }

            v_extract(sc, 1.0 / bool_scale)
        })
    }

    /// Compute the nonzero Vatti boolean of two sets of closed rings in xy with clip_type 0 intersection, 1 union, 2 a minus b, each set first turned outer counter-clockwise and holes clockwise by nesting depth; returns closed rings, outer counter-clockwise and holes clockwise, where compute returns open ones.
    pub fn compute_regions(a: &[Polyline], b: &[Polyline], clip_type: i32) -> Vec<Polyline> {
        let rings_a = v_oriented(a);
        let rings_b = v_oriented(b);
        let mut ca = Vec::new();
        let mut cb = Vec::new();

        for ring in &rings_a {
            ca.extend_from_slice(ring);
        }

        for ring in &rings_b {
            cb.extend_from_slice(ring);
        }

        let bool_scale = v_bool_scale(&ca, ca.len() / 3, &cb, cb.len() / 3);

        SCRATCH.with(|cell| {
            let sc = &mut *cell.borrow_mut();
            sc.reset(ca.len() / 3 + cb.len() / 3);

            for ring in &rings_a {
                v_add_path_from_doubles(sc, ring, ring.len() / 3, 0, bool_scale);
            }

            for ring in &rings_b {
                v_add_path_from_doubles(sc, ring, ring.len() / 3, 1, bool_scale);
            }

            if !v_execute_internal(sc, clip_type) {
                return vec![];
            }

            let mut rings = v_extract(sc, 1.0 / bool_scale);

            for ring in rings.iter_mut() {
                let first = ring.get_point(0).unwrap();
                ring.add_point(first);
            }

            rings
        })
    }

    /// Return the number of output points of compute without building polylines.
    pub fn compute_count(a: &Polyline, b: &Polyline, clip_type: i32) -> i32 {
        let ca = &a.coords;
        let cb = &b.coords;
        let na = v_strip_closing(ca, ca.len() / 3);
        let nb = v_strip_closing(cb, cb.len() / 3);

        if na < 3 || nb < 3 {
            return 0;
        }

        let bool_scale = v_bool_scale(ca, na, cb, nb);

        SCRATCH.with(|cell| {
            let sc = &mut *cell.borrow_mut();
            sc.reset(na + nb);
            let (va_head, a_min_x, a_max_x, a_min_y, a_max_y) =
                v_add_path_from_doubles(sc, ca, na, 0, bool_scale);
            let (vb_head, b_min_x, b_max_x, b_min_y, b_max_y) =
                v_add_path_from_doubles(sc, cb, nb, 1, bool_scale);

            let (Some(va_head), Some(vb_head)) = (va_head, vb_head) else {
                return 0;
            };

            if a_max_x < b_min_x || b_max_x < a_min_x || a_max_y < b_min_y || b_max_y < a_min_y {
                return v_select_count(
                    ca.len() / 3,
                    cb.len() / 3,
                    pip_vertex(sc, sc.vtx_pool[va_head].pt, vb_head),
                    pip_vertex(sc, sc.vtx_pool[vb_head].pt, va_head),
                    clip_type,
                );
            }

            if !v_execute_internal(sc, clip_type) {
                return 0;
            }

            let mut total = 0;
            let mut i = 0;

            while i < sc.outrec_list.len() {
                let outrec = sc.outrec_list[i];
                i += 1;

                let Some(op) = v_ring_start(sc, outrec) else {
                    continue;
                };

                total += v_ring_size(sc, op) as i32;
            }

            total
        })
    }

    /// Compute on flat xy arrays, write up to max_out result points to out_xy and return the total.
    pub fn compute_raw(
        a_xy: &[f64],
        na: usize,
        b_xy: &[f64],
        nb: usize,
        clip_type: i32,
        out_xy: &mut [f64],
        max_out: usize,
    ) -> i32 {
        let mut a = Polyline::from_coords(vec![0.0; na * 3]);
        let mut b = Polyline::from_coords(vec![0.0; nb * 3]);

        for i in 0..na {
            a.coords[i * 3] = a_xy[i * 2];
            a.coords[i * 3 + 1] = a_xy[i * 2 + 1];
        }

        for i in 0..nb {
            b.coords[i * 3] = b_xy[i * 2];
            b.coords[i * 3 + 1] = b_xy[i * 2 + 1];
        }

        let result = Self::compute(&a, &b, clip_type);
        let mut total = 0usize;

        for r in &result {
            let c = &r.coords;

            for i in 0..c.len() / 3 {
                if total < max_out {
                    out_xy[total * 2] = c[i * 3];
                    out_xy[total * 2 + 1] = c[i * 3 + 1];
                }

                total += 1;
            }
        }

        total as i32
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Open subject against closed clip
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the pieces of an open polyline that lie inside a closed clip polygon in the xy plane.
    pub fn clip_open_against_closed(
        open_subject: &Polyline,
        closed_clip: &Polyline,
    ) -> Vec<Polyline> {
        let mut result: Vec<Polyline> = Vec::new();
        let cs = &open_subject.coords;
        let cc = &closed_clip.coords;
        let ns = cs.len() / 3;
        let nc = v_strip_closing(cc, cc.len() / 3);

        if ns < 2 || nc < 3 {
            return result;
        }

        let mut cur: Vec<f64> = Vec::new();

        if v_point_in_poly(cc, nc, cs[0], cs[1]) {
            v_push_xy(&mut cur, cs[0], cs[1]);
        }

        for si in 0..(ns - 1) {
            let ax = cs[si * 3];
            let ay = cs[si * 3 + 1];
            let bx = cs[(si + 1) * 3];
            let by = cs[(si + 1) * 3 + 1];
            let dx = bx - ax;
            let dy = by - ay;
            let ts = v_crossings(cc, nc, ax, ay, dx, dy);
            let mut prev_t = 0.0;

            for &t in &ts {
                if t - prev_t < 1e-12 {
                    prev_t = t;
                    continue;
                }

                let mid_t = 0.5 * (prev_t + t);
                v_push_xy(&mut cur, ax + dx * t, ay + dy * t);

                if v_point_in_poly(cc, nc, ax + dx * mid_t, ay + dy * mid_t) {
                    v_flush(&mut cur, &mut result);
                }

                prev_t = t;
            }

            if prev_t >= 1.0 - 1e-12 {
                continue;
            }

            let mid_t = 0.5 * (prev_t + 1.0);

            if v_point_in_poly(cc, nc, ax + dx * mid_t, ay + dy * mid_t) {
                v_push_xy(&mut cur, bx, by);
            }
        }

        v_flush(&mut cur, &mut result);

        result
    }
}

/// Even-odd ray cast of (px, py) against the first nc points of cc.
fn v_point_in_poly(cc: &[f64], nc: usize, px: f64, py: f64) -> bool {
    let mut inside = false;
    let mut j = nc - 1;

    for i in 0..nc {
        let xi = cc[i * 3];
        let yi = cc[i * 3 + 1];
        let xj = cc[j * 3];
        let yj = cc[j * 3 + 1];
        j = i;

        if (yi > py) == (yj > py) {
            continue;
        }

        let xint = xj + (py - yj) * (xi - xj) / (yi - yj);

        if px < xint {
            inside = !inside;
        }
    }

    inside
}

/// Sorted parameters in (0, 1] where segment (a, b) crosses an edge of the clip.
fn v_crossings(cc: &[f64], nc: usize, ax: f64, ay: f64, dx: f64, dy: f64) -> Vec<f64> {
    let mut ts: Vec<f64> = Vec::new();
    let mut j = nc - 1;

    for i in 0..nc {
        let ex = cc[i * 3] - cc[j * 3];
        let ey = cc[i * 3 + 1] - cc[j * 3 + 1];
        let rx = cc[j * 3] - ax;
        let ry = cc[j * 3 + 1] - ay;
        j = i;
        let denom = dy * ex - dx * ey;

        if denom.abs() < 1e-18 {
            continue;
        }

        let t = (ry * ex - rx * ey) / denom;
        let u = (ry * dx - rx * dy) / denom;

        if t > 1e-12 && t <= 1.0 + 1e-12 && (-1e-9..=1.0 + 1e-9).contains(&u) {
            ts.push(t.clamp(0.0, 1.0));
        }
    }

    ts.sort_by(|a, b| a.total_cmp(b));

    ts
}

fn v_push_xy(cur: &mut Vec<f64>, x: f64, y: f64) {
    let n = cur.len();

    if n >= 3 && (cur[n - 3] - x).abs() < 1e-9 && (cur[n - 2] - y).abs() < 1e-9 {
        return;
    }

    cur.push(x);
    cur.push(y);
    cur.push(0.0);
}

fn v_flush(cur: &mut Vec<f64>, result: &mut Vec<Polyline>) {
    if cur.len() >= 6 {
        result.push(Polyline::from_coords(cur.clone()));
    }

    cur.clear();
}

// ═══════════════════════════════════════════════════════════════════════════
// Ring sets
// ═══════════════════════════════════════════════════════════════════════════
/// Signed xy area of the first count points of flat coordinates, positive counter-clockwise.
fn v_ring_area(coords: &[f64], count: usize) -> f64 {
    let mut area = 0.0;

    for i in 0..count {
        area += coords[i * 3] * coords[((i + 1) % count) * 3 + 1]
            - coords[((i + 1) % count) * 3] * coords[i * 3 + 1];
    }

    area / 2.0
}

/// The rings of one operand as flat coordinates without closing points, outer counter-clockwise and holes clockwise by how many other rings hold a point just inside each.
fn v_oriented(rings: &[Polyline]) -> Vec<Vec<f64>> {
    let mut flat: Vec<Vec<f64>> = Vec::new();

    for ring in rings {
        let count = v_strip_closing(&ring.coords, ring.coords.len() / 3);

        if count >= 3 {
            flat.push(ring.coords[..count * 3].to_vec());
        }
    }

    let mut oriented = flat.clone();

    for i in 0..flat.len() {
        let count = flat[i].len() / 3;
        let area = v_ring_area(&flat[i], count);
        let dx = flat[i][3] - flat[i][0];
        let dy = flat[i][4] - flat[i][1];
        let side = if area > 0.0 {
            Tolerance::RELATIVE
        } else {
            -Tolerance::RELATIVE
        };
        let px = (flat[i][0] + flat[i][3]) * 0.5 - dy * side;
        let py = (flat[i][1] + flat[i][4]) * 0.5 + dx * side;
        let mut depth = 0;

        for (j, other) in flat.iter().enumerate() {
            if j != i && v_point_in_poly(other, other.len() / 3, px, py) {
                depth += 1;
            }
        }

        if (area > 0.0) == (depth % 2 == 0) {
            continue;
        }

        for k in 0..count {
            for axis in 0..3 {
                oriented[i][k * 3 + axis] = flat[i][(count - 1 - k) * 3 + axis];
            }
        }
    }

    oriented
}
