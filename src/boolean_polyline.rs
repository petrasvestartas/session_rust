use crate::Polyline;

const VF_LOCAL_MAX: u32 = 4;
const VF_LOCAL_MIN: u32 = 8;

type OptIdx = Option<usize>;

#[derive(Clone, Copy, Default, PartialEq, Eq)]
struct BIVec2 {
    x: i64,
    y: i64,
}

#[derive(Clone, Default)]
struct VVertex {
    pt: BIVec2,
    next: OptIdx,
    prev: OptIdx,
    flags: u32,
}

#[derive(Clone)]
struct VLocalMinima {
    vertex: usize,
    polytype: i8,
}

#[derive(Clone, Default)]
struct VOutPt {
    pt: BIVec2,
    next: usize,
    prev: usize,
    outrec: usize,
    horz: bool,
}

#[derive(Clone, Default)]
struct VOutRec {
    idx: usize,
    front_edge: OptIdx,
    back_edge: OptIdx,
    pts: OptIdx,
    owner: OptIdx,
}

#[derive(Clone, Default)]
struct VActive {
    bot: BIVec2,
    top: BIVec2,
    curr_x: i64,
    dx: f64,
    wind_dx: i32,
    wind_cnt: i32,
    wind_cnt2: i32,
    outrec: OptIdx,
    prev_in_ael: OptIdx,
    next_in_ael: OptIdx,
    prev_in_sel: OptIdx,
    next_in_sel: OptIdx,
    jump: OptIdx,
    vertex_top: OptIdx,
    local_min: OptIdx,
    is_left_bound: bool,
    join_with: i8,
}

struct VIntersectNode {
    pt: BIVec2,
    edge1: usize,
    edge2: usize,
}

#[derive(Clone)]
struct VHorzSeg {
    left_op: usize,
    right_op: OptIdx,
    left_to_right: bool,
}

struct VHorzJoin {
    op1: usize,
    op2: usize,
}

/// Arena of the sweep: every pointer of the C++ engine is an index into one of these pools.
struct VattiScratch {
    vtx_pool: Vec<VVertex>,
    act_pool: Vec<VActive>,
    opt_pool: Vec<VOutPt>,
    orc_pool: Vec<VOutRec>,
    locmin_list: Vec<VLocalMinima>,
    intersect_nodes: Vec<VIntersectNode>,
    horz_seg_list: Vec<VHorzSeg>,
    horz_join_list: Vec<VHorzJoin>,
    outrec_list: Vec<usize>,
    scanline_list: Vec<i64>,
    actives: OptIdx,
    sel: OptIdx,
    bot_y: i64,
    locmin_idx: usize,
    succeeded: bool,
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
            scanline_list: Vec::new(),
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
        self.scanline_list.reserve(total * 2);
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
    fn insert_scanline(&mut self, y: i64) {
        self.scanline_list.push(y);
        let mut i = self.scanline_list.len() - 1;
        while i > 0 {
            let p = (i - 1) / 2;
            if self.scanline_list[p] >= self.scanline_list[i] {
                break;
            }
            self.scanline_list.swap(p, i);
            i = p;
        }
    }
    fn pop_top(&mut self) {
        let last = self.scanline_list.len() - 1;
        self.scanline_list.swap(0, last);
        self.scanline_list.pop();
        let n = self.scanline_list.len();
        let mut i = 0;
        loop {
            let l = 2 * i + 1;
            let r = 2 * i + 2;
            let mut m = i;
            if l < n && self.scanline_list[l] > self.scanline_list[m] {
                m = l;
            }
            if r < n && self.scanline_list[r] > self.scanline_list[m] {
                m = r;
            }
            if m == i {
                break;
            }
            self.scanline_list.swap(i, m);
            i = m;
        }
    }
    fn pop_scanline(&mut self) -> Option<i64> {
        if self.scanline_list.is_empty() {
            return None;
        }
        let y = self.scanline_list[0];
        self.pop_top();
        while !self.scanline_list.is_empty() && self.scanline_list[0] == y {
            self.pop_top();
        }
        Some(y)
    }
    fn pop_locmin(&mut self, y: i64) -> Option<VLocalMinima> {
        if self.locmin_idx >= self.locmin_list.len() {
            return None;
        }
        let vertex = self.locmin_list[self.locmin_idx].vertex;
        if self.vtx_pool[vertex].pt.y != y {
            return None;
        }
        let lm = self.locmin_list[self.locmin_idx].clone();
        self.locmin_idx += 1;
        Some(lm)
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
        a.bot.x + (a.dx * (y - a.bot.y) as f64).round() as i64
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

fn v_next_vertex(sc: &VattiScratch, e: usize) -> OptIdx {
    let a = &sc.act_pool[e];
    let vt = a.vertex_top?;
    if a.wind_dx > 0 {
        sc.vtx_pool[vt].next
    } else {
        sc.vtx_pool[vt].prev
    }
}

fn v_prev_prev_vertex(sc: &VattiScratch, e: usize) -> OptIdx {
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
fn v_dot_product(p1: BIVec2, p2: BIVec2, p3: BIVec2) -> f64 {
    (p2.x - p1.x) as f64 * (p3.x - p2.x) as f64 + (p2.y - p1.y) as f64 * (p3.y - p2.y) as f64
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
            x: a.x + (t * dx1).round() as i64,
            y: a.y + (t * dy1).round() as i64,
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
        x: s1.x + (q * dx).round() as i64,
        y: s1.y + (q * dy).round() as i64,
    }
}

fn v_sign_d(v: f64) -> i32 {
    (v > 0.0) as i32 - (v < 0.0) as i32
}

fn v_segs_intersect(a: BIVec2, b: BIVec2, c: BIVec2, d: BIVec2) -> bool {
    (v_sign_d(v_cross_product(a, c, d)) * v_sign_d(v_cross_product(b, c, d)) < 0)
        && (v_sign_d(v_cross_product(c, a, b)) * v_sign_d(v_cross_product(d, a, b)) < 0)
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

/// Links n scaled points into a circular vertex list; returns its head or None if degenerate.
fn v_link_path(sc: &mut VattiScratch, pts: &[BIVec2], n: usize, polytype: i8) -> OptIdx {
    let base = sc.vtx_pool.len();
    sc.vtx_pool.resize(base + n, VVertex::default());
    sc.vtx_pool[base].pt = pts[0];
    let mut prev_v = base;
    let mut cnt = 1;
    for pt in pts.iter().take(n).skip(1) {
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

fn v_bounds(v: &[BIVec2]) -> (i64, i64, i64, i64) {
    let (mut min_x, mut max_x, mut min_y, mut max_y) = (v[0].x, v[0].x, v[0].y, v[0].y);
    for p in v.iter().skip(1) {
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

fn v_cvt_to_i64(coords: &[f64], n: usize, bool_scale: f64) -> Vec<BIVec2> {
    let mut pts = Vec::with_capacity(n);
    for i in 0..n {
        pts.push(BIVec2 {
            x: (coords[i * 3] * bool_scale).round() as i64,
            y: (coords[i * 3 + 1] * bool_scale).round() as i64,
        });
    }
    pts
}

fn v_add_path_from_doubles(
    sc: &mut VattiScratch,
    coords: &[f64],
    n: usize,
    polytype: i8,
    bool_scale: f64,
) -> (OptIdx, i64, i64, i64, i64) {
    if n < 3 {
        return (None, 0, 0, 0, 0);
    }
    let pts = v_cvt_to_i64(coords, n, bool_scale);
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

fn v_get_maxima_pair(sc: &VattiScratch, e: usize) -> OptIdx {
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

fn v_get_curr_y_maxima(sc: &VattiScratch, e: usize) -> OptIdx {
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

fn v_get_prev_hot(sc: &VattiScratch, e: usize) -> OptIdx {
    let mut p = sc.act_pool[e].prev_in_ael;
    while let Some(pi) = p {
        if v_is_hot(sc, pi) {
            return Some(pi);
        }
        p = sc.act_pool[pi].prev_in_ael;
    }
    None
}

fn v_is_valid_ael_order(sc: &VattiScratch, res: usize, new: usize) -> bool {
    if sc.act_pool[new].curr_x != sc.act_pool[res].curr_x {
        return sc.act_pool[new].curr_x > sc.act_pool[res].curr_x;
    }
    let d = v_cross_product(
        sc.act_pool[res].top,
        sc.act_pool[new].bot,
        sc.act_pool[new].top,
    );
    if d != 0.0 {
        return d < 0.0;
    }
    if !v_is_maxima_e(sc, res) && sc.act_pool[res].top.y > sc.act_pool[new].top.y {
        return v_cross_product(
            sc.act_pool[new].bot,
            sc.act_pool[res].top,
            sc.vtx_pool[v_next_vertex(sc, res).unwrap()].pt,
        ) <= 0.0;
    }
    if !v_is_maxima_e(sc, new) && sc.act_pool[new].top.y > sc.act_pool[res].top.y {
        return v_cross_product(
            sc.act_pool[new].bot,
            sc.act_pool[new].top,
            sc.vtx_pool[v_next_vertex(sc, new).unwrap()].pt,
        ) >= 0.0;
    }
    let y = sc.act_pool[new].bot.y;
    if sc.act_pool[res].bot.y != y
        || sc.vtx_pool[sc.locmin_list[sc.act_pool[res].local_min.unwrap()].vertex]
            .pt
            .y
            != y
    {
        return sc.act_pool[new].is_left_bound;
    }
    if sc.act_pool[res].is_left_bound != sc.act_pool[new].is_left_bound {
        return sc.act_pool[new].is_left_bound;
    }
    let pp_res = v_prev_prev_vertex(sc, res);
    if pp_res.is_some()
        && v_is_collinear(
            sc.vtx_pool[pp_res.unwrap()].pt,
            sc.act_pool[res].bot,
            sc.act_pool[res].top,
        )
    {
        return true;
    }
    let pp_new = v_prev_prev_vertex(sc, new);
    match (pp_res, pp_new) {
        (Some(pp_res), Some(pp_new)) => {
            (v_cross_product(
                sc.vtx_pool[pp_res].pt,
                sc.act_pool[new].bot,
                sc.vtx_pool[pp_new].pt,
            ) > 0.0)
                == sc.act_pool[new].is_left_bound
        }
        _ => sc.act_pool[new].is_left_bound,
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

fn v_add_local_max_poly(sc: &mut VattiScratch, e1: usize, e2: usize, pt: BIVec2) -> OptIdx {
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

fn v_check_join_left(sc: &mut VattiScratch, e: usize, pt: BIVec2, check_cx: bool) {
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
    if check_cx {
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

fn v_check_join_right(sc: &mut VattiScratch, e: usize, pt: BIVec2, check_cx: bool) {
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
    if check_cx {
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

fn v_intersect_edges(sc: &mut VattiScratch, e1: usize, e2: usize, pt: BIVec2, cliptype: i32) {
    if v_is_joined(sc, e1) {
        v_split(sc, e1, pt);
    }
    if v_is_joined(sc, e2) {
        v_split(sc, e2, pt);
    }
    if v_polytype(sc, e1) == v_polytype(sc, e2) {
        let (wc, wd, ewd) = (
            sc.act_pool[e1].wind_cnt,
            sc.act_pool[e2].wind_dx,
            sc.act_pool[e1].wind_dx,
        );
        sc.act_pool[e1].wind_cnt = if wc + wd == 0 { -wc } else { wc + wd };
        let (wc2, wd2) = (sc.act_pool[e2].wind_cnt, ewd);
        sc.act_pool[e2].wind_cnt = if wc2 - wd2 == 0 { -wc2 } else { wc2 - wd2 };
    } else {
        sc.act_pool[e1].wind_cnt2 += sc.act_pool[e2].wind_dx;
        sc.act_pool[e2].wind_cnt2 -= sc.act_pool[e1].wind_dx;
    }
    let ow1 = sc.act_pool[e1].wind_cnt.abs();
    let ow2 = sc.act_pool[e2].wind_cnt.abs();
    let in01_1 = ow1 == 0 || ow1 == 1;
    let in01_2 = ow2 == 0 || ow2 == 1;
    if (!v_is_hot(sc, e1) && !in01_1) || (!v_is_hot(sc, e2) && !in01_2) {
        return;
    }
    if v_is_hot(sc, e1) && v_is_hot(sc, e2) {
        if (ow1 != 0 && ow1 != 1)
            || (ow2 != 0 && ow2 != 1)
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
        let wc2_1 = sc.act_pool[e1].wind_cnt2.abs() as i64;
        let wc2_2 = sc.act_pool[e2].wind_cnt2.abs() as i64;
        if !v_same_polytype(sc, e1, e2) {
            v_add_local_min_poly(sc, e1, e2, pt, false);
        } else if ow1 == 1 && ow2 == 1 {
            match cliptype {
                0 => {
                    if wc2_1 > 0 && wc2_2 > 0 {
                        v_add_local_min_poly(sc, e1, e2, pt, false);
                    }
                }
                1 => {
                    if wc2_1 <= 0 && wc2_2 <= 0 {
                        v_add_local_min_poly(sc, e1, e2, pt, false);
                    }
                }
                _ => {
                    if (v_polytype(sc, e1) == 1 && wc2_1 > 0 && wc2_2 > 0)
                        || (v_polytype(sc, e1) == 0 && wc2_1 <= 0 && wc2_2 <= 0)
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

fn v_update_edge_into_ael(sc: &mut VattiScratch, e: usize) {
    let nv = v_next_vertex(sc, e).unwrap();
    sc.act_pool[e].bot = sc.act_pool[e].top;
    sc.act_pool[e].vertex_top = Some(nv);
    sc.act_pool[e].top = sc.vtx_pool[nv].pt;
    sc.act_pool[e].curr_x = sc.act_pool[e].bot.x;
    sc.act_pool[e].dx = v_get_dx(sc.act_pool[e].bot, sc.act_pool[e].top);
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
        sc.act_pool[e].dx = v_get_dx(sc.act_pool[e].bot, sc.act_pool[e].top);
        return;
    }
    sc.insert_scanline(sc.act_pool[e].top.y);
    v_check_join_left(sc, e, sc.act_pool[e].bot, false);
    v_check_join_right(sc, e, sc.act_pool[e].bot, true);
}

fn v_reset_horz_dir(sc: &VattiScratch, e: usize, max_v: OptIdx) -> (bool, i64, i64) {
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

fn v_do_horizontal(sc: &mut VattiScratch, horz: usize, cliptype: i32) {
    let y = sc.act_pool[horz].bot.y;
    let vertex_max = v_get_curr_y_maxima(sc, horz);
    let (mut is_ltr, mut hl, mut hr) = v_reset_horz_dir(sc, horz, vertex_max);
    if v_is_hot(sc, horz) {
        let op = v_add_outpt(
            sc,
            horz,
            BIVec2 {
                x: sc.act_pool[horz].curr_x,
                y,
            },
        );
        sc.horz_seg_list.push(VHorzSeg {
            left_op: op,
            right_op: None,
            left_to_right: true,
        });
    }
    loop {
        let mut ei = if is_ltr {
            sc.act_pool[horz].next_in_ael
        } else {
            sc.act_pool[horz].prev_in_ael
        };
        while let Some(e) = ei {
            if sc.act_pool[e].vertex_top == vertex_max {
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
                return;
            }
            if vertex_max != sc.act_pool[horz].vertex_top {
                if (is_ltr && sc.act_pool[e].curr_x > hr) || (!is_ltr && sc.act_pool[e].curr_x < hl)
                {
                    break;
                }
                if sc.act_pool[e].curr_x == sc.act_pool[horz].top.x && !v_is_horizontal(sc, e) {
                    let pt2 = sc.vtx_pool[v_next_vertex(sc, horz).unwrap()].pt;
                    if is_ltr {
                        if v_top_x(sc, e, pt2.y) >= pt2.x {
                            break;
                        }
                    } else {
                        if v_top_x(sc, e, pt2.y) <= pt2.x {
                            break;
                        }
                    }
                }
            }
            let pt = BIVec2 {
                x: sc.act_pool[e].curr_x,
                y: sc.act_pool[horz].bot.y,
            };
            if is_ltr {
                v_intersect_edges(sc, horz, e, pt, cliptype);
                v_swap_positions_in_ael(sc, horz, e);
                v_check_join_left(sc, e, pt, false);
                sc.act_pool[horz].curr_x = sc.act_pool[e].curr_x;
                ei = sc.act_pool[horz].next_in_ael;
            } else {
                v_intersect_edges(sc, e, horz, pt, cliptype);
                v_swap_positions_in_ael(sc, e, horz);
                v_check_join_right(sc, e, pt, false);
                sc.act_pool[horz].curr_x = sc.act_pool[e].curr_x;
                ei = sc.act_pool[horz].prev_in_ael;
            }
            if sc.act_pool[horz].outrec.is_some() {
                let last_op = {
                    let or = sc.act_pool[horz].outrec.unwrap();
                    let pts = sc.orc_pool[or].pts.unwrap();
                    if sc.orc_pool[or].front_edge == Some(horz) {
                        pts
                    } else {
                        sc.opt_pool[pts].next
                    }
                };
                sc.horz_seg_list.push(VHorzSeg {
                    left_op: last_op,
                    right_op: None,
                    left_to_right: true,
                });
            }
        }
        let nv = v_next_vertex(sc, horz);
        if nv.is_none() || sc.vtx_pool[nv.unwrap()].pt.y != sc.act_pool[horz].top.y {
            break;
        }
        if v_is_hot(sc, horz) {
            v_add_outpt(sc, horz, sc.act_pool[horz].top);
        }
        v_update_edge_into_ael(sc, horz);
        let r = v_reset_horz_dir(sc, horz, vertex_max);
        is_ltr = r.0;
        hl = r.1;
        hr = r.2;
    }
    if v_is_hot(sc, horz) {
        let op = v_add_outpt(sc, horz, sc.act_pool[horz].top);
        sc.horz_seg_list.push(VHorzSeg {
            left_op: op,
            right_op: None,
            left_to_right: true,
        });
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

fn v_convert_horz_segs_to_joins(sc: &mut VattiScratch) {
    let n = sc.horz_seg_list.len();
    for i in 0..n {
        let op = sc.horz_seg_list[i].left_op;
        let mut or = sc.opt_pool[op].outrec;
        while sc.orc_pool[or].pts.is_none() {
            or = sc.orc_pool[or].owner.unwrap_or(or);
            if sc.orc_pool[or].pts.is_some() || sc.orc_pool[or].owner.is_none() {
                break;
            }
        }
        if sc.orc_pool[or].pts.is_none() {
            sc.horz_seg_list[i].right_op = None;
            continue;
        }
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
        if sc.opt_pool[op_p].pt.x == sc.opt_pool[op_n].pt.x {
            sc.horz_seg_list[i].right_op = None;
            continue;
        }
        if sc.opt_pool[op_p].pt.x < sc.opt_pool[op_n].pt.x {
            sc.horz_seg_list[i].left_op = op_p;
            sc.horz_seg_list[i].right_op = Some(op_n);
            sc.horz_seg_list[i].left_to_right = true;
        } else {
            sc.horz_seg_list[i].left_op = op_n;
            sc.horz_seg_list[i].right_op = Some(op_p);
            sc.horz_seg_list[i].left_to_right = false;
        }
        if sc.opt_pool[sc.horz_seg_list[i].left_op].horz {
            sc.horz_seg_list[i].right_op = None;
            continue;
        }
        sc.opt_pool[sc.horz_seg_list[i].left_op].horz = true;
    }
    let valid: usize = sc
        .horz_seg_list
        .iter()
        .filter(|h| h.right_op.is_some())
        .count();
    if valid < 2 {
        return;
    }
    sc.horz_seg_list.sort_by(|a, b| {
        if a.right_op.is_none() || b.right_op.is_none() {
            return a.right_op.is_some().cmp(&b.right_op.is_some()).reverse();
        }
        sc.opt_pool[b.left_op]
            .pt
            .x
            .cmp(&sc.opt_pool[a.left_op].pt.x)
            .reverse()
    });
    let j = valid;
    for i in 0..j.saturating_sub(1) {
        for k in (i + 1)..j {
            let hs1_lo = sc.horz_seg_list[i].left_op;
            let hs1_ro = sc.horz_seg_list[i].right_op.unwrap();
            let hs2_lo = sc.horz_seg_list[k].left_op;
            let hs2_ro = sc.horz_seg_list[k].right_op.unwrap();
            if sc.opt_pool[hs2_lo].pt.x >= sc.opt_pool[hs1_ro].pt.x
                || sc.horz_seg_list[k].left_to_right == sc.horz_seg_list[i].left_to_right
                || sc.opt_pool[hs2_ro].pt.x <= sc.opt_pool[hs1_lo].pt.x
            {
                continue;
            }
            let cy = sc.opt_pool[hs1_lo].pt.y;
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
                let d1 = v_dup_outpt(sc, lo1, true);
                let d2 = v_dup_outpt(sc, lo2, false);
                sc.horz_join_list.push(VHorzJoin { op1: d1, op2: d2 });
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
                let d1 = v_dup_outpt(sc, lo2, true);
                let d2 = v_dup_outpt(sc, lo1, false);
                sc.horz_join_list.push(VHorzJoin { op1: d1, op2: d2 });
            }
            sc.horz_seg_list[i].left_op = lo1;
            sc.horz_seg_list[k].left_op = lo2;
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
    if sc.actives.is_none() {
        return false;
    }
    let first = sc.actives.unwrap();
    if sc.act_pool[first].next_in_ael.is_none() {
        return false;
    }
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
    let mut left_opt = sc.sel;
    while left_opt.is_some() && sc.act_pool[left_opt.unwrap()].jump.is_some() {
        let mut prev_base: OptIdx = None;
        while left_opt.is_some() && sc.act_pool[left_opt.unwrap()].jump.is_some() {
            let left_start = left_opt.unwrap();
            let mut curr_base = left_start;
            let right_start = sc.act_pool[left_start].jump.unwrap();
            let mut l_end: OptIdx = Some(right_start);
            let r_end = sc.act_pool[right_start].jump;
            sc.act_pool[left_start].jump = r_end;
            let mut left = left_opt;
            let mut right = Some(right_start);
            while left != l_end && right != r_end {
                if sc.act_pool[right.unwrap()].curr_x < sc.act_pool[left.unwrap()].curr_x {
                    let ri = right.unwrap();
                    let mut tmp = sc.act_pool[ri].prev_in_sel;
                    loop {
                        let ti = tmp.unwrap();
                        v_add_new_isect_node(sc, ti, ri, top_y);
                        if tmp == left {
                            break;
                        }
                        tmp = sc.act_pool[ti].prev_in_sel;
                    }
                    let ri_next = sc.act_pool[ri].next_in_sel;
                    if let Some(rn) = ri_next {
                        sc.act_pool[rn].prev_in_sel = sc.act_pool[ri].prev_in_sel;
                    }
                    let ri_prev = sc.act_pool[ri].prev_in_sel.unwrap();
                    sc.act_pool[ri_prev].next_in_sel = ri_next;
                    right = ri_next;
                    l_end = right;
                    let li = left.unwrap();
                    sc.act_pool[ri].prev_in_sel = sc.act_pool[li].prev_in_sel;
                    if let Some(p) = sc.act_pool[ri].prev_in_sel {
                        sc.act_pool[p].next_in_sel = Some(ri);
                    }
                    sc.act_pool[ri].next_in_sel = Some(li);
                    sc.act_pool[li].prev_in_sel = Some(ri);
                    if left == Some(curr_base) {
                        curr_base = ri;
                        sc.act_pool[curr_base].jump = r_end;
                        match prev_base {
                            None => sc.sel = Some(curr_base),
                            Some(pb) => sc.act_pool[pb].jump = Some(curr_base),
                        }
                    }
                } else {
                    left = sc.act_pool[left.unwrap()].next_in_sel;
                }
            }
            prev_base = Some(curr_base);
            left_opt = r_end;
        }
        left_opt = sc.sel;
    }
    !sc.intersect_nodes.is_empty()
}

fn v_process_intersect_list(sc: &mut VattiScratch, cliptype: i32) {
    sc.intersect_nodes.sort_by(|a, b| {
        if a.pt.y == b.pt.y {
            a.pt.x.cmp(&b.pt.x)
        } else {
            b.pt.y.cmp(&a.pt.y)
        }
    });
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

fn v_insert_local_minima_into_ael(sc: &mut VattiScratch, bot_y: i64, cliptype: i32) {
    while let Some(lm) = sc.pop_locmin(bot_y) {
        let locmin_idx = sc.locmin_idx - 1;
        let lm_pt = sc.vtx_pool[lm.vertex].pt;
        let lb = sc.new_active();
        sc.act_pool[lb].bot = lm_pt;
        sc.act_pool[lb].curr_x = lm_pt.x;
        sc.act_pool[lb].wind_dx = -1;
        sc.act_pool[lb].vertex_top = sc.vtx_pool[lm.vertex].prev;
        sc.act_pool[lb].top = sc.vtx_pool[sc.act_pool[lb].vertex_top.unwrap()].pt;
        sc.act_pool[lb].local_min = Some(locmin_idx);
        sc.act_pool[lb].dx = v_get_dx(sc.act_pool[lb].bot, sc.act_pool[lb].top);
        let rb = sc.new_active();
        sc.act_pool[rb].bot = lm_pt;
        sc.act_pool[rb].curr_x = lm_pt.x;
        sc.act_pool[rb].wind_dx = 1;
        sc.act_pool[rb].vertex_top = sc.vtx_pool[lm.vertex].next;
        sc.act_pool[rb].top = sc.vtx_pool[sc.act_pool[rb].vertex_top.unwrap()].pt;
        sc.act_pool[rb].local_min = Some(locmin_idx);
        sc.act_pool[rb].dx = v_get_dx(sc.act_pool[rb].bot, sc.act_pool[rb].top);
        let mut lb = lb;
        let mut rb = rb;
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
            sc.act_pool[rb].next_in_sel = sc.sel;
            sc.sel = Some(rb);
        } else {
            v_check_join_right(sc, rb, sc.act_pool[rb].bot, false);
            sc.insert_scanline(sc.act_pool[rb].top.y);
        }
        if v_is_horizontal(sc, lb) {
            sc.act_pool[lb].next_in_sel = sc.sel;
            sc.sel = Some(lb);
        } else {
            sc.insert_scanline(sc.act_pool[lb].top.y);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Maxima and top of scanbeam
// ═══════════════════════════════════════════════════════════════════════════

fn v_do_maxima(sc: &mut VattiScratch, e: usize, cliptype: i32) -> OptIdx {
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
                sc.act_pool[e].next_in_sel = sc.sel;
                sc.sel = Some(e);
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
    loop {
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
    loop {
        let prev = sc.opt_pool[op2].prev;
        let next = sc.opt_pool[op2].next;
        if v_is_collinear(
            sc.opt_pool[prev].pt,
            sc.opt_pool[op2].pt,
            sc.opt_pool[next].pt,
        ) && (sc.opt_pool[op2].pt == sc.opt_pool[prev].pt
            || sc.opt_pool[op2].pt == sc.opt_pool[next].pt
            || v_dot_product(
                sc.opt_pool[prev].pt,
                sc.opt_pool[op2].pt,
                sc.opt_pool[next].pt,
            ) < 0.0)
        {
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

fn v_execute_internal(sc: &mut VattiScratch, cliptype: i32) -> bool {
    sc.locmin_list.sort_by(|a, b| {
        let ay = sc.vtx_pool[a.vertex].pt.y;
        let by = sc.vtx_pool[b.vertex].pt.y;
        if by != ay {
            by.cmp(&ay)
        } else {
            sc.vtx_pool[a.vertex].pt.x.cmp(&sc.vtx_pool[b.vertex].pt.x)
        }
    });
    for i in 0..sc.locmin_list.len() {
        let y = sc.vtx_pool[sc.locmin_list[i].vertex].pt.y;
        sc.insert_scanline(y);
    }
    sc.locmin_idx = 0;
    let mut y = match sc.pop_scanline() {
        Some(y) => y,
        None => return true,
    };
    while sc.succeeded {
        v_insert_local_minima_into_ael(sc, y, cliptype);
        while let Some(e) = sc.sel {
            sc.sel = sc.act_pool[e].next_in_sel;
            v_do_horizontal(sc, e, cliptype);
        }
        if !sc.horz_seg_list.is_empty() {
            v_convert_horz_segs_to_joins(sc);
            sc.horz_seg_list.clear();
        }
        sc.bot_y = y;
        y = match sc.pop_scanline() {
            Some(y) => y,
            None => break,
        };
        if sc.succeeded && v_build_intersect_list(sc, y) {
            v_process_intersect_list(sc, cliptype);
            sc.intersect_nodes.clear();
        }
        v_do_top_of_scanbeam(sc, y, cliptype);
        while let Some(e) = sc.sel {
            sc.sel = sc.act_pool[e].next_in_sel;
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

/// Drops a closing point that repeats the first one.
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

fn v_any_cross(va: &[BIVec2], vb: &[BIVec2]) -> bool {
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
            if v_segs_intersect(a1, a2, b1, b2) {
                return true;
            }
        }
    }
    false
}

fn v_centroid(v: &[BIVec2]) -> BIVec2 {
    let mut c = BIVec2 { x: 0, y: 0 };
    for p in v {
        c.x += p.x;
        c.y += p.y;
    }
    c.x /= v.len() as i64;
    c.y /= v.len() as i64;
    c
}

/// Containment of non-crossing polygons: vertex test, validated by the centroid, then the centroid
/// nudged by one unit when it sits on the boundary.
fn v_contains(va: &[BIVec2], vb: &[BIVec2]) -> (bool, bool) {
    let mut a_in_b = pip_i(va[0], vb);
    let mut b_in_a = pip_i(vb[0], va);
    let ca_cen = v_centroid(va);
    let cb_cen = v_centroid(vb);
    if a_in_b && !pip_i(ca_cen, vb) {
        a_in_b = false;
    }
    if b_in_a && !pip_i(cb_cen, va) {
        b_in_a = false;
    }
    if a_in_b || b_in_a {
        return (a_in_b, b_in_a);
    }
    a_in_b = pip_i(ca_cen, vb);
    b_in_a = pip_i(cb_cen, va);
    if a_in_b || b_in_a {
        return (a_in_b, b_in_a);
    }
    a_in_b = pip_i(
        BIVec2 {
            x: ca_cen.x + 1,
            y: ca_cen.y + 1,
        },
        vb,
    );
    b_in_a = pip_i(
        BIVec2 {
            x: cb_cen.x + 1,
            y: cb_cen.y + 1,
        },
        va,
    );
    (a_in_b, b_in_a)
}

/// First output point of a finished ring, or None when the ring is degenerate.
fn v_ring_start(sc: &mut VattiScratch, outrec: usize) -> OptIdx {
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
    for i in 0..sc.outrec_list.len() {
        let op = match v_ring_start(sc, sc.outrec_list[i]) {
            Some(op) => op,
            None => continue,
        };
        let mut coords = Vec::new();
        let mut o = sc.opt_pool[op].next;
        let mut last = sc.opt_pool[o].pt;
        coords.push(last.x as f64 * inv_scale);
        coords.push(last.y as f64 * inv_scale);
        coords.push(0.0);
        o = sc.opt_pool[o].next;
        while o != sc.opt_pool[op].next {
            if sc.opt_pool[o].pt != last {
                last = sc.opt_pool[o].pt;
                coords.push(last.x as f64 * inv_scale);
                coords.push(last.y as f64 * inv_scale);
                coords.push(0.0);
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

// ═══════════════════════════════════════════════════════════════════════════
// Open subject against closed clip
// ═══════════════════════════════════════════════════════════════════════════

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
// Boolean operations
// ═══════════════════════════════════════════════════════════════════════════

pub struct BooleanPolyline;

impl BooleanPolyline {
    /// Vatti boolean of two closed planar polylines; clip_type 0 intersection, 1 union, 2 a minus b.
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
            if na * nb <= 400 {
                let va = v_cvt_to_i64(ca, na, bool_scale);
                let vb = v_cvt_to_i64(cb, nb, bool_scale);
                let (a_min_x, a_max_x, a_min_y, a_max_y) = v_bounds(&va);
                let (b_min_x, b_max_x, b_min_y, b_max_y) = v_bounds(&vb);
                if a_max_x < b_min_x || b_max_x < a_min_x || a_max_y < b_min_y || b_max_y < a_min_y
                {
                    return v_select(a, b, pip_i(va[0], &vb), pip_i(vb[0], &va), clip_type);
                }
                if !v_any_cross(&va, &vb) {
                    let (a_in_b, b_in_a) = v_contains(&va, &vb);
                    return v_select(a, b, a_in_b, b_in_a, clip_type);
                }
                v_add_path(sc, &va, na, 0);
                v_add_path(sc, &vb, nb, 1);
            } else {
                let (va_head, a_min_x, a_max_x, a_min_y, a_max_y) =
                    v_add_path_from_doubles(sc, ca, na, 0, bool_scale);
                let (vb_head, b_min_x, b_max_x, b_min_y, b_max_y) =
                    v_add_path_from_doubles(sc, cb, nb, 1, bool_scale);
                let (va_head, vb_head) = match (va_head, vb_head) {
                    (Some(va_head), Some(vb_head)) => (va_head, vb_head),
                    _ => return vec![],
                };
                if a_max_x < b_min_x || b_max_x < a_min_x || a_max_y < b_min_y || b_max_y < a_min_y
                {
                    let a_in_b = pip_vertex(sc, sc.vtx_pool[va_head].pt, vb_head);
                    let b_in_a = pip_vertex(sc, sc.vtx_pool[vb_head].pt, va_head);
                    return v_select(a, b, a_in_b, b_in_a, clip_type);
                }
            }
            if !v_execute_internal(sc, clip_type) {
                return vec![];
            }
            v_extract(sc, 1.0 / bool_scale)
        })
    }

    /// Number of output points of compute, without building polylines.
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
            if va_head.is_none() || vb_head.is_none() {
                return 0;
            }
            if a_max_x < b_min_x || b_max_x < a_min_x || a_max_y < b_min_y || b_max_y < a_min_y {
                return 0;
            }
            if !v_execute_internal(sc, clip_type) {
                return 0;
            }
            let mut total = 0;
            for i in 0..sc.outrec_list.len() {
                let op = match v_ring_start(sc, sc.outrec_list[i]) {
                    Some(op) => op,
                    None => continue,
                };
                let mut o = op;
                loop {
                    total += 1;
                    o = sc.opt_pool[o].next;
                    if o == op {
                        break;
                    }
                }
            }
            total
        })
    }

    /// compute on flat xy arrays; writes up to max_out result points to out_xy and returns the total.
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

    /// Pieces of an open polyline that lie inside a closed clip polygon, in the xy plane.
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
