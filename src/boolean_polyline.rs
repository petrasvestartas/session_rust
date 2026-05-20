// Vatti scanline polygon clipping in session_rust types. Closed-path NonZero
// fill only. Uses arena-index pattern: all pointers are usize indices into
// Vec pools.

use crate::Polyline;

const VF_LOCAL_MAX: u32 = 4;
const VF_LOCAL_MIN: u32 = 8;

type OptIdx = Option<usize>;

#[derive(Clone, Copy, Default, PartialEq, Eq)]
struct Pt { x: i64, y: i64 }

#[derive(Clone, Default)]
struct VVertex { pt: Pt, next: OptIdx, prev: OptIdx, flags: u32 }
#[derive(Clone)]
struct VLocalMinima { vertex: usize, polytype: i8 }
#[derive(Clone, Default)]
struct VOutPt { pt: Pt, next: usize, prev: usize, outrec: usize, horz_used: bool }
#[derive(Clone, Default)]
struct VOutRec { idx: usize, front_edge: OptIdx, back_edge: OptIdx, pts: OptIdx, owner: OptIdx }
#[derive(Clone, Default)]
struct VActive {
    bot: Pt, top: Pt, curr_x: i64, dx: f32,
    wind_dx: i32, wind_cnt: i32, wind_cnt2: i32,
    outrec: OptIdx, prev_in_ael: OptIdx, next_in_ael: OptIdx,
    prev_in_sel: OptIdx, next_in_sel: OptIdx, jump: OptIdx,
    vertex_top: OptIdx, local_min: OptIdx,
    is_left_bound: bool, join_with: i8,
}
struct VIntersectNode { pt: Pt, edge1: usize, edge2: usize }
#[derive(Clone)]
struct VHorzSeg { left_op: usize, right_op: OptIdx, left_to_right: bool }
struct VHorzJoin { op1: usize, op2: usize }

struct Sc {
    vtx: Vec<VVertex>, act: Vec<VActive>, opt: Vec<VOutPt>, orc: Vec<VOutRec>,
    locmin: Vec<VLocalMinima>, isnodes: Vec<VIntersectNode>,
    hsegs: Vec<VHorzSeg>, hjoins: Vec<VHorzJoin>,
    orclist: Vec<usize>, scan: Vec<i64>,
    actives: OptIdx, sel: OptIdx, bot_y: i64, lm_idx: usize, ok: bool,
}

impl Sc {
    fn new() -> Self {
        Self { vtx: Vec::new(), act: Vec::new(), opt: Vec::new(), orc: Vec::new(),
               locmin: Vec::new(), isnodes: Vec::new(), hsegs: Vec::new(), hjoins: Vec::new(),
               orclist: Vec::new(), scan: Vec::new(),
               actives: None, sel: None, bot_y: 0, lm_idx: 0, ok: true }
    }
    /// Reset lengths to 0 but preserve capacity, matching C++ `static thread_local VattiScratch vtls`
    /// reset semantics in boolean_polyline.cpp:159. Used by the SCRATCH thread_local pool.
    fn clear_for_reuse(&mut self) {
        self.vtx.clear(); self.act.clear(); self.opt.clear(); self.orc.clear();
        self.locmin.clear(); self.isnodes.clear(); self.hsegs.clear(); self.hjoins.clear();
        self.orclist.clear(); self.scan.clear();
        self.actives = None; self.sel = None; self.bot_y = 0; self.lm_idx = 0; self.ok = true;
    }
    fn alloc_vtx(&mut self) -> usize { self.vtx.push(VVertex::default()); self.vtx.len()-1 }
    fn alloc_act(&mut self) -> usize { self.act.push(VActive::default()); self.act.len()-1 }
    fn alloc_opt(&mut self, pt: Pt, outrec: usize) -> usize {
        let i = self.opt.len(); self.opt.push(VOutPt { pt, next: i, prev: i, outrec, horz_used: false }); i
    }
    fn alloc_orc(&mut self) -> usize {
        let i = self.orc.len();
        self.orc.push(VOutRec { idx: i, ..Default::default() });
        self.orclist.push(i); i
    }
    fn push_scan(&mut self, y: i64) {
        self.scan.push(y);
        let mut i = self.scan.len() - 1;
        while i > 0 { let p = (i-1)/2; if self.scan[p] >= self.scan[i] { break; } self.scan.swap(p, i); i = p; }
    }
    fn pop_scan(&mut self) -> Option<i64> {
        if self.scan.is_empty() { return None; }
        let y = self.scan[0];
        let last = self.scan.len() - 1;
        self.scan.swap(0, last); self.scan.pop();
        let mut i = 0; let n = self.scan.len();
        loop { let (l, r) = (2*i+1, 2*i+2); let mut m = i;
            if l < n && self.scan[l] > self.scan[m] { m = l; }
            if r < n && self.scan[r] > self.scan[m] { m = r; }
            if m == i { break; } self.scan.swap(i, m); i = m; }
        // skip duplicates
        while !self.scan.is_empty() && self.scan[0] == y {
            let last = self.scan.len() - 1;
            self.scan.swap(0, last); self.scan.pop();
            let mut i = 0; let n = self.scan.len();
            loop { let (l, r) = (2*i+1, 2*i+2); let mut m = i;
                if l < n && self.scan[l] > self.scan[m] { m = l; }
                if r < n && self.scan[r] > self.scan[m] { m = r; }
                if m == i { break; } self.scan.swap(i, m); i = m; }
        }
        Some(y)
    }
    fn pop_lm(&mut self, y: i64) -> Option<VLocalMinima> {
        if self.lm_idx >= self.locmin.len() { return None; }
        let vtx_idx = self.locmin[self.lm_idx].vertex;
        if self.vtx[vtx_idx].pt.y != y { return None; }
        let lm = self.locmin[self.lm_idx].clone(); self.lm_idx += 1; Some(lm)
    }
}

// ── Geometry helpers ─────────────────────────────────────────────────────

fn get_dx(p1: Pt, p2: Pt) -> f32 {
    let dy = (p2.y - p1.y) as f32;
    if dy != 0.0 { (p2.x - p1.x) as f32 / dy }
    else if p2.x > p1.x { -f32::MAX } else { f32::MAX }
}

fn top_x(sc: &Sc, e: usize, y: i64) -> i64 {
    let a = &sc.act[e];
    if y == a.top.y || a.top.x == a.bot.x { a.top.x }
    else if y == a.bot.y { a.bot.x }
    else { a.bot.x + (a.dx * (y - a.bot.y) as f32).round() as i64 }
}

fn is_horz(sc: &Sc, e: usize) -> bool { sc.act[e].top.y == sc.act[e].bot.y }
fn is_hot(sc: &Sc, e: usize) -> bool { sc.act[e].outrec.is_some() }
fn is_maxima_v(sc: &Sc, v: usize) -> bool { sc.vtx[v].flags & VF_LOCAL_MAX != 0 }
fn is_maxima(sc: &Sc, e: usize) -> bool { sc.act[e].vertex_top.map_or(false, |v| is_maxima_v(sc, v)) }
fn is_front(sc: &Sc, e: usize) -> bool { sc.act[e].outrec.map_or(false, |o| sc.orc[o].front_edge == Some(e)) }
fn is_joined(sc: &Sc, e: usize) -> bool { sc.act[e].join_with != 0 }
fn polytype(sc: &Sc, e: usize) -> i8 { sc.act[e].local_min.map_or(0, |lm| sc.locmin[lm].polytype) }
fn same_pt(sc: &Sc, e1: usize, e2: usize) -> bool { polytype(sc, e1) == polytype(sc, e2) }

fn next_vertex(sc: &Sc, e: usize) -> OptIdx {
    let a = &sc.act[e]; let vt = a.vertex_top?;
    if a.wind_dx > 0 { sc.vtx[vt].next } else { sc.vtx[vt].prev }
}

fn prev_prev_vertex(sc: &Sc, e: usize) -> OptIdx {
    let vt = sc.act[e].vertex_top?;
    if sc.act[e].wind_dx > 0 { sc.vtx[sc.vtx[vt].prev?].prev } else { sc.vtx[sc.vtx[vt].next?].next }
}

fn cross3(p1: Pt, p2: Pt, p3: Pt) -> f32 {
    (p2.x-p1.x) as f32 * (p3.y-p2.y) as f32 - (p2.y-p1.y) as f32 * (p3.x-p2.x) as f32
}
fn dot3(p1: Pt, p2: Pt, p3: Pt) -> f32 {
    (p2.x-p1.x) as f32 * (p3.x-p2.x) as f32 + (p2.y-p1.y) as f32 * (p3.y-p2.y) as f32
}
fn is_collinear(p1: Pt, s: Pt, p2: Pt) -> bool {
    (s.x-p1.x) as i128 * (p2.y-s.y) as i128 == (s.y-p1.y) as i128 * (p2.x-s.x) as i128
}
fn perpendic_dist_sq(pt: Pt, l1: Pt, l2: Pt) -> f32 {
    let (a, b, c, d) = ((pt.x-l1.x) as f32, (pt.y-l1.y) as f32, (l2.x-l1.x) as f32, (l2.y-l1.y) as f32);
    if c == 0.0 && d == 0.0 { return 0.0; }
    let e = a*d - c*b; (e*e) / (c*c + d*d)
}

fn get_seg_isect(a: Pt, b: Pt, c: Pt, d: Pt) -> Option<Pt> {
    let (dx1, dy1) = ((b.x-a.x) as f32, (b.y-a.y) as f32);
    let (dx2, dy2) = ((d.x-c.x) as f32, (d.y-c.y) as f32);
    let det = dy1*dx2 - dy2*dx1;
    if det == 0.0 { return None; }
    let t = ((a.x-c.x) as f32 * dy2 - (a.y-c.y) as f32 * dx2) / det;
    if t <= 0.0 { Some(a) } else if t >= 1.0 { Some(b) }
    else { Some(Pt { x: a.x + (t*dx1).round() as i64, y: a.y + (t*dy1).round() as i64 }) }
}

fn closest_on_seg(pt: Pt, s1: Pt, s2: Pt) -> Pt {
    if s1 == s2 { return s1; }
    let (dx, dy) = ((s2.x-s1.x) as f32, (s2.y-s1.y) as f32);
    let mut q = ((pt.x-s1.x) as f32 * dx + (pt.y-s1.y) as f32 * dy) / (dx*dx + dy*dy);
    if q < 0.0 { q = 0.0; } else if q > 1.0 { q = 1.0; }
    Pt { x: s1.x + (q*dx).round() as i64, y: s1.y + (q*dy).round() as i64 }
}

fn segs_intersect(a: Pt, b: Pt, c: Pt, d: Pt) -> bool {
    let sign = |v: f32| -> i32 { if v == 0.0 { 0 } else if v > 0.0 { 1 } else { -1 } };
    (sign(cross3(a,c,d)) * sign(cross3(b,c,d)) < 0) && (sign(cross3(c,a,b)) * sign(cross3(d,a,b)) < 0)
}

fn area_outpt(sc: &Sc, start: usize) -> f32 {
    let mut r = 0.0; let mut o = start;
    loop {
        let prev = sc.opt[o].prev; let pp = sc.opt[prev].pt; let cp = sc.opt[o].pt;
        r += (pp.y + cp.y) as f32 * (pp.x - cp.x) as f32;
        o = sc.opt[o].next; if o == start { break; }
    }
    r * 0.5
}

fn area_tri(p1: Pt, p2: Pt, p3: Pt) -> f32 {
    (p3.y+p1.y) as f32 * (p3.x-p1.x) as f32 + (p1.y+p2.y) as f32 * (p1.x-p2.x) as f32 + (p2.y+p3.y) as f32 * (p2.x-p3.x) as f32
}

fn pts_close(a: Pt, b: Pt) -> bool { (a.x-b.x).abs() < 2 && (a.y-b.y).abs() < 2 }

fn very_small_tri(sc: &Sc, op: usize) -> bool {
    let n = sc.opt[op].next; let nn = sc.opt[n].next; let p = sc.opt[op].prev;
    nn == p && (pts_close(sc.opt[p].pt, sc.opt[n].pt) || pts_close(sc.opt[op].pt, sc.opt[n].pt) || pts_close(sc.opt[op].pt, sc.opt[p].pt))
}

fn valid_closed(sc: &Sc, op: usize) -> bool {
    let n = sc.opt[op].next; n != op && n != sc.opt[op].prev && !very_small_tri(sc, op)
}

fn pip_i(pt: Pt, poly: &[Pt]) -> bool {
    let mut w: i32 = 0; let n = poly.len();
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i+1) % n]);
        let cross = (b.x-a.x) as i128 * (pt.y-a.y) as i128 - (b.y-a.y) as i128 * (pt.x-a.x) as i128;
        if a.y <= pt.y { if b.y > pt.y && cross > 0 { w += 1; } }
        else { if b.y <= pt.y && cross < 0 { w -= 1; } }
    }
    w != 0
}

fn pip_vertex(sc: &Sc, pt: Pt, head: usize) -> bool {
    let mut w: i32 = 0; let mut v = head;
    loop {
        let a = sc.vtx[v].pt; let b = sc.vtx[sc.vtx[v].next.unwrap()].pt;
        let cross = (b.x-a.x) as i128 * (pt.y-a.y) as i128 - (b.y-a.y) as i128 * (pt.x-a.x) as i128;
        if a.y <= pt.y { if b.y > pt.y && cross > 0 { w += 1; } }
        else { if b.y <= pt.y && cross < 0 { w -= 1; } }
        v = sc.vtx[v].next.unwrap(); if v == head { break; }
    }
    w != 0
}

// ── Vertex building ──────────────────────────────────────────────────────

fn add_path_from_coords(sc: &mut Sc, ca: &[f32], n: usize, pty: i8, bool_scale: f32) -> (OptIdx, i64, i64, i64, i64) {
    if n < 3 { return (None, 0, 0, 0, 0); }
    let base = sc.vtx.len();
    sc.vtx.resize(base + n, VVertex::default());
    let cvt = |i: usize| -> Pt {
        Pt { x: (ca[i*3] * bool_scale).round() as i64, y: (ca[i*3+1] * bool_scale).round() as i64 }
    };
    let pt0 = cvt(0);
    sc.vtx[base].pt = pt0;
    let (mut min_x, mut max_x, mut min_y, mut max_y) = (pt0.x, pt0.x, pt0.y, pt0.y);
    let mut prev = base; let mut cnt = 1usize;
    for i in 1..n {
        let pt = cvt(i);
        if pt == sc.vtx[prev].pt { continue; }
        let ci = base + cnt;
        sc.vtx[ci].pt = pt; sc.vtx[ci].prev = Some(prev); sc.vtx[prev].next = Some(ci);
        prev = ci; cnt += 1;
        if pt.x < min_x { min_x = pt.x; } else if pt.x > max_x { max_x = pt.x; }
        if pt.y < min_y { min_y = pt.y; } else if pt.y > max_y { max_y = pt.y; }
    }
    if cnt < 3 { sc.vtx.truncate(base); return (None, 0, 0, 0, 0); }
    if sc.vtx[prev].pt == sc.vtx[base].pt { prev = sc.vtx[prev].prev.unwrap(); cnt -= 1; }
    if cnt < 3 { sc.vtx.truncate(base); return (None, 0, 0, 0, 0); }
    sc.vtx.truncate(base + cnt);
    sc.vtx[prev].next = Some(base); sc.vtx[base].prev = Some(prev);
    // local minima detection
    let mut pv = sc.vtx[base].prev.unwrap();
    while pv != base && sc.vtx[pv].pt.y == sc.vtx[base].pt.y { pv = sc.vtx[pv].prev.unwrap(); }
    if pv == base { return (Some(base), min_x, max_x, min_y, max_y); }
    let mut going_up = sc.vtx[pv].pt.y > sc.vtx[base].pt.y;
    let going_up0 = going_up;
    pv = base;
    let mut cv = sc.vtx[base].next.unwrap();
    while cv != base {
        if sc.vtx[cv].pt.y > sc.vtx[pv].pt.y && going_up {
            sc.vtx[pv].flags |= VF_LOCAL_MAX; going_up = false;
        } else if sc.vtx[cv].pt.y < sc.vtx[pv].pt.y && !going_up {
            going_up = true; sc.vtx[pv].flags |= VF_LOCAL_MIN;
            sc.locmin.push(VLocalMinima { vertex: pv, polytype: pty });
        }
        pv = cv; cv = sc.vtx[cv].next.unwrap();
    }
    if going_up != going_up0 {
        if going_up0 { sc.vtx[pv].flags |= VF_LOCAL_MIN; sc.locmin.push(VLocalMinima { vertex: pv, polytype: pty }); }
        else { sc.vtx[pv].flags |= VF_LOCAL_MAX; }
    }
    (Some(base), min_x, max_x, min_y, max_y)
}

// ── AEL operations ───────────────────────────────────────────────────────

fn get_maxima_pair(sc: &Sc, e: usize) -> OptIdx {
    let vt = sc.act[e].vertex_top?;
    let mut e2 = sc.act[e].next_in_ael;
    while let Some(ei) = e2 { if sc.act[ei].vertex_top == Some(vt) { return Some(ei); } e2 = sc.act[ei].next_in_ael; }
    None
}

fn get_curr_y_maxima(sc: &Sc, e: usize) -> OptIdx {
    let mut r = sc.act[e].vertex_top?;
    if sc.act[e].wind_dx > 0 { while sc.vtx[sc.vtx[r].next?].pt.y == sc.vtx[r].pt.y { r = sc.vtx[r].next?; } }
    else { while sc.vtx[sc.vtx[r].prev?].pt.y == sc.vtx[r].pt.y { r = sc.vtx[r].prev?; } }
    if is_maxima_v(sc, r) { Some(r) } else { None }
}

fn get_prev_hot(sc: &Sc, e: usize) -> OptIdx {
    let mut p = sc.act[e].prev_in_ael;
    while let Some(pi) = p { if is_hot(sc, pi) { return Some(pi); } p = sc.act[pi].prev_in_ael; }
    None
}

fn is_valid_ael_order(sc: &Sc, res: usize, new: usize) -> bool {
    if sc.act[new].curr_x != sc.act[res].curr_x { return sc.act[new].curr_x > sc.act[res].curr_x; }
    let d = cross3(sc.act[res].top, sc.act[new].bot, sc.act[new].top);
    if d != 0.0 { return d < 0.0; }
    if !is_maxima(sc, res) && sc.act[res].top.y > sc.act[new].top.y {
        return cross3(sc.act[new].bot, sc.act[res].top, sc.vtx[next_vertex(sc, res).unwrap()].pt) <= 0.0;
    }
    if !is_maxima(sc, new) && sc.act[new].top.y > sc.act[res].top.y {
        return cross3(sc.act[new].bot, sc.act[new].top, sc.vtx[next_vertex(sc, new).unwrap()].pt) >= 0.0;
    }
    let y = sc.act[new].bot.y;
    if sc.act[res].bot.y != y || sc.vtx[sc.locmin[sc.act[res].local_min.unwrap()].vertex].pt.y != y { return sc.act[new].is_left_bound; }
    if sc.act[res].is_left_bound != sc.act[new].is_left_bound { return sc.act[new].is_left_bound; }
    let pp_res = prev_prev_vertex(sc, res);
    if pp_res.is_some() && is_collinear(sc.vtx[pp_res.unwrap()].pt, sc.act[res].bot, sc.act[res].top) { return true; }
    let pp_new = prev_prev_vertex(sc, new);
    if pp_res.is_some() && pp_new.is_some() {
        (cross3(sc.vtx[pp_res.unwrap()].pt, sc.act[new].bot, sc.vtx[pp_new.unwrap()].pt) > 0.0) == sc.act[new].is_left_bound
    } else { sc.act[new].is_left_bound }
}

fn insert_left_edge(sc: &mut Sc, e: usize) {
    if sc.actives.is_none() {
        sc.act[e].prev_in_ael = None; sc.act[e].next_in_ael = None; sc.actives = Some(e);
    } else if !is_valid_ael_order(sc, sc.actives.unwrap(), e) {
        sc.act[e].prev_in_ael = None; sc.act[e].next_in_ael = sc.actives;
        sc.act[sc.actives.unwrap()].prev_in_ael = Some(e); sc.actives = Some(e);
    } else {
        let mut e2 = sc.actives.unwrap();
        while sc.act[e2].next_in_ael.is_some() && is_valid_ael_order(sc, sc.act[e2].next_in_ael.unwrap(), e) { e2 = sc.act[e2].next_in_ael.unwrap(); }
        if sc.act[e2].join_with == 2 { e2 = sc.act[e2].next_in_ael.unwrap(); }
        let next = sc.act[e2].next_in_ael;
        sc.act[e].next_in_ael = next; if let Some(n) = next { sc.act[n].prev_in_ael = Some(e); }
        sc.act[e].prev_in_ael = Some(e2); sc.act[e2].next_in_ael = Some(e);
    }
}

fn insert_right_edge(sc: &mut Sc, e: usize, e2: usize) {
    let next = sc.act[e].next_in_ael;
    sc.act[e2].next_in_ael = next; if let Some(n) = next { sc.act[n].prev_in_ael = Some(e2); }
    sc.act[e2].prev_in_ael = Some(e); sc.act[e].next_in_ael = Some(e2);
}

fn swap_positions_in_ael(sc: &mut Sc, e1: usize, e2: usize) {
    let next = sc.act[e2].next_in_ael; if let Some(n) = next { sc.act[n].prev_in_ael = Some(e1); }
    let prev = sc.act[e1].prev_in_ael; if let Some(p) = prev { sc.act[p].next_in_ael = Some(e2); }
    sc.act[e2].prev_in_ael = prev; sc.act[e2].next_in_ael = Some(e1);
    sc.act[e1].prev_in_ael = Some(e2); sc.act[e1].next_in_ael = next;
    if sc.act[e2].prev_in_ael.is_none() { sc.actives = Some(e2); }
}

fn delete_from_ael(sc: &mut Sc, e: usize) {
    let prev = sc.act[e].prev_in_ael; let next = sc.act[e].next_in_ael;
    if prev.is_none() && next.is_none() && sc.actives != Some(e) { return; }
    if let Some(p) = prev { sc.act[p].next_in_ael = next; } else { sc.actives = next; }
    if let Some(n) = next { sc.act[n].prev_in_ael = prev; }
}

// ── Winding + contribution ───────────────────────────────────────────────

fn set_wind_count(sc: &mut Sc, e: usize) {
    let pt = polytype(sc, e);
    let mut e2 = sc.act[e].prev_in_ael;
    while e2.is_some() && polytype(sc, e2.unwrap()) != pt { e2 = sc.act[e2.unwrap()].prev_in_ael; }
    if e2.is_none() {
        sc.act[e].wind_cnt = sc.act[e].wind_dx;
        e2 = sc.actives;
    } else {
        let ei = e2.unwrap();
        let wc = sc.act[ei].wind_cnt; let wd = sc.act[ei].wind_dx; let ewd = sc.act[e].wind_dx;
        if wc * wd < 0 {
            sc.act[e].wind_cnt = if wc.abs() > 1 { if wd * ewd < 0 { wc } else { wc + ewd } } else { ewd };
        } else {
            sc.act[e].wind_cnt = if wd * ewd < 0 { wc } else { wc + ewd };
        }
        sc.act[e].wind_cnt2 = sc.act[ei].wind_cnt2;
        e2 = sc.act[ei].next_in_ael;
    }
    while e2.is_some() && e2.unwrap() != e {
        let ei = e2.unwrap();
        if polytype(sc, ei) != pt { sc.act[e].wind_cnt2 += sc.act[ei].wind_dx; }
        e2 = sc.act[ei].next_in_ael;
    }
}

fn is_contributing(sc: &Sc, e: usize, ct: i32) -> bool {
    if sc.act[e].wind_cnt.abs() != 1 { return false; }
    let wc2 = sc.act[e].wind_cnt2.abs();
    if ct == 0 { wc2 != 0 } else if ct == 1 { wc2 == 0 }
    else { let r = wc2 == 0; if polytype(sc, e) == 0 { r } else { !r } }
}

// ── Output operations ────────────────────────────────────────────────────

fn set_sides(sc: &mut Sc, or: usize, f: usize, b: usize) { sc.orc[or].front_edge = Some(f); sc.orc[or].back_edge = Some(b); }

fn swap_outrecs(sc: &mut Sc, e1: usize, e2: usize) {
    let (or1, or2) = (sc.act[e1].outrec, sc.act[e2].outrec);
    if or1 == or2 { if let Some(o) = or1 { let fe = sc.orc[o].front_edge; sc.orc[o].front_edge = sc.orc[o].back_edge; sc.orc[o].back_edge = fe; } return; }
    if let Some(o) = or1 { if sc.orc[o].front_edge == Some(e1) { sc.orc[o].front_edge = Some(e2); } else { sc.orc[o].back_edge = Some(e2); } }
    if let Some(o) = or2 { if sc.orc[o].front_edge == Some(e2) { sc.orc[o].front_edge = Some(e1); } else { sc.orc[o].back_edge = Some(e1); } }
    sc.act[e1].outrec = or2; sc.act[e2].outrec = or1;
}

fn add_outpt(sc: &mut Sc, e: usize, pt: Pt) -> usize {
    let or = sc.act[e].outrec.unwrap();
    let to_front = is_front(sc, e);
    let op_front = sc.orc[or].pts.unwrap();
    let op_back = sc.opt[op_front].next;
    if to_front && pt == sc.opt[op_front].pt { return op_front; }
    if !to_front && pt == sc.opt[op_back].pt { return op_back; }
    let nop = sc.alloc_opt(pt, or);
    sc.opt[op_back].prev = nop; sc.opt[nop].prev = op_front; sc.opt[nop].next = op_back; sc.opt[op_front].next = nop;
    if to_front { sc.orc[or].pts = Some(nop); }
    nop
}

fn add_local_min_poly(sc: &mut Sc, e1: usize, e2: usize, pt: Pt, is_new: bool) -> usize {
    let or = sc.alloc_orc();
    sc.act[e1].outrec = Some(or); sc.act[e2].outrec = Some(or);
    let prev_hot = get_prev_hot(sc, e1);
    if let Some(ph) = prev_hot {
        let ascending = sc.act[ph].outrec.map_or(false, |o| sc.orc[o].front_edge == Some(ph));
        if ascending == is_new { set_sides(sc, or, e2, e1); } else { set_sides(sc, or, e1, e2); }
    } else {
        sc.orc[or].owner = None;
        if is_new { set_sides(sc, or, e1, e2); } else { set_sides(sc, or, e2, e1); }
    }
    let op = sc.alloc_opt(pt, or);
    sc.orc[or].pts = Some(op);
    op
}

fn uncouple(sc: &mut Sc, e: usize) {
    if let Some(or) = sc.act[e].outrec {
        if let Some(fe) = sc.orc[or].front_edge { sc.act[fe].outrec = None; }
        if let Some(be) = sc.orc[or].back_edge { sc.act[be].outrec = None; }
        sc.orc[or].front_edge = None; sc.orc[or].back_edge = None;
    }
}

fn join_outrec_paths(sc: &mut Sc, e1: usize, e2: usize) {
    let or1 = sc.act[e1].outrec.unwrap(); let or2 = sc.act[e2].outrec.unwrap();
    let p1_st = sc.orc[or1].pts.unwrap(); let p2_st = sc.orc[or2].pts.unwrap();
    let p1_end = sc.opt[p1_st].next; let p2_end = sc.opt[p2_st].next;
    if is_front(sc, e1) {
        sc.opt[p2_end].prev = p1_st; sc.opt[p1_st].next = p2_end;
        sc.opt[p2_st].next = p1_end; sc.opt[p1_end].prev = p2_st;
        sc.orc[or1].pts = Some(p2_st);
        sc.orc[or1].front_edge = sc.orc[or2].front_edge;
        if let Some(fe) = sc.orc[or1].front_edge { sc.act[fe].outrec = Some(or1); }
    } else {
        sc.opt[p1_end].prev = p2_st; sc.opt[p2_st].next = p1_end;
        sc.opt[p1_st].next = p2_end; sc.opt[p2_end].prev = p1_st;
        sc.orc[or1].back_edge = sc.orc[or2].back_edge;
        if let Some(be) = sc.orc[or1].back_edge { sc.act[be].outrec = Some(or1); }
    }
    sc.orc[or2].front_edge = None; sc.orc[or2].back_edge = None; sc.orc[or2].pts = None;
    sc.orc[or2].owner = Some(or1);
    sc.act[e1].outrec = None; sc.act[e2].outrec = None;
}

fn split(sc: &mut Sc, e: usize, pt: Pt) {
    if sc.act[e].join_with == 2 {
        let next = sc.act[e].next_in_ael.unwrap();
        sc.act[e].join_with = 0; sc.act[next].join_with = 0;
        add_local_min_poly(sc, e, next, pt, true);
    } else {
        let prev = sc.act[e].prev_in_ael.unwrap();
        sc.act[e].join_with = 0; sc.act[prev].join_with = 0;
        add_local_min_poly(sc, prev, e, pt, true);
    }
}

fn add_local_max_poly(sc: &mut Sc, e1: usize, e2: usize, pt: Pt) -> OptIdx {
    if is_joined(sc, e1) { split(sc, e1, pt); }
    if is_joined(sc, e2) { split(sc, e2, pt); }
    if is_front(sc, e1) == is_front(sc, e2) { sc.ok = false; return None; }
    let result = add_outpt(sc, e1, pt);
    if sc.act[e1].outrec == sc.act[e2].outrec {
        let or = sc.act[e1].outrec.unwrap();
        sc.orc[or].pts = Some(result);
        uncouple(sc, e1);
    } else if sc.act[e1].outrec.map_or(0, |o| sc.orc[o].idx) < sc.act[e2].outrec.map_or(0, |o| sc.orc[o].idx) {
        join_outrec_paths(sc, e1, e2);
    } else { join_outrec_paths(sc, e2, e1); }
    Some(result)
}

// ── CheckJoin ────────────────────────────────────────────────────────────

fn check_join_left(sc: &mut Sc, e: usize, pt: Pt, check_cx: bool) {
    let prev = match sc.act[e].prev_in_ael { Some(p) => p, None => return };
    if !is_hot(sc, e) || !is_hot(sc, prev) || is_horz(sc, e) || is_horz(sc, prev) { return; }
    if (pt.y < sc.act[e].top.y + 2 || pt.y < sc.act[prev].top.y + 2) && (sc.act[e].bot.y > pt.y || sc.act[prev].bot.y > pt.y) { return; }
    if check_cx { if perpendic_dist_sq(pt, sc.act[prev].bot, sc.act[prev].top) > 0.25 { return; } }
    else if sc.act[e].curr_x != sc.act[prev].curr_x { return; }
    if !is_collinear(sc.act[e].top, pt, sc.act[prev].top) { return; }
    let or_e = sc.act[e].outrec.map(|o| sc.orc[o].idx).unwrap_or(usize::MAX);
    let or_p = sc.act[prev].outrec.map(|o| sc.orc[o].idx).unwrap_or(usize::MAX);
    if or_e == or_p { add_local_max_poly(sc, prev, e, pt); }
    else if or_e < or_p { join_outrec_paths(sc, e, prev); }
    else { join_outrec_paths(sc, prev, e); }
    sc.act[prev].join_with = 2; sc.act[e].join_with = 1;
}

fn check_join_right(sc: &mut Sc, e: usize, pt: Pt, check_cx: bool) {
    let next = match sc.act[e].next_in_ael { Some(n) => n, None => return };
    if !is_hot(sc, e) || !is_hot(sc, next) || is_horz(sc, e) || is_horz(sc, next) { return; }
    if (pt.y < sc.act[e].top.y + 2 || pt.y < sc.act[next].top.y + 2) && (sc.act[e].bot.y > pt.y || sc.act[next].bot.y > pt.y) { return; }
    if check_cx { if perpendic_dist_sq(pt, sc.act[next].bot, sc.act[next].top) > 0.35 { return; } }
    else if sc.act[e].curr_x != sc.act[next].curr_x { return; }
    if !is_collinear(sc.act[e].top, pt, sc.act[next].top) { return; }
    let or_e = sc.act[e].outrec.map(|o| sc.orc[o].idx).unwrap_or(usize::MAX);
    let or_n = sc.act[next].outrec.map(|o| sc.orc[o].idx).unwrap_or(usize::MAX);
    if or_e == or_n { add_local_max_poly(sc, e, next, pt); }
    else if or_e < or_n { join_outrec_paths(sc, e, next); }
    else { join_outrec_paths(sc, next, e); }
    sc.act[e].join_with = 2; sc.act[next].join_with = 1;
}

// ── IntersectEdges ───────────────────────────────────────────────────────

fn intersect_edges(sc: &mut Sc, e1: usize, e2: usize, pt: Pt, ct: i32) {
    if is_joined(sc, e1) { split(sc, e1, pt); }
    if is_joined(sc, e2) { split(sc, e2, pt); }
    if polytype(sc, e1) == polytype(sc, e2) {
        let (wc, wd, ewd) = (sc.act[e1].wind_cnt, sc.act[e2].wind_dx, sc.act[e1].wind_dx);
        sc.act[e1].wind_cnt = if wc + wd == 0 { -wc } else { wc + wd };
        let (wc2, wd2) = (sc.act[e2].wind_cnt, ewd);
        sc.act[e2].wind_cnt = if wc2 - wd2 == 0 { -wc2 } else { wc2 - wd2 };
    } else {
        sc.act[e1].wind_cnt2 += sc.act[e2].wind_dx;
        sc.act[e2].wind_cnt2 -= sc.act[e1].wind_dx;
    }
    let ow1 = sc.act[e1].wind_cnt.abs(); let ow2 = sc.act[e2].wind_cnt.abs();
    let in01_1 = ow1 == 0 || ow1 == 1; let in01_2 = ow2 == 0 || ow2 == 1;
    if (!is_hot(sc, e1) && !in01_1) || (!is_hot(sc, e2) && !in01_2) { return; }
    if is_hot(sc, e1) && is_hot(sc, e2) {
        if (ow1 != 0 && ow1 != 1) || (ow2 != 0 && ow2 != 1) || polytype(sc, e1) != polytype(sc, e2) {
            add_local_max_poly(sc, e1, e2, pt);
        } else if is_front(sc, e1) || sc.act[e1].outrec == sc.act[e2].outrec {
            add_local_max_poly(sc, e1, e2, pt);
            add_local_min_poly(sc, e1, e2, pt, false);
        } else {
            add_outpt(sc, e1, pt); add_outpt(sc, e2, pt);
            swap_outrecs(sc, e1, e2);
        }
    } else if is_hot(sc, e1) { add_outpt(sc, e1, pt); swap_outrecs(sc, e1, e2); }
    else if is_hot(sc, e2) { add_outpt(sc, e2, pt); swap_outrecs(sc, e1, e2); }
    else {
        let wc2_1 = sc.act[e1].wind_cnt2.abs() as i64; let wc2_2 = sc.act[e2].wind_cnt2.abs() as i64;
        if !same_pt(sc, e1, e2) { add_local_min_poly(sc, e1, e2, pt, false); }
        else if ow1 == 1 && ow2 == 1 {
            match ct {
                0 => { if wc2_1 > 0 && wc2_2 > 0 { add_local_min_poly(sc, e1, e2, pt, false); } }
                1 => { if wc2_1 <= 0 && wc2_2 <= 0 { add_local_min_poly(sc, e1, e2, pt, false); } }
                _ => {
                    if (polytype(sc, e1) == 1 && wc2_1 > 0 && wc2_2 > 0) || (polytype(sc, e1) == 0 && wc2_1 <= 0 && wc2_2 <= 0) {
                        add_local_min_poly(sc, e1, e2, pt, false);
                    }
                }
            }
        }
    }
}

// ── Horizontal + Update edge ─────────────────────────────────────────────

fn update_edge_into_ael(sc: &mut Sc, e: usize, _ct: i32) {
    let nv = next_vertex(sc, e).unwrap();
    sc.act[e].bot = sc.act[e].top; sc.act[e].vertex_top = Some(nv);
    sc.act[e].top = sc.vtx[nv].pt; sc.act[e].curr_x = sc.act[e].bot.x;
    sc.act[e].dx = get_dx(sc.act[e].bot, sc.act[e].top);
    if is_joined(sc, e) { split(sc, e, sc.act[e].bot); }
    if is_horz(sc, e) {
        // trim horz
        let mut pt = sc.vtx[next_vertex(sc, e).unwrap()].pt;
        while pt.y == sc.act[e].top.y {
            if (pt.x < sc.act[e].top.x) != (sc.act[e].bot.x < sc.act[e].top.x) { break; }
            sc.act[e].vertex_top = next_vertex(sc, e);
            sc.act[e].top = pt;
            if is_maxima(sc, e) { break; }
            pt = sc.vtx[next_vertex(sc, e).unwrap()].pt;
        }
        sc.act[e].dx = get_dx(sc.act[e].bot, sc.act[e].top);
        return;
    }
    sc.push_scan(sc.act[e].top.y);
    check_join_left(sc, e, sc.act[e].bot, false);
    check_join_right(sc, e, sc.act[e].bot, true);
}

fn reset_horz_dir(sc: &Sc, e: usize, max_v: OptIdx) -> (bool, i64, i64) {
    if sc.act[e].bot.x == sc.act[e].top.x {
        let mut ae = sc.act[e].next_in_ael;
        while ae.is_some() && sc.act[ae.unwrap()].vertex_top != max_v { ae = sc.act[ae.unwrap()].next_in_ael; }
        (ae.is_some(), sc.act[e].curr_x, sc.act[e].curr_x)
    } else if sc.act[e].curr_x < sc.act[e].top.x { (true, sc.act[e].curr_x, sc.act[e].top.x) }
    else { (false, sc.act[e].top.x, sc.act[e].curr_x) }
}

fn do_horizontal(sc: &mut Sc, horz: usize, ct: i32) {
    let y = sc.act[horz].bot.y;
    let vertex_max = get_curr_y_maxima(sc, horz);
    let (mut is_ltr, mut hl, mut hr) = reset_horz_dir(sc, horz, vertex_max);
    if is_hot(sc, horz) {
        let op = add_outpt(sc, horz, Pt { x: sc.act[horz].curr_x, y });
        sc.hsegs.push(VHorzSeg { left_op: op, right_op: None, left_to_right: true });
    }
    loop {
        let mut ei = if is_ltr { sc.act[horz].next_in_ael } else { sc.act[horz].prev_in_ael };
        while let Some(e) = ei {
            if sc.act[e].vertex_top == vertex_max {
                if is_hot(sc, horz) && is_joined(sc, e) { split(sc, e, sc.act[e].top); }
                if is_hot(sc, horz) {
                    while sc.act[horz].vertex_top != vertex_max {
                        add_outpt(sc, horz, sc.act[horz].top);
                        update_edge_into_ael(sc, horz, ct);
                    }
                    if is_ltr { add_local_max_poly(sc, horz, e, sc.act[horz].top); }
                    else { add_local_max_poly(sc, e, horz, sc.act[horz].top); }
                }
                delete_from_ael(sc, e); delete_from_ael(sc, horz); return;
            }
            if vertex_max != sc.act[horz].vertex_top {
                if (is_ltr && sc.act[e].curr_x > hr) || (!is_ltr && sc.act[e].curr_x < hl) { break; }
                if sc.act[e].curr_x == sc.act[horz].top.x && !is_horz(sc, e) {
                    let pt2 = sc.vtx[next_vertex(sc, horz).unwrap()].pt;
                    if is_ltr { if top_x(sc, e, pt2.y) >= pt2.x { break; } }
                    else { if top_x(sc, e, pt2.y) <= pt2.x { break; } }
                }
            }
            let pt = Pt { x: sc.act[e].curr_x, y: sc.act[horz].bot.y };
            if is_ltr {
                intersect_edges(sc, horz, e, pt, ct); swap_positions_in_ael(sc, horz, e);
                check_join_left(sc, e, pt, false);
                sc.act[horz].curr_x = sc.act[e].curr_x; ei = sc.act[horz].next_in_ael;
            } else {
                intersect_edges(sc, e, horz, pt, ct); swap_positions_in_ael(sc, e, horz);
                check_join_right(sc, e, pt, false);
                sc.act[horz].curr_x = sc.act[e].curr_x; ei = sc.act[horz].prev_in_ael;
            }
            if sc.act[horz].outrec.is_some() {
                let last_op = { let or = sc.act[horz].outrec.unwrap(); let pts = sc.orc[or].pts.unwrap();
                    if sc.orc[or].front_edge == Some(horz) { pts } else { sc.opt[pts].next } };
                sc.hsegs.push(VHorzSeg { left_op: last_op, right_op: None, left_to_right: true });
            }
        }
        let nv = next_vertex(sc, horz);
        if nv.is_none() || sc.vtx[nv.unwrap()].pt.y != sc.act[horz].top.y { break; }
        if is_hot(sc, horz) { add_outpt(sc, horz, sc.act[horz].top); }
        update_edge_into_ael(sc, horz, ct);
        let r = reset_horz_dir(sc, horz, vertex_max);
        is_ltr = r.0; hl = r.1; hr = r.2;
    }
    if is_hot(sc, horz) {
        let op = add_outpt(sc, horz, sc.act[horz].top);
        sc.hsegs.push(VHorzSeg { left_op: op, right_op: None, left_to_right: true });
    }
    update_edge_into_ael(sc, horz, ct);
}

// ── HorzSeg/HorzJoin ────────────────────────────────────────────────────

fn dup_outpt(sc: &mut Sc, op: usize, after: bool) -> usize {
    let r = sc.alloc_opt(sc.opt[op].pt, sc.opt[op].outrec);
    if after {
        let next = sc.opt[op].next; sc.opt[r].next = next; sc.opt[next].prev = r;
        sc.opt[r].prev = op; sc.opt[op].next = r;
    } else {
        let prev = sc.opt[op].prev; sc.opt[r].prev = prev; sc.opt[prev].next = r;
        sc.opt[r].next = op; sc.opt[op].prev = r;
    }
    r
}

fn convert_horz_segs_to_joins(sc: &mut Sc) {
    let n = sc.hsegs.len();
    for i in 0..n {
        let op = sc.hsegs[i].left_op;
        let mut or = sc.opt[op].outrec;
        while sc.orc[or].pts.is_none() { or = sc.orc[or].owner.unwrap_or(or); if sc.orc[or].pts.is_some() || sc.orc[or].owner.is_none() { break; } }
        if sc.orc[or].pts.is_none() { sc.hsegs[i].right_op = None; continue; }
        let has_edges = sc.orc[or].front_edge.is_some();
        let cy = sc.opt[op].pt.y;
        let mut op_p = op; let mut op_n = op;
        if has_edges {
            let op_a = sc.orc[or].pts.unwrap(); let op_z = sc.opt[op_a].next;
            while op_p != op_z && sc.opt[sc.opt[op_p].prev].pt.y == cy { op_p = sc.opt[op_p].prev; }
            while op_n != op_a && sc.opt[sc.opt[op_n].next].pt.y == cy { op_n = sc.opt[op_n].next; }
        } else {
            while sc.opt[op_p].prev != op_n && sc.opt[sc.opt[op_p].prev].pt.y == cy { op_p = sc.opt[op_p].prev; }
            while sc.opt[op_n].next != op_p && sc.opt[sc.opt[op_n].next].pt.y == cy { op_n = sc.opt[op_n].next; }
        }
        if sc.opt[op_p].pt.x == sc.opt[op_n].pt.x { sc.hsegs[i].right_op = None; continue; }
        if sc.opt[op_p].pt.x < sc.opt[op_n].pt.x {
            sc.hsegs[i].left_op = op_p; sc.hsegs[i].right_op = Some(op_n); sc.hsegs[i].left_to_right = true;
        } else {
            sc.hsegs[i].left_op = op_n; sc.hsegs[i].right_op = Some(op_p); sc.hsegs[i].left_to_right = false;
        }
        if sc.opt[sc.hsegs[i].left_op].horz_used { sc.hsegs[i].right_op = None; continue; }
        sc.opt[sc.hsegs[i].left_op].horz_used = true;
    }
    let valid: usize = sc.hsegs.iter().filter(|h| h.right_op.is_some()).count();
    if valid < 2 { return; }
    sc.hsegs.sort_by(|a, b| {
        if a.right_op.is_none() || b.right_op.is_none() { return a.right_op.is_some().cmp(&b.right_op.is_some()).reverse(); }
        sc.opt[b.left_op].pt.x.cmp(&sc.opt[a.left_op].pt.x).reverse()
    });
    let j = valid;
    for i in 0..j.saturating_sub(1) {
        for k in (i+1)..j {
            let hs1_lo = sc.hsegs[i].left_op; let hs1_ro = sc.hsegs[i].right_op.unwrap();
            let hs2_lo = sc.hsegs[k].left_op; let hs2_ro = sc.hsegs[k].right_op.unwrap();
            if sc.opt[hs2_lo].pt.x >= sc.opt[hs1_ro].pt.x || sc.hsegs[k].left_to_right == sc.hsegs[i].left_to_right || sc.opt[hs2_ro].pt.x <= sc.opt[hs1_lo].pt.x { continue; }
            let cy = sc.opt[hs1_lo].pt.y;
            let mut lo1 = sc.hsegs[i].left_op; let mut lo2 = sc.hsegs[k].left_op;
            if sc.hsegs[i].left_to_right {
                while sc.opt[sc.opt[lo1].next].pt.y == cy && sc.opt[sc.opt[lo1].next].pt.x <= sc.opt[lo2].pt.x { lo1 = sc.opt[lo1].next; }
                while sc.opt[sc.opt[lo2].prev].pt.y == cy && sc.opt[sc.opt[lo2].prev].pt.x <= sc.opt[lo1].pt.x { lo2 = sc.opt[lo2].prev; }
                let d1 = dup_outpt(sc, lo1, true); let d2 = dup_outpt(sc, lo2, false);
                sc.hjoins.push(VHorzJoin { op1: d1, op2: d2 });
            } else {
                while sc.opt[sc.opt[lo1].prev].pt.y == cy && sc.opt[sc.opt[lo1].prev].pt.x <= sc.opt[lo2].pt.x { lo1 = sc.opt[lo1].prev; }
                while sc.opt[sc.opt[lo2].next].pt.y == cy && sc.opt[sc.opt[lo2].next].pt.x <= sc.opt[lo1].pt.x { lo2 = sc.opt[lo2].next; }
                let d1 = dup_outpt(sc, lo2, true); let d2 = dup_outpt(sc, lo1, false);
                sc.hjoins.push(VHorzJoin { op1: d1, op2: d2 });
            }
            sc.hsegs[i].left_op = lo1; sc.hsegs[k].left_op = lo2;
        }
    }
}

fn fix_outrec_pts(sc: &mut Sc, or: usize) {
    let start = sc.orc[or].pts.unwrap(); let mut o = start;
    loop { sc.opt[o].outrec = or; o = sc.opt[o].next; if o == start { break; } }
}

fn process_horz_joins(sc: &mut Sc) {
    for ji in 0..sc.hjoins.len() {
        let op1 = sc.hjoins[ji].op1; let op2 = sc.hjoins[ji].op2;
        let mut or1 = sc.opt[op1].outrec; while sc.orc[or1].pts.is_none() { or1 = sc.orc[or1].owner.unwrap_or(or1); }
        let mut or2 = sc.opt[op2].outrec; while sc.orc[or2].pts.is_none() { or2 = sc.orc[or2].owner.unwrap_or(or2); }
        let op1b = sc.opt[op1].next; let op2b = sc.opt[op2].prev;
        sc.opt[op1].next = op2; sc.opt[op2].prev = op1;
        sc.opt[op1b].prev = op2b; sc.opt[op2b].next = op1b;
        if or1 == or2 {
            let nr = sc.alloc_orc(); sc.orc[nr].pts = Some(op1b); fix_outrec_pts(sc, nr);
            if sc.opt[sc.orc[or1].pts.unwrap()].outrec == nr { sc.orc[or1].pts = Some(op1); sc.opt[op1].outrec = or1; }
            sc.orc[nr].owner = Some(or1);
        } else { sc.orc[or2].pts = None; sc.orc[or2].owner = Some(or1); }
    }
}

// ── Intersection detection ───────────────────────────────────────────────

fn add_new_isect_node(sc: &mut Sc, e1: usize, e2: usize, top_y: i64) {
    let ip = match get_seg_isect(sc.act[e1].bot, sc.act[e1].top, sc.act[e2].bot, sc.act[e2].top) {
        Some(p) => p, None => Pt { x: sc.act[e1].curr_x, y: top_y }
    };
    let mut ip = ip;
    if ip.y > sc.bot_y || ip.y < top_y {
        let (ad1, ad2) = (sc.act[e1].dx.abs(), sc.act[e2].dx.abs());
        if ad1 > 100.0 && ad2 > 100.0 {
            ip = if ad1 > ad2 { closest_on_seg(ip, sc.act[e1].bot, sc.act[e1].top) } else { closest_on_seg(ip, sc.act[e2].bot, sc.act[e2].top) };
        } else if ad1 > 100.0 { ip = closest_on_seg(ip, sc.act[e1].bot, sc.act[e1].top); }
        else if ad2 > 100.0 { ip = closest_on_seg(ip, sc.act[e2].bot, sc.act[e2].top); }
        else { if ip.y < top_y { ip.y = top_y; } else { ip.y = sc.bot_y; }
            ip.x = if ad1 < ad2 { top_x(sc, e1, ip.y) } else { top_x(sc, e2, ip.y) }; }
    }
    sc.isnodes.push(VIntersectNode { pt: ip, edge1: e1, edge2: e2 });
}

fn build_intersect_list(sc: &mut Sc, top_y: i64) -> bool {
    if sc.actives.is_none() { return false; }
    let first = sc.actives.unwrap();
    if sc.act[first].next_in_ael.is_none() { return false; }
    // copy AEL to SEL with jump pointers
    let mut e_opt = sc.actives;
    sc.sel = e_opt;
    while let Some(e) = e_opt {
        sc.act[e].prev_in_sel = sc.act[e].prev_in_ael;
        sc.act[e].next_in_sel = sc.act[e].next_in_ael;
        sc.act[e].jump = sc.act[e].next_in_sel;
        if sc.act[e].join_with == 1 { sc.act[e].curr_x = sc.act[sc.act[e].prev_in_ael.unwrap()].curr_x; }
        else { sc.act[e].curr_x = top_x(sc, e, top_y); }
        e_opt = sc.act[e].next_in_ael;
    }
    // merge sort
    let mut left_opt = sc.sel;
    while left_opt.is_some() && sc.act[left_opt.unwrap()].jump.is_some() {
        let mut prev_base: OptIdx = None;
        while left_opt.is_some() && sc.act[left_opt.unwrap()].jump.is_some() {
            let left_start = left_opt.unwrap();
            let mut curr_base = left_start;
            let right_start = sc.act[left_start].jump.unwrap();
            let mut l_end: OptIdx = Some(right_start);
            let r_end = sc.act[right_start].jump;
            sc.act[left_start].jump = r_end;
            let mut left = left_opt;
            let mut right = Some(right_start);
            while left != l_end && right != r_end {
                if sc.act[right.unwrap()].curr_x < sc.act[left.unwrap()].curr_x {
                    let ri = right.unwrap();
                    let mut tmp = sc.act[ri].prev_in_sel;
                    loop {
                        let ti = tmp.unwrap();
                        add_new_isect_node(sc, ti, ri, top_y);
                        if tmp == left { break; }
                        tmp = sc.act[ti].prev_in_sel;
                    }
                    // extract ri from sel
                    let ri_next = sc.act[ri].next_in_sel;
                    if let Some(rn) = ri_next { sc.act[rn].prev_in_sel = sc.act[ri].prev_in_sel; }
                    let ri_prev = sc.act[ri].prev_in_sel.unwrap();
                    sc.act[ri_prev].next_in_sel = ri_next;
                    right = ri_next;
                    l_end = right;
                    // insert ri before left
                    let li = left.unwrap();
                    sc.act[ri].prev_in_sel = sc.act[li].prev_in_sel;
                    if let Some(p) = sc.act[ri].prev_in_sel { sc.act[p].next_in_sel = Some(ri); }
                    sc.act[ri].next_in_sel = Some(li); sc.act[li].prev_in_sel = Some(ri);
                    if left == Some(curr_base) {
                        curr_base = ri; sc.act[curr_base].jump = r_end;
                        if prev_base.is_none() { sc.sel = Some(curr_base); }
                        else { sc.act[prev_base.unwrap()].jump = Some(curr_base); }
                    }
                } else { left = sc.act[left.unwrap()].next_in_sel; }
            }
            prev_base = Some(curr_base); left_opt = r_end;
        }
        left_opt = sc.sel;
    }
    !sc.isnodes.is_empty()
}

fn process_intersect_list(sc: &mut Sc, ct: i32) {
    sc.isnodes.sort_by(|a, b| {
        if a.pt.y == b.pt.y { a.pt.x.cmp(&b.pt.x) } else { b.pt.y.cmp(&a.pt.y) }
    });
    let n = sc.isnodes.len();
    for i in 0..n {
        let e1 = sc.isnodes[i].edge1; let e2 = sc.isnodes[i].edge2;
        let adj = sc.act[e1].next_in_ael == Some(e2) || sc.act[e1].prev_in_ael == Some(e2);
        if !adj {
            for j in (i+1)..n {
                let je1 = sc.isnodes[j].edge1; let je2 = sc.isnodes[j].edge2;
                if sc.act[je1].next_in_ael == Some(je2) || sc.act[je1].prev_in_ael == Some(je2) {
                    sc.isnodes.swap(i, j); break;
                }
            }
        }
        let pt = sc.isnodes[i].pt; let e1 = sc.isnodes[i].edge1; let e2 = sc.isnodes[i].edge2;
        intersect_edges(sc, e1, e2, pt, ct); swap_positions_in_ael(sc, e1, e2);
        sc.act[e1].curr_x = pt.x; sc.act[e2].curr_x = pt.x;
        check_join_left(sc, e2, pt, true); check_join_right(sc, e1, pt, true);
    }
}

// ── Insert local minima ──────────────────────────────────────────────────

fn insert_local_minima_into_ael(sc: &mut Sc, bot_y: i64, ct: i32) {
    while let Some(lm) = sc.pop_lm(bot_y) {
        let lm_idx = sc.lm_idx - 1;
        let lm_pt = sc.vtx[lm.vertex].pt;
        let lb = sc.alloc_act();
        sc.act[lb].bot = lm_pt; sc.act[lb].curr_x = lm_pt.x; sc.act[lb].wind_dx = -1;
        sc.act[lb].vertex_top = sc.vtx[lm.vertex].prev; sc.act[lb].top = sc.vtx[sc.act[lb].vertex_top.unwrap()].pt;
        sc.act[lb].local_min = Some(lm_idx); sc.act[lb].dx = get_dx(sc.act[lb].bot, sc.act[lb].top);
        let rb = sc.alloc_act();
        sc.act[rb].bot = lm_pt; sc.act[rb].curr_x = lm_pt.x; sc.act[rb].wind_dx = 1;
        sc.act[rb].vertex_top = sc.vtx[lm.vertex].next; sc.act[rb].top = sc.vtx[sc.act[rb].vertex_top.unwrap()].pt;
        sc.act[rb].local_min = Some(lm_idx); sc.act[rb].dx = get_dx(sc.act[rb].bot, sc.act[rb].top);
        // swap if needed
        let mut lb = lb; let mut rb = rb;
        if is_horz(sc, lb) { if sc.act[lb].dx == -f32::MAX { std::mem::swap(&mut lb, &mut rb); } }
        else if is_horz(sc, rb) { if sc.act[rb].dx == f32::MAX { std::mem::swap(&mut lb, &mut rb); } }
        else if sc.act[lb].dx < sc.act[rb].dx { std::mem::swap(&mut lb, &mut rb); }
        sc.act[lb].is_left_bound = true;
        insert_left_edge(sc, lb);
        set_wind_count(sc, lb);
        let contributing = is_contributing(sc, lb, ct);
        sc.act[rb].is_left_bound = false;
        sc.act[rb].wind_cnt = sc.act[lb].wind_cnt; sc.act[rb].wind_cnt2 = sc.act[lb].wind_cnt2;
        insert_right_edge(sc, lb, rb);
        if contributing {
            add_local_min_poly(sc, lb, rb, sc.act[lb].bot, true);
            if !is_horz(sc, lb) { check_join_left(sc, lb, sc.act[lb].bot, false); }
        }
        while sc.act[rb].next_in_ael.is_some() && is_valid_ael_order(sc, sc.act[rb].next_in_ael.unwrap(), rb) {
            let next = sc.act[rb].next_in_ael.unwrap();
            intersect_edges(sc, rb, next, sc.act[rb].bot, ct);
            swap_positions_in_ael(sc, rb, next);
        }
        if is_horz(sc, rb) { sc.act[rb].next_in_sel = sc.sel; sc.sel = Some(rb); }
        else { check_join_right(sc, rb, sc.act[rb].bot, false); sc.push_scan(sc.act[rb].top.y); }
        if is_horz(sc, lb) { sc.act[lb].next_in_sel = sc.sel; sc.sel = Some(lb); }
        else { sc.push_scan(sc.act[lb].top.y); }
    }
}

// ── DoMaxima + DoTopOfScanbeam ───────────────────────────────────────────

fn do_maxima(sc: &mut Sc, e: usize, ct: i32) -> OptIdx {
    let prev_e = sc.act[e].prev_in_ael;
    let max_pair = match get_maxima_pair(sc, e) { Some(mp) => mp, None => return sc.act[e].next_in_ael };
    if is_joined(sc, e) { split(sc, e, sc.act[e].top); }
    if is_joined(sc, max_pair) { split(sc, max_pair, sc.act[max_pair].top); }
    let mut next_e = sc.act[e].next_in_ael;
    while next_e != Some(max_pair) {
        let ne = next_e.unwrap();
        intersect_edges(sc, e, ne, sc.act[e].top, ct);
        swap_positions_in_ael(sc, e, ne);
        next_e = sc.act[e].next_in_ael;
    }
    if is_hot(sc, e) { add_local_max_poly(sc, e, max_pair, sc.act[e].top); }
    delete_from_ael(sc, max_pair); delete_from_ael(sc, e);
    if let Some(p) = prev_e { sc.act[p].next_in_ael } else { sc.actives }
}

fn do_top_of_scanbeam(sc: &mut Sc, y: i64, ct: i32) {
    sc.sel = None;
    let mut e_opt = sc.actives;
    while let Some(e) = e_opt {
        if sc.act[e].top.y == y {
            sc.act[e].curr_x = sc.act[e].top.x;
            if is_maxima(sc, e) { e_opt = do_maxima(sc, e, ct); continue; }
            if is_hot(sc, e) { add_outpt(sc, e, sc.act[e].top); }
            update_edge_into_ael(sc, e, ct);
            if is_horz(sc, e) { sc.act[e].next_in_sel = sc.sel; sc.sel = Some(e); }
        } else { sc.act[e].curr_x = top_x(sc, e, y); }
        e_opt = sc.act[e].next_in_ael;
    }
}

// ── CleanCollinear + FixSelfIntersects ───────────────────────────────────

fn dispose_outpt(sc: &mut Sc, op: usize) -> usize {
    let r = sc.opt[op].next; let prev = sc.opt[op].prev;
    sc.opt[prev].next = r; sc.opt[r].prev = prev; r
}

fn do_split_op(sc: &mut Sc, or: usize, split_op: usize) {
    let prev_op = sc.opt[split_op].prev; let nn_op = sc.opt[sc.opt[split_op].next].next;
    sc.orc[or].pts = Some(prev_op);
    let ip = match get_seg_isect(sc.opt[prev_op].pt, sc.opt[split_op].pt, sc.opt[sc.opt[split_op].next].pt, sc.opt[nn_op].pt) {
        Some(p) => p, None => return
    };
    let area1 = area_outpt(sc, sc.orc[or].pts.unwrap());
    if area1.abs() < 2.0 { sc.orc[or].pts = None; return; }
    let area2 = area_tri(ip, sc.opt[split_op].pt, sc.opt[sc.opt[split_op].next].pt);
    if ip == sc.opt[prev_op].pt || ip == sc.opt[nn_op].pt {
        sc.opt[nn_op].prev = prev_op; sc.opt[prev_op].next = nn_op;
    } else {
        let nop = sc.alloc_opt(ip, sc.opt[prev_op].outrec);
        sc.opt[nop].prev = prev_op; sc.opt[nop].next = nn_op; sc.opt[nn_op].prev = nop; sc.opt[prev_op].next = nop;
    }
    if area2.abs() >= 1.0 && (area2.abs() > area1.abs() || (area2 > 0.0) == (area1 > 0.0)) {
        let or_owner = sc.orc[or].owner;
        let nr = sc.alloc_orc(); sc.orc[nr].owner = or_owner;
        let split_next = sc.opt[split_op].next;
        sc.opt[split_op].outrec = nr; sc.opt[split_next].outrec = nr;
        let nop = sc.alloc_opt(ip, nr);
        sc.opt[nop].prev = split_next; sc.opt[nop].next = split_op;
        sc.orc[nr].pts = Some(nop);
        sc.opt[split_op].prev = nop; sc.opt[split_next].next = nop;
    }
}

fn fix_self_intersects(sc: &mut Sc, or: usize) {
    let mut op2 = match sc.orc[or].pts { Some(p) => p, None => return };
    loop {
        if sc.opt[op2].prev == sc.opt[sc.opt[op2].next].next { break; }
        if segs_intersect(sc.opt[sc.opt[op2].prev].pt, sc.opt[op2].pt, sc.opt[sc.opt[op2].next].pt, sc.opt[sc.opt[sc.opt[op2].next].next].pt) {
            let pts = sc.orc[or].pts.unwrap();
            if op2 == pts || sc.opt[op2].next == pts { sc.orc[or].pts = Some(sc.opt[pts].prev); }
            do_split_op(sc, or, op2);
            if sc.orc[or].pts.is_none() { break; }
            op2 = sc.orc[or].pts.unwrap(); continue;
        }
        op2 = sc.opt[op2].next;
        if op2 == sc.orc[or].pts.unwrap_or(usize::MAX) { break; }
    }
}

fn clean_collinear(sc: &mut Sc, or_idx: usize) {
    let mut or = or_idx;
    while sc.orc[or].pts.is_none() { match sc.orc[or].owner { Some(o) => or = o, None => return }; }
    if !valid_closed(sc, sc.orc[or].pts.unwrap()) { sc.orc[or].pts = None; return; }
    let mut start_op = sc.orc[or].pts.unwrap();
    let mut op2 = start_op;
    loop {
        let prev = sc.opt[op2].prev; let next = sc.opt[op2].next;
        if is_collinear(sc.opt[prev].pt, sc.opt[op2].pt, sc.opt[next].pt) &&
           (sc.opt[op2].pt == sc.opt[prev].pt || sc.opt[op2].pt == sc.opt[next].pt || dot3(sc.opt[prev].pt, sc.opt[op2].pt, sc.opt[next].pt) < 0.0) {
            if op2 == sc.orc[or].pts.unwrap() { sc.orc[or].pts = Some(prev); }
            op2 = dispose_outpt(sc, op2);
            if !valid_closed(sc, op2) { sc.orc[or].pts = None; return; }
            start_op = op2; continue;
        }
        op2 = sc.opt[op2].next;
        if op2 == start_op { break; }
    }
    fix_self_intersects(sc, or);
}

// ── ExecuteInternal ──────────────────────────────────────────────────────

fn execute_internal(sc: &mut Sc, ct: i32) -> bool {
    sc.locmin.sort_by(|a, b| {
        let ay = sc.vtx[a.vertex].pt.y; let by = sc.vtx[b.vertex].pt.y;
        if by != ay { by.cmp(&ay) } else { sc.vtx[b.vertex].pt.x.cmp(&sc.vtx[a.vertex].pt.x).reverse() }
    });
    for lm in &sc.locmin { sc.scan.push(sc.vtx[lm.vertex].pt.y); }
    // build heap
    let n = sc.scan.len();
    for i in (0..n/2).rev() {
        let mut k = i;
        loop { let (l, r) = (2*k+1, 2*k+2); let mut m = k;
            if l < n && sc.scan[l] > sc.scan[m] { m = l; } if r < n && sc.scan[r] > sc.scan[m] { m = r; }
            if m == k { break; } sc.scan.swap(k, m); k = m; }
    }
    sc.lm_idx = 0;
    let mut y = match sc.pop_scan() { Some(y) => y, None => return true };
    while sc.ok {
        insert_local_minima_into_ael(sc, y, ct);
        // pop horz
        while sc.sel.is_some() { let e = sc.sel.unwrap(); sc.sel = sc.act[e].next_in_sel; do_horizontal(sc, e, ct); }
        if !sc.hsegs.is_empty() { convert_horz_segs_to_joins(sc); sc.hsegs.clear(); }
        sc.bot_y = y;
        y = match sc.pop_scan() { Some(y) => y, None => break };
        if sc.ok && build_intersect_list(sc, y) { process_intersect_list(sc, ct); sc.isnodes.clear(); }
        do_top_of_scanbeam(sc, y, ct);
        while sc.sel.is_some() { let e = sc.sel.unwrap(); sc.sel = sc.act[e].next_in_sel; do_horizontal(sc, e, ct); }
    }
    if sc.ok { process_horz_joins(sc); }
    sc.ok
}

// ── Convex intersection fast path (Sutherland-Hodgman) ──────────────────
//
// For convex CCW inputs (which all ElementPlate faces are — top, bottom, and
// every side face is a convex quad), the full Vatti scanline is enormous
// overkill. Sutherland-Hodgman runs in O(na*nb) with ~6 flops per vertex,
// no fixed-point conversion, no priority queue, no allocator churn.
//
// For na=nb=4 (quad-vs-quad), the Vatti path costs ~50µs per call;
// Sutherland-Hodgman is ~1µs. With 3081 joints in main_1, this saves ~150ms.

/// Returns +1 if convex CCW, -1 if convex CW, 0 if non-convex or degenerate.
/// Reads x,y from stride-3 coords; ignores z.
fn convex_orientation_2d(coords: &[f32], n: usize) -> i32 {
    if n < 3 { return 0; }
    let mut sign: i32 = 0;
    for i in 0..n {
        let i0 = i * 3;
        let i1 = ((i + 1) % n) * 3;
        let i2 = ((i + 2) % n) * 3;
        let dx1 = coords[i1] - coords[i0];
        let dy1 = coords[i1 + 1] - coords[i0 + 1];
        let dx2 = coords[i2] - coords[i1];
        let dy2 = coords[i2 + 1] - coords[i1 + 1];
        let cross = dx1 * dy2 - dy1 * dx2;
        if cross > 1e-12 {
            if sign < 0 { return 0; }
            sign = 1;
        } else if cross < -1e-12 {
            if sign > 0 { return 0; }
            sign = -1;
        }
        // Collinear vertices (cross ≈ 0) are valid in a convex polygon.
    }
    sign
}

/// Sutherland-Hodgman intersection of two convex CCW polygons in 2D.
/// Inputs are stride-3 coords (z=0 ignored). Returns a single open Polyline
/// (stride-3, no closing duplicate — matches Vatti output convention), or an
/// empty Vec if the intersection is degenerate (< 3 vertices).
///
/// Callers must pass thread-local scratch buffers to avoid per-call allocation.
fn sutherland_hodgman_ccw_intersect(
    buf_in: &mut Vec<(f32, f32)>,
    buf_out: &mut Vec<(f32, f32)>,
    ca: &[f32], na: usize,
    cb: &[f32], nb: usize,
) -> Vec<Polyline> {
    buf_in.clear();
    buf_out.clear();
    for i in 0..na {
        buf_in.push((ca[i * 3], ca[i * 3 + 1]));
    }

    // For each edge of clip polygon B (CCW), clip the subject by that edge's
    // half-plane. CCW polygon → "inside" is on the left of each directed edge,
    // i.e. cross = ex*(p.y - es.y) - ey*(p.x - es.x) > 0.
    let mut bx_prev = cb[(nb - 1) * 3];
    let mut by_prev = cb[(nb - 1) * 3 + 1];
    for j in 0..nb {
        let bx_curr = cb[j * 3];
        let by_curr = cb[j * 3 + 1];
        let ex = bx_curr - bx_prev;
        let ey = by_curr - by_prev;

        if buf_in.is_empty() {
            bx_prev = bx_curr;
            by_prev = by_curr;
            continue;
        }

        buf_out.clear();
        let n_in = buf_in.len();
        let mut prev = buf_in[n_in - 1];
        let mut prev_cross = ex * (prev.1 - by_prev) - ey * (prev.0 - bx_prev);
        for k in 0..n_in {
            let curr = buf_in[k];
            let curr_cross = ex * (curr.1 - by_prev) - ey * (curr.0 - bx_prev);
            let prev_inside = prev_cross >= 0.0;
            let curr_inside = curr_cross >= 0.0;
            if curr_inside {
                if !prev_inside {
                    let t = prev_cross / (prev_cross - curr_cross);
                    buf_out.push((
                        prev.0 + t * (curr.0 - prev.0),
                        prev.1 + t * (curr.1 - prev.1),
                    ));
                }
                buf_out.push(curr);
            } else if prev_inside {
                let t = prev_cross / (prev_cross - curr_cross);
                buf_out.push((
                    prev.0 + t * (curr.0 - prev.0),
                    prev.1 + t * (curr.1 - prev.1),
                ));
            }
            prev = curr;
            prev_cross = curr_cross;
        }

        // Swap the two Vec headers (ptr/len/capacity) in place — this makes
        // `buf_out` the new subject polygon for the next iteration while
        // `buf_in` becomes the scratch for the next output. Both stay owned
        // by the caller's thread-local cell; capacities persist.
        std::mem::swap::<Vec<(f32, f32)>>(buf_in, buf_out);
        bx_prev = bx_curr;
        by_prev = by_curr;
    }

    if buf_in.len() < 3 {
        return vec![];
    }

    // Emit an OPEN polyline (stride-3, z=0). The Vatti output convention is
    // unique vertices with no closing duplicate; callers (e.g. face_to_face at
    // intersection.rs:1826) explicitly call .closed() if they need a ring.
    let n_out = buf_in.len();
    let mut coords = Vec::with_capacity(n_out * 3);
    for &(x, y) in buf_in.iter() {
        coords.push(x);
        coords.push(y);
        coords.push(0.0);
    }
    vec![Polyline::from_coords(coords)]
}

// ── Public API ───────────────────────────────────────────────────────────

thread_local! {
    /// Reusable Vatti scratch pool — matches C++ `static thread_local VattiScratch vtls`
    /// at boolean_polyline.cpp:159. Reusing the same Vec capacities across the ~3000+
    /// boolean_op calls in `compute_face_to_face` eliminates per-call allocation churn.
    static SCRATCH: std::cell::RefCell<Sc> = std::cell::RefCell::new(Sc::new());

    /// Reusable ping-pong buffers for the Sutherland-Hodgman convex fast path.
    /// Held as a tuple so a single `borrow_mut()` hands out both halves without
    /// running afoul of RefCell aliasing.
    static SH_BUFFERS: std::cell::RefCell<(Vec<(f32, f32)>, Vec<(f32, f32)>)>
        = std::cell::RefCell::new((Vec::new(), Vec::new()));
}

pub fn boolean_op(a: &Polyline, b: &Polyline, clip_type: i32) -> Vec<Polyline> {
    let ca = &a.coords; let cb = &b.coords;
    let mut na = ca.len() / 3; let mut nb = cb.len() / 3;
    // strip closing duplicate
    if na >= 2 { let dx = ca[(na-1)*3]-ca[0]; let dy = ca[(na-1)*3+1]-ca[1]; if dx*dx+dy*dy < 1e-20 { na -= 1; } }
    if nb >= 2 { let dx = cb[(nb-1)*3]-cb[0]; let dy = cb[(nb-1)*3+1]-cb[1]; if dx*dx+dy*dy < 1e-20 { nb -= 1; } }
    if na < 3 || nb < 3 { return vec![]; }

    // Compute safe scale from actual coordinate range to prevent int64 overflow
    // in cross products: (max_coord * scale)^2 must fit in int64
    let mut max_coord: f32 = 0.0;
    for i in 0..na { max_coord = max_coord.max(ca[i*3].abs()).max(ca[i*3+1].abs()); }
    for i in 0..nb { max_coord = max_coord.max(cb[i*3].abs()).max(cb[i*3+1].abs()); }
    if max_coord < 1e-12 { max_coord = 1.0; }
    let bool_scale: f32 = ((i64::MAX as f32).sqrt() / max_coord * 0.99).floor();
    let bool_inv_scale: f32 = 1.0 / bool_scale;

    SCRATCH.with(|cell| {
        let mut sc = cell.borrow_mut();
        sc.clear_for_reuse();
        // Pre-reserve like C++ at boolean_polyline.cpp:1133-1139 — eliminates grow-by-doubling
        let total = na + nb;
        sc.vtx.reserve(total + 4);
        sc.act.reserve(total * 2 + 4);
        sc.opt.reserve(total * 4);
        sc.orc.reserve(total);
        sc.locmin.reserve(total);
        sc.orclist.reserve(total);
        sc.scan.reserve(total * 2);
        boolean_op_inner(&mut sc, a, b, ca, cb, na, nb, clip_type, bool_scale, bool_inv_scale)
    })
}

fn boolean_op_inner(
    sc: &mut Sc,
    a: &Polyline, b: &Polyline,
    ca: &[f32], cb: &[f32],
    na: usize, nb: usize,
    clip_type: i32,
    bool_scale: f32, bool_inv_scale: f32,
) -> Vec<Polyline> {
    // Small polygon fast path
    if (na as i64) * (nb as i64) <= 400 {
        let cvt = |c: &[f32], i: usize| -> Pt { Pt { x: (c[i*3]*bool_scale).round() as i64, y: (c[i*3+1]*bool_scale).round() as i64 } };
        let va: Vec<Pt> = (0..na).map(|i| cvt(ca, i)).collect();
        let vb: Vec<Pt> = (0..nb).map(|i| cvt(cb, i)).collect();
        let (mut aminx, mut amaxx, mut aminy, mut amaxy) = (va[0].x, va[0].x, va[0].y, va[0].y);
        for p in &va[1..] { aminx = aminx.min(p.x); amaxx = amaxx.max(p.x); aminy = aminy.min(p.y); amaxy = amaxy.max(p.y); }
        let (mut bminx, mut bmaxx, mut bminy, mut bmaxy) = (vb[0].x, vb[0].x, vb[0].y, vb[0].y);
        for p in &vb[1..] { bminx = bminx.min(p.x); bmaxx = bmaxx.max(p.x); bminy = bminy.min(p.y); bmaxy = bmaxy.max(p.y); }
        if amaxx < bminx || bmaxx < aminx || amaxy < bminy || bmaxy < aminy {
            let a_in_b = pip_i(va[0], &vb); let b_in_a = pip_i(vb[0], &va);
            if clip_type == 0 { return if a_in_b { vec![a.clone()] } else if b_in_a { vec![b.clone()] } else { vec![] }; }
            if clip_type == 1 { return if a_in_b { vec![b.clone()] } else if b_in_a { vec![a.clone()] } else { vec![a.clone(), b.clone()] }; }
            return if a_in_b { vec![] } else if b_in_a { vec![a.clone()] } else { vec![a.clone()] };
        }
        let mut any_cross = false;
        'outer: for i in 0..na {
            let (a1, a2) = (va[i], va[(i+1)%na]);
            for j in 0..nb {
                let (b1, b2) = (vb[j], vb[(j+1)%nb]);
                if segs_intersect(a1, a2, b1, b2) { any_cross = true; break 'outer; }
            }
        }
        if !any_cross {
            let a_in_b = pip_i(va[0], &vb); let b_in_a = pip_i(vb[0], &va);
            if clip_type == 0 { return if a_in_b { vec![a.clone()] } else if b_in_a { vec![b.clone()] } else { vec![] }; }
            if clip_type == 1 { return if a_in_b { vec![b.clone()] } else if b_in_a { vec![a.clone()] } else { vec![a.clone(), b.clone()] }; }
            return if a_in_b { vec![] } else if b_in_a { vec![a.clone()] } else { vec![a.clone()] };
        }
        // Convex intersection fast path: we've just verified via any_cross==true
        // that the polygons actually intersect AND we haven't short-circuited via
        // disjoint-AABB or no-crossing. For convex CCW quads (every ElementPlate
        // face — see element.rs:760-800), Sutherland-Hodgman in float64 skips the
        // rest of the Vatti sweep-line pipeline below. For non-convex inputs this
        // falls through to the full Vatti path as before.
        if clip_type == 0
            && convex_orientation_2d(ca, na) == 1
            && convex_orientation_2d(cb, nb) == 1
        {
            return SH_BUFFERS.with(|cell| {
                let mut borrow = cell.borrow_mut();
                let (buf_in, buf_out) = &mut *borrow;
                sutherland_hodgman_ccw_intersect(buf_in, buf_out, ca, na, cb, nb)
            });
        }
        // build vertex lists from va/vb arrays
        for v in &va { let vi = sc.alloc_vtx(); sc.vtx[vi].pt = *v; }
        // link + detect local minima for va
        let base_a = 0;
        { let mut prev = base_a; let mut cnt = 1usize;
          for i in 1..na { if va[i] == sc.vtx[prev].pt { continue; } let ci = base_a + cnt;
              sc.vtx[ci].prev = Some(prev); sc.vtx[prev].next = Some(ci); prev = ci; cnt += 1; }
          sc.vtx[prev].next = Some(base_a); sc.vtx[base_a].prev = Some(prev);
          // locmin detection
          let mut pv = sc.vtx[base_a].prev.unwrap();
          while pv != base_a && sc.vtx[pv].pt.y == sc.vtx[base_a].pt.y { pv = sc.vtx[pv].prev.unwrap(); }
          if pv != base_a {
              let mut going_up = sc.vtx[pv].pt.y > sc.vtx[base_a].pt.y; let going_up0 = going_up;
              pv = base_a; let mut cv = sc.vtx[base_a].next.unwrap();
              while cv != base_a {
                  if sc.vtx[cv].pt.y > sc.vtx[pv].pt.y && going_up { sc.vtx[pv].flags |= VF_LOCAL_MAX; going_up = false; }
                  else if sc.vtx[cv].pt.y < sc.vtx[pv].pt.y && !going_up { going_up = true; sc.vtx[pv].flags |= VF_LOCAL_MIN; sc.locmin.push(VLocalMinima { vertex: pv, polytype: 0 }); }
                  pv = cv; cv = sc.vtx[cv].next.unwrap();
              }
              if going_up != going_up0 { if going_up0 { sc.vtx[pv].flags |= VF_LOCAL_MIN; sc.locmin.push(VLocalMinima { vertex: pv, polytype: 0 }); } else { sc.vtx[pv].flags |= VF_LOCAL_MAX; } }
          }
        }
        let base_b = na;
        for v in &vb { let vi = sc.alloc_vtx(); sc.vtx[vi].pt = *v; }
        // wait, we already allocated va vertices above. Let me fix: the vb vertices start at na
        { let mut prev = base_b; let mut cnt = 1usize;
          for i in 1..nb { if vb[i] == sc.vtx[prev].pt { continue; } let ci = base_b + cnt;
              sc.vtx[ci].prev = Some(prev); sc.vtx[prev].next = Some(ci); prev = ci; cnt += 1; }
          sc.vtx[prev].next = Some(base_b); sc.vtx[base_b].prev = Some(prev);
          let mut pv = sc.vtx[base_b].prev.unwrap();
          while pv != base_b && sc.vtx[pv].pt.y == sc.vtx[base_b].pt.y { pv = sc.vtx[pv].prev.unwrap(); }
          if pv != base_b {
              let mut going_up = sc.vtx[pv].pt.y > sc.vtx[base_b].pt.y; let going_up0 = going_up;
              pv = base_b; let mut cv = sc.vtx[base_b].next.unwrap();
              while cv != base_b {
                  if sc.vtx[cv].pt.y > sc.vtx[pv].pt.y && going_up { sc.vtx[pv].flags |= VF_LOCAL_MAX; going_up = false; }
                  else if sc.vtx[cv].pt.y < sc.vtx[pv].pt.y && !going_up { going_up = true; sc.vtx[pv].flags |= VF_LOCAL_MIN; sc.locmin.push(VLocalMinima { vertex: pv, polytype: 1 }); }
                  pv = cv; cv = sc.vtx[cv].next.unwrap();
              }
              if going_up != going_up0 { if going_up0 { sc.vtx[pv].flags |= VF_LOCAL_MIN; sc.locmin.push(VLocalMinima { vertex: pv, polytype: 1 }); } else { sc.vtx[pv].flags |= VF_LOCAL_MAX; } }
          }
        }
    } else {
        // Large polygon: direct double→VVertex single pass
        let (va_head, aminx, amaxx, aminy, amaxy) = add_path_from_coords(sc, ca, na, 0, bool_scale);
        let (vb_head, bminx, bmaxx, bminy, bmaxy) = add_path_from_coords(sc, cb, nb, 1, bool_scale);
        if va_head.is_none() || vb_head.is_none() { return vec![]; }
        if amaxx < bminx || bmaxx < aminx || amaxy < bminy || bmaxy < aminy {
            let a_in_b = pip_vertex(sc, sc.vtx[va_head.unwrap()].pt, vb_head.unwrap());
            let b_in_a = pip_vertex(sc, sc.vtx[vb_head.unwrap()].pt, va_head.unwrap());
            if clip_type == 0 { return if a_in_b { vec![a.clone()] } else if b_in_a { vec![b.clone()] } else { vec![] }; }
            if clip_type == 1 { return if a_in_b { vec![b.clone()] } else if b_in_a { vec![a.clone()] } else { vec![a.clone(), b.clone()] }; }
            return if a_in_b { vec![] } else if b_in_a { vec![a.clone()] } else { vec![a.clone()] };
        }
    }

    if !execute_internal(sc, clip_type) { return vec![]; }

    // Extract output
    let mut out = Vec::new();
    let orc_indices: Vec<usize> = sc.orclist.clone();
    for &or_idx in &orc_indices {
        if sc.orc[or_idx].pts.is_none() { continue; }
        clean_collinear(sc, or_idx);
        if sc.orc[or_idx].pts.is_none() { continue; }
        let op = sc.orc[or_idx].pts.unwrap();
        if sc.opt[op].next == op || sc.opt[op].next == sc.opt[op].prev { continue; }
        if very_small_tri(sc, op) { continue; }
        let mut coords = Vec::new();
        let mut o = sc.opt[op].next; let mut last = sc.opt[o].pt;
        coords.push(last.x as f32 * bool_inv_scale); coords.push(last.y as f32 * bool_inv_scale); coords.push(0.0);
        o = sc.opt[o].next;
        let start = sc.opt[op].next;
        while o != start {
            if sc.opt[o].pt != last { last = sc.opt[o].pt;
                coords.push(last.x as f32 * bool_inv_scale); coords.push(last.y as f32 * bool_inv_scale); coords.push(0.0); }
            o = sc.opt[o].next;
        }
        if coords.len() >= 9 { out.push(Polyline::from_coords(coords)); }
    }
    out
}

#[cfg(test)]
#[path = "boolean_polyline_test.rs"]
mod boolean_polyline_test;
