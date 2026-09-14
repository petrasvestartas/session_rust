use crate::brep::{BRep, BRepOrientation, BRepRef};
use crate::closest::Closest;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::nurbssurface_trimmed::NurbsSurfaceTrimmed;
use crate::point::Point;
use crate::vector::Vector;
use std::collections::HashMap;

// ═══════════════════════════════════════════════════════════════════════════
// ISO 10303-21 parser
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone, Copy, PartialEq)]
enum StepTag {
    Ref,
    Num,
    Str,
    Enum,
    List,
    Null,
}

#[derive(Clone)]
struct StepParam {
    tag: StepTag,
    ref_id: i32,
    num: f64,
    str: String,
    list: Vec<StepParam>,
}

impl StepParam {
    fn new() -> Self {
        StepParam {
            tag: StepTag::Null,
            ref_id: 0,
            num: 0.0,
            str: String::new(),
            list: Vec::new(),
        }
    }
}

struct StepSubEntity {
    type_: String,
    params: Vec<StepParam>,
}

struct StepEntity {
    parts: Vec<StepSubEntity>,
}

impl StepEntity {
    fn has(&self, t: &str) -> bool {
        self.parts.iter().any(|p| p.type_ == t)
    }

    fn find(&self, t: &str) -> Option<&StepSubEntity> {
        self.parts.iter().find(|p| p.type_ == t)
    }
}

struct StepFile {
    entities: HashMap<i32, StepEntity>,
}

impl StepFile {
    fn ids_of_type(&self, t: &str) -> Vec<i32> {
        let mut out: Vec<i32> = self
            .entities
            .iter()
            .filter(|(_, e)| e.has(t))
            .map(|(k, _)| *k)
            .collect();
        out.sort();
        out
    }
}

const PI: f64 = std::f64::consts::PI;
const PI_2: f64 = std::f64::consts::FRAC_PI_2;

struct Cursor<'a> {
    s: &'a [u8],
    p: usize,
}

fn skip_ws(c: &mut Cursor) {
    while c.p < c.s.len() && c.s[c.p].is_ascii_whitespace() {
        c.p += 1;
    }
}

fn consume(c: &mut Cursor, ch: u8) -> bool {
    skip_ws(c);
    if c.p < c.s.len() && c.s[c.p] == ch {
        c.p += 1;
        return true;
    }
    false
}

fn isident(ch: u8) -> bool {
    ch.is_ascii_uppercase() || ch.is_ascii_digit() || ch == b'_'
}

fn parse_params(c: &mut Cursor) -> Vec<StepParam> {
    let mut out = Vec::new();
    consume(c, b'(');
    loop {
        skip_ws(c);
        if c.p >= c.s.len() || c.s[c.p] == b')' {
            break;
        }
        out.push(parse_param(c));
        skip_ws(c);
        if c.p < c.s.len() && c.s[c.p] == b',' {
            c.p += 1;
        }
    }
    consume(c, b')');
    out
}

fn parse_number(c: &mut Cursor) -> f64 {
    let start = c.p;
    if c.p < c.s.len() && (c.s[c.p] == b'+' || c.s[c.p] == b'-') {
        c.p += 1;
    }
    while c.p < c.s.len() && (c.s[c.p].is_ascii_digit() || c.s[c.p] == b'.') {
        c.p += 1;
    }
    if c.p < c.s.len() && (c.s[c.p] == b'e' || c.s[c.p] == b'E') {
        let mut q = c.p + 1;
        if q < c.s.len() && (c.s[q] == b'+' || c.s[q] == b'-') {
            q += 1;
        }
        if q < c.s.len() && c.s[q].is_ascii_digit() {
            c.p = q;
            while c.p < c.s.len() && c.s[c.p].is_ascii_digit() {
                c.p += 1;
            }
        }
    }
    std::str::from_utf8(&c.s[start..c.p])
        .ok()
        .and_then(|t| t.parse::<f64>().ok())
        .unwrap_or(0.0)
}

fn parse_ident(c: &mut Cursor) -> String {
    let start = c.p;
    while c.p < c.s.len() && isident(c.s[c.p]) {
        c.p += 1;
    }
    String::from_utf8_lossy(&c.s[start..c.p]).into_owned()
}

fn parse_param(c: &mut Cursor) -> StepParam {
    skip_ws(c);
    let mut r = StepParam::new();
    if c.p >= c.s.len() {
        return r;
    }
    let ch = c.s[c.p];
    if ch == b'#' {
        c.p += 1;
        let mut id = 0i32;
        while c.p < c.s.len() && c.s[c.p].is_ascii_digit() {
            id = id * 10 + (c.s[c.p] - b'0') as i32;
            c.p += 1;
        }
        r.tag = StepTag::Ref;
        r.ref_id = id;
        return r;
    }
    if ch == b'$' || ch == b'*' {
        c.p += 1;
        r.tag = StepTag::Null;
        return r;
    }
    if ch == b'(' {
        r.tag = StepTag::List;
        r.list = parse_params(c);
        return r;
    }
    if ch == b'\'' {
        c.p += 1;
        let mut buf = Vec::new();
        while c.p < c.s.len() {
            if c.s[c.p] == b'\'' {
                c.p += 1;
                if c.p < c.s.len() && c.s[c.p] == b'\'' {
                    buf.push(b'\'');
                    c.p += 1;
                } else {
                    break;
                }
            } else {
                buf.push(c.s[c.p]);
                c.p += 1;
            }
        }
        r.str = String::from_utf8_lossy(&buf).into_owned();
        r.tag = StepTag::Str;
        return r;
    }
    if ch == b'.' {
        c.p += 1;
        let start = c.p;
        while c.p < c.s.len() && c.s[c.p] != b'.' {
            c.p += 1;
        }
        r.str = String::from_utf8_lossy(&c.s[start..c.p]).into_owned();
        if c.p < c.s.len() {
            c.p += 1;
        }
        r.tag = StepTag::Enum;
        return r;
    }
    if ch.is_ascii_digit() || ch == b'-' || ch == b'+' {
        r.num = parse_number(c);
        r.tag = StepTag::Num;
        return r;
    }
    if ch.is_ascii_uppercase() {
        r.str = parse_ident(c);
        skip_ws(c);
        r.tag = StepTag::Enum;
        if c.p < c.s.len() && c.s[c.p] == b'(' {
            parse_params(c);
        }
        return r;
    }
    c.p += 1;
    r
}

fn parse_step_string(content: &str, sf: &mut StepFile) {
    let mut c = Cursor {
        s: content.as_bytes(),
        p: 0,
    };
    while c.p < c.s.len() {
        skip_ws(&mut c);
        if c.p >= c.s.len() {
            break;
        }
        if c.s[c.p] != b'#' {
            while c.p < c.s.len() && c.s[c.p] != b'\n' {
                c.p += 1;
            }
            continue;
        }
        c.p += 1;
        let mut id = 0i32;
        while c.p < c.s.len() && c.s[c.p].is_ascii_digit() {
            id = id * 10 + (c.s[c.p] - b'0') as i32;
            c.p += 1;
        }
        if !consume(&mut c, b'=') {
            continue;
        }
        skip_ws(&mut c);
        if c.p >= c.s.len() {
            break;
        }
        let mut ent = StepEntity { parts: Vec::new() };
        if c.s[c.p] == b'(' {
            c.p += 1;
            while c.p < c.s.len() {
                skip_ws(&mut c);
                if c.s[c.p] == b')' {
                    c.p += 1;
                    break;
                }
                if !c.s[c.p].is_ascii_uppercase() {
                    break;
                }
                let type_ = parse_ident(&mut c);
                skip_ws(&mut c);
                let mut params = Vec::new();
                if c.p < c.s.len() && c.s[c.p] == b'(' {
                    params = parse_params(&mut c);
                }
                ent.parts.push(StepSubEntity { type_, params });
            }
        } else {
            let type_ = parse_ident(&mut c);
            skip_ws(&mut c);
            let mut params = Vec::new();
            if c.p < c.s.len() && c.s[c.p] == b'(' {
                params = parse_params(&mut c);
            }
            ent.parts.push(StepSubEntity { type_, params });
        }
        sf.entities.entry(id).or_insert(ent);
        let mut in_str = false;
        while c.p < c.s.len() {
            if c.s[c.p] == b'\'' && !in_str {
                in_str = true;
            } else if c.s[c.p] == b'\'' && in_str {
                in_str = false;
            } else if c.s[c.p] == b';' && !in_str {
                c.p += 1;
                break;
            }
            c.p += 1;
        }
    }
}

fn parse_step_file(filepath: &str) -> StepFile {
    let mut sf = StepFile {
        entities: HashMap::new(),
    };
    let raw = match std::fs::read(filepath) {
        Ok(r) => String::from_utf8_lossy(&r).into_owned(),
        Err(_) => return sf,
    };
    let bytes = raw.as_bytes();
    let mut text: Vec<u8> = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        if i + 1 < bytes.len() && bytes[i] == b'/' && bytes[i + 1] == b'*' {
            i += 2;
            while i + 1 < bytes.len() && !(bytes[i] == b'*' && bytes[i + 1] == b'/') {
                i += 1;
            }
            i += 2;
        } else {
            text.push(bytes[i]);
            i += 1;
        }
    }
    let text = String::from_utf8_lossy(&text).into_owned();
    let Some(lo) = text.find("DATA") else {
        return sf;
    };
    let Some(semi) = text[lo..].find(';').map(|k| k + lo) else {
        return sf;
    };
    let Some(endsec) = text[semi..].find("ENDSEC").map(|k| k + semi) else {
        return sf;
    };
    parse_step_string(&text[semi + 1..endsec], &mut sf);
    sf
}

// ═══════════════════════════════════════════════════════════════════════════
// Knot utilities
// ═══════════════════════════════════════════════════════════════════════════

fn expand_knots(vals: &[f64], mults: &[i32]) -> Vec<f64> {
    let mut flat = Vec::new();
    for i in 0..vals.len().min(mults.len()) {
        for _ in 0..mults[i] {
            flat.push(vals[i]);
        }
    }
    flat
}

fn compress_knots(flat: &[f64]) -> (Vec<f64>, Vec<i32>) {
    let mut vals: Vec<f64> = Vec::new();
    let mut mults: Vec<i32> = Vec::new();
    for &v in flat {
        if vals.is_empty() || (v - vals[vals.len() - 1]).abs() > 1e-12 {
            vals.push(v);
            mults.push(1);
        } else {
            let n = mults.len();
            mults[n - 1] += 1;
        }
    }
    (vals, mults)
}

fn full_from_internal(internal: &[f64]) -> Vec<f64> {
    if internal.is_empty() {
        return Vec::new();
    }
    let mut full = vec![internal[0]];
    full.extend_from_slice(internal);
    full.push(internal[internal.len() - 1]);
    full
}

fn internal_from_full(full: &[f64]) -> Vec<f64> {
    if full.len() < 2 {
        return full.to_vec();
    }
    full[1..full.len() - 1].to_vec()
}

// ═══════════════════════════════════════════════════════════════════════════
// StepReader
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
struct Axis2 {
    origin: Point,
    ax: Vector,
    ay: Vector,
    az: Vector,
    ok: bool,
}

impl Axis2 {
    fn new() -> Self {
        Axis2 {
            origin: Point::new(0.0, 0.0, 0.0),
            ax: Vector::new(1.0, 0.0, 0.0),
            ay: Vector::new(0.0, 1.0, 0.0),
            az: Vector::new(0.0, 0.0, 1.0),
            ok: false,
        }
    }
}

#[derive(Clone)]
struct Proj {
    kind: i32,
    a: Axis2,
}

#[derive(Clone)]
struct AnFace {
    kind: i32,
    a: Axis2,
    r: f64,
    r2: f64,
}

fn coords(params: &[StepParam]) -> Vec<f64> {
    for p in params {
        if p.tag == StepTag::List {
            let out: Vec<f64> = p
                .list
                .iter()
                .filter(|v| v.tag == StepTag::Num)
                .map(|v| v.num)
                .collect();
            if !out.is_empty() {
                return out;
            }
        }
    }
    Vec::new()
}

fn first_ref(params: &[StepParam]) -> i32 {
    for p in params {
        if p.tag == StepTag::Ref {
            return p.ref_id;
        }
    }
    -1
}

fn all_refs(params: &[StepParam]) -> Vec<i32> {
    params
        .iter()
        .filter(|p| p.tag == StepTag::Ref)
        .map(|p| p.ref_id)
        .collect()
}

fn list_refs(params: &[StepParam]) -> Vec<i32> {
    let mut out = Vec::new();
    for p in params {
        if p.tag == StepTag::List {
            for v in &p.list {
                if v.tag == StepTag::Ref {
                    out.push(v.ref_id);
                }
            }
        }
    }
    out
}

fn int_list(p: &StepParam) -> Vec<i32> {
    let mut out = Vec::new();
    if p.tag == StepTag::List {
        for v in &p.list {
            if v.tag == StepTag::Num {
                out.push(v.num as i32);
            }
        }
    }
    out
}

fn dbl_list(p: &StepParam) -> Vec<f64> {
    let mut out = Vec::new();
    if p.tag == StepTag::List {
        for v in &p.list {
            if v.tag == StepTag::Num {
                out.push(v.num);
            }
        }
    }
    out
}

fn dbl_list_list(p: &StepParam) -> Vec<Vec<f64>> {
    let mut out = Vec::new();
    if p.tag == StepTag::List {
        for row in &p.list {
            out.push(dbl_list(row));
        }
    }
    out
}

fn ref_list_list(p: &StepParam) -> Vec<Vec<i32>> {
    let mut out = Vec::new();
    if p.tag == StepTag::List {
        for row in &p.list {
            let mut row_refs = Vec::new();
            if row.tag == StepTag::List {
                for v in &row.list {
                    if v.tag == StepTag::Ref {
                        row_refs.push(v.ref_id);
                    }
                }
            }
            out.push(row_refs);
        }
    }
    out
}

fn arc_param_of_angle(theta: f64) -> f64 {
    if theta <= 0.0 {
        return 0.0;
    }
    if theta >= PI_2 {
        return 1.0;
    }
    let w = 2.0f64.sqrt() / 2.0;
    let mut tau = theta / PI_2;
    for _ in 0..8 {
        let o = 1.0 - tau;
        let x = o * o + 2.0 * w * tau * o;
        let y = 2.0 * w * tau * o + tau * tau;
        let dx = -2.0 * o + 2.0 * w * (1.0 - 2.0 * tau);
        let dy = 2.0 * w * (1.0 - 2.0 * tau) + 2.0 * tau;
        let f = y.atan2(x) - theta;
        let r2 = x * x + y * y;
        let df = (x * dy - y * dx) / (if r2 > 1e-30 { r2 } else { 1e-30 });
        if df.abs() < 1e-30 {
            break;
        }
        let step = f / df;
        tau -= step;
        tau = tau.clamp(0.0, 1.0);
        if step.abs() < 1e-15 {
            break;
        }
    }
    tau
}

fn chart_u_of_angle(ang: f64, q0: i32) -> f64 {
    let q = ang / PI_2 - q0 as f64;
    let spanf = (q + 1e-12).floor();
    let theta = (q - spanf) * PI_2;
    spanf + arc_param_of_angle(theta)
}

fn curve_domain(nc: &NurbsCurve) -> Option<(f64, f64)> {
    let kts = nc.get_nurbsknots();
    if kts.is_empty() {
        return None;
    }
    let deg = nc.order() - 1;
    let tmin = kts[if deg > 0 { deg - 1 } else { 0 }];
    let tmax = kts[kts.len() - (if deg > 0 { deg } else { 1 })];
    Some((tmin, tmax))
}

fn angle_of(a: &Axis2, pt: &Point) -> f64 {
    let dx = pt[0] - a.origin[0];
    let dy = pt[1] - a.origin[1];
    let dz = pt[2] - a.origin[2];
    let u = dx * a.ax[0] + dy * a.ax[1] + dz * a.ax[2];
    let v2 = dx * a.ay[0] + dy * a.ay[1] + dz * a.ay[2];
    v2.atan2(u)
}

fn an_place(a: &AnFace, lx: f64, ly: f64, lz: f64) -> Point {
    Point::new(
        a.a.origin[0] + lx * a.a.ax[0] + ly * a.a.ay[0] + lz * a.a.az[0],
        a.a.origin[1] + lx * a.a.ax[1] + ly * a.a.ay[1] + lz * a.a.az[1],
        a.a.origin[2] + lx * a.a.ax[2] + ly * a.a.ay[2] + lz * a.a.az[2],
    )
}

fn quarter_knots(nspans: usize) -> Vec<f64> {
    let mut knots = vec![0.0, 0.0];
    for s in 1..nspans {
        knots.push(s as f64);
        knots.push(s as f64);
    }
    knots.push(nspans as f64);
    knots.push(nspans as f64);
    knots
}

fn polyline_nurbs(pts: &[Point], dim: usize) -> NurbsCurve {
    let n = pts.len();
    if n < 2 {
        return NurbsCurve::new(dim, false, 2, 0);
    }
    let mut nc = NurbsCurve::new(dim, false, 2, n);
    for i in 0..n {
        nc.m_nurbsknot[i] = i as f64;
    }
    for i in 0..n {
        for d in 0..dim {
            nc.m_cv[i * dim + d] = pts[i][d];
        }
    }
    nc
}

fn distance(a: &Point, b: &Point) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

struct StepReader<'a> {
    sf: &'a StepFile,
    pt_cache: HashMap<i32, Point>,
    dir_cache: HashMap<i32, Vector>,
    ax_cache: HashMap<i32, Axis2>,
}

impl<'a> StepReader<'a> {
    fn new(sf: &'a StepFile) -> Self {
        StepReader {
            sf,
            pt_cache: HashMap::new(),
            dir_cache: HashMap::new(),
            ax_cache: HashMap::new(),
        }
    }

    fn get(&self, id: i32) -> Option<&'a StepEntity> {
        self.sf.entities.get(&id)
    }

    fn get_point(&mut self, id: i32) -> Point {
        if let Some(p) = self.pt_cache.get(&id) {
            return p.clone();
        }
        let mut pt = Point::new(0.0, 0.0, 0.0);
        if let Some(sub) = self.get(id).and_then(|e| e.find("CARTESIAN_POINT")) {
            let c = coords(&sub.params);
            if c.len() >= 3 {
                pt = Point::new(c[0], c[1], c[2]);
            } else if c.len() == 2 {
                pt = Point::new(c[0], c[1], 0.0);
            }
        }
        self.pt_cache.insert(id, pt.clone());
        pt
    }

    fn get_direction(&mut self, id: i32) -> Vector {
        if let Some(v) = self.dir_cache.get(&id) {
            return v.clone();
        }
        let mut v = Vector::new(0.0, 0.0, 1.0);
        if let Some(sub) = self.get(id).and_then(|e| e.find("DIRECTION")) {
            let c = coords(&sub.params);
            if c.len() >= 3 {
                v = Vector::new(c[0], c[1], c[2]);
            }
        }
        self.dir_cache.insert(id, v.clone());
        v
    }

    fn get_axis2(&mut self, id: i32) -> Axis2 {
        if let Some(a) = self.ax_cache.get(&id) {
            return a.clone();
        }
        let mut a = Axis2::new();
        let refs = match self.get(id).and_then(|e| e.find("AXIS2_PLACEMENT_3D")) {
            Some(sub) => all_refs(&sub.params),
            None => Vec::new(),
        };
        if !refs.is_empty() {
            a.origin = self.get_point(refs[0]);
            a.az = if refs.len() > 1 {
                self.get_direction(refs[1])
            } else {
                Vector::new(0.0, 0.0, 1.0)
            };
            let ln = (a.az[0] * a.az[0] + a.az[1] * a.az[1] + a.az[2] * a.az[2]).sqrt();
            if ln > 1e-12 {
                a.az = Vector::new(a.az[0] / ln, a.az[1] / ln, a.az[2] / ln);
            }
            if refs.len() > 2 {
                a.ax = self.get_direction(refs[2]);
            } else {
                a.ax = if a.az[0].abs() < 0.9 {
                    Vector::new(1.0, 0.0, 0.0)
                } else {
                    Vector::new(0.0, 1.0, 0.0)
                };
            }
            let dot = a.ax[0] * a.az[0] + a.ax[1] * a.az[1] + a.ax[2] * a.az[2];
            a.ax = Vector::new(
                a.ax[0] - dot * a.az[0],
                a.ax[1] - dot * a.az[1],
                a.ax[2] - dot * a.az[2],
            );
            let xn = (a.ax[0] * a.ax[0] + a.ax[1] * a.ax[1] + a.ax[2] * a.ax[2]).sqrt();
            if xn > 1e-12 {
                a.ax = Vector::new(a.ax[0] / xn, a.ax[1] / xn, a.ax[2] / xn);
            }
            a.ay = Vector::new(
                a.az[1] * a.ax[2] - a.az[2] * a.ax[1],
                a.az[2] * a.ax[0] - a.az[0] * a.ax[2],
                a.az[0] * a.ax[1] - a.az[1] * a.ax[0],
            );
            a.ok = true;
        }
        self.ax_cache.insert(id, a.clone());
        a
    }

    fn get_nurbs_curve(&mut self, id: i32) -> NurbsCurve {
        let Some(e) = self.get(id) else {
            return NurbsCurve::default();
        };
        let bsc = e.find("B_SPLINE_CURVE_WITH_KNOTS");
        let bsc_base = e.find("B_SPLINE_CURVE");
        let rat = e.find("RATIONAL_B_SPLINE_CURVE");

        let mut degree = 0i32;
        let mut pt_refs: Vec<i32> = Vec::new();
        let mut mults_i: Vec<i32> = Vec::new();
        let mut knots_d: Vec<f64> = Vec::new();
        let mut weights: Vec<f64> = Vec::new();

        match (bsc, bsc_base) {
            (Some(bsc), None) => {
                let pp = &bsc.params;
                if pp.len() < 8 {
                    return NurbsCurve::default();
                }
                if pp[1].tag == StepTag::Num {
                    degree = pp[1].num as i32;
                }
                if pp[2].tag == StepTag::List {
                    for v in &pp[2].list {
                        if v.tag == StepTag::Ref {
                            pt_refs.push(v.ref_id);
                        }
                    }
                }
                if pp[6].tag == StepTag::List {
                    mults_i = int_list(&pp[6]);
                }
                if pp[7].tag == StepTag::List {
                    knots_d = dbl_list(&pp[7]);
                }
            }
            (Some(bsc), Some(bsc_base)) => {
                let base_pp = &bsc_base.params;
                let knt_pp = &bsc.params;
                if !base_pp.is_empty() && base_pp[0].tag == StepTag::Num {
                    degree = base_pp[0].num as i32;
                }
                if base_pp.len() > 1 && base_pp[1].tag == StepTag::List {
                    for v in &base_pp[1].list {
                        if v.tag == StepTag::Ref {
                            pt_refs.push(v.ref_id);
                        }
                    }
                }
                if !knt_pp.is_empty() {
                    mults_i = int_list(&knt_pp[0]);
                }
                if knt_pp.len() > 1 {
                    knots_d = dbl_list(&knt_pp[1]);
                }
            }
            _ => return NurbsCurve::default(),
        }

        if pt_refs.is_empty() || mults_i.is_empty() || knots_d.is_empty() {
            return NurbsCurve::default();
        }
        let order = (degree + 1) as usize;
        let cv_count = pt_refs.len();

        let full = expand_knots(&knots_d, &mults_i);
        if full.len() != cv_count + order {
            return NurbsCurve::default();
        }
        let internal = internal_from_full(&full);

        let is_rat = rat.is_some();
        if let Some(rat) = rat {
            if !rat.params.is_empty() && rat.params[0].tag == StepTag::List {
                weights = dbl_list(&rat.params[0]);
            }
        }

        let mut nc = NurbsCurve::new(3, is_rat, order, cv_count);
        if nc.m_nurbsknot.len() != internal.len() {
            return NurbsCurve::default();
        }
        nc.m_nurbsknot.copy_from_slice(&internal);

        let stride = nc.m_cv_stride;
        for i in 0..cv_count {
            let pt = self.get_point(pt_refs[i]);
            if is_rat {
                let w = if i < weights.len() { weights[i] } else { 1.0 };
                nc.m_cv[i * stride] = w * pt[0];
                nc.m_cv[i * stride + 1] = w * pt[1];
                nc.m_cv[i * stride + 2] = w * pt[2];
                nc.m_cv[i * stride + 3] = w;
            } else {
                nc.m_cv[i * stride] = pt[0];
                nc.m_cv[i * stride + 1] = pt[1];
                nc.m_cv[i * stride + 2] = pt[2];
            }
        }
        nc
    }

    fn get_nurbs_surface(&mut self, id: i32) -> NurbsSurface {
        let Some(e) = self.get(id) else {
            return NurbsSurface::default();
        };
        let bss = e.find("B_SPLINE_SURFACE_WITH_KNOTS");
        let bss_base = e.find("B_SPLINE_SURFACE");
        let rat = e.find("RATIONAL_B_SPLINE_SURFACE");

        let mut u_deg = 0i32;
        let mut v_deg = 0i32;
        let mut ctrl_pts: Vec<Vec<i32>> = Vec::new();
        let mut u_mults: Vec<i32> = Vec::new();
        let mut v_mults: Vec<i32> = Vec::new();
        let mut u_knots: Vec<f64> = Vec::new();
        let mut v_knots: Vec<f64> = Vec::new();
        let mut weights: Vec<Vec<f64>> = Vec::new();

        match (bss, bss_base) {
            (Some(bss), None) => {
                let pp = &bss.params;
                if pp.len() < 12 {
                    return NurbsSurface::default();
                }
                if pp[1].tag == StepTag::Num {
                    u_deg = pp[1].num as i32;
                }
                if pp[2].tag == StepTag::Num {
                    v_deg = pp[2].num as i32;
                }
                if pp[3].tag == StepTag::List {
                    ctrl_pts = ref_list_list(&pp[3]);
                }
                if pp[8].tag == StepTag::List {
                    u_mults = int_list(&pp[8]);
                }
                if pp[9].tag == StepTag::List {
                    v_mults = int_list(&pp[9]);
                }
                if pp[10].tag == StepTag::List {
                    u_knots = dbl_list(&pp[10]);
                }
                if pp[11].tag == StepTag::List {
                    v_knots = dbl_list(&pp[11]);
                }
            }
            (Some(bss), Some(bss_base)) => {
                let base_pp = &bss_base.params;
                let knt_pp = &bss.params;
                if base_pp.len() < 3 {
                    return NurbsSurface::default();
                }
                if base_pp[0].tag == StepTag::Num {
                    u_deg = base_pp[0].num as i32;
                }
                if base_pp[1].tag == StepTag::Num {
                    v_deg = base_pp[1].num as i32;
                }
                if base_pp[2].tag == StepTag::List {
                    ctrl_pts = ref_list_list(&base_pp[2]);
                }
                if knt_pp.len() > 3 {
                    u_mults = int_list(&knt_pp[0]);
                    v_mults = int_list(&knt_pp[1]);
                    u_knots = dbl_list(&knt_pp[2]);
                    v_knots = dbl_list(&knt_pp[3]);
                }
            }
            _ => return NurbsSurface::default(),
        }

        if ctrl_pts.is_empty() || u_mults.is_empty() || v_mults.is_empty() {
            return NurbsSurface::default();
        }
        let cv_u = ctrl_pts.len();
        let cv_v = ctrl_pts[0].len();
        if cv_u == 0 || cv_v == 0 {
            return NurbsSurface::default();
        }

        let full_u = expand_knots(&u_knots, &u_mults);
        let full_v = expand_knots(&v_knots, &v_mults);
        if full_u.len() != cv_u + u_deg as usize + 1 {
            return NurbsSurface::default();
        }
        if full_v.len() != cv_v + v_deg as usize + 1 {
            return NurbsSurface::default();
        }
        let int_u = internal_from_full(&full_u);
        let int_v = internal_from_full(&full_v);

        let is_rat = rat.is_some();
        if let Some(rat) = rat {
            if !rat.params.is_empty() && rat.params[0].tag == StepTag::List {
                weights = dbl_list_list(&rat.params[0]);
            }
        }

        let mut srf = NurbsSurface::new(
            3,
            is_rat,
            u_deg as usize + 1,
            v_deg as usize + 1,
            cv_u,
            cv_v,
        );
        if !srf.is_valid() {
            return NurbsSurface::default();
        }
        if srf.m_nurbsknot[0].len() != int_u.len() || srf.m_nurbsknot[1].len() != int_v.len() {
            return NurbsSurface::default();
        }
        srf.m_nurbsknot[0] = int_u;
        srf.m_nurbsknot[1] = int_v;

        for u in 0..cv_u {
            for v in 0..cv_v.min(ctrl_pts[u].len()) {
                let pt = self.get_point(ctrl_pts[u][v]);
                if is_rat {
                    let w = if u < weights.len() && v < weights[u].len() {
                        weights[u][v]
                    } else {
                        1.0
                    };
                    srf.set_cv_4d(u, v, w * pt[0], w * pt[1], w * pt[2], w);
                } else {
                    srf.set_cv(u, v, &pt);
                }
            }
        }
        srf
    }

    fn basis_curve_of(&self, curve_id: i32) -> i32 {
        let Some(e) = self.get(curve_id) else {
            return curve_id;
        };
        let sc = e.find("SURFACE_CURVE").or_else(|| e.find("SEAM_CURVE"));
        let Some(sc) = sc else { return curve_id };
        let r = first_ref(&sc.params);
        if r >= 0 {
            r
        } else {
            curve_id
        }
    }

    fn sample_curve(
        &mut self,
        curve_id: i32,
        v_start: &Point,
        v_end: &Point,
        n: usize,
    ) -> Vec<Point> {
        let curve_id = self.basis_curve_of(curve_id);
        let ends = vec![v_start.clone(), v_end.clone()];
        let Some(e) = self.get(curve_id) else {
            return ends;
        };
        if e.has("B_SPLINE_CURVE_WITH_KNOTS") {
            let nc = self.get_nurbs_curve(curve_id);
            if nc.cv_count() >= nc.order() && nc.order() >= 2 {
                let Some((tmin, tmax)) = curve_domain(&nc) else {
                    return ends;
                };
                let mut pts = Vec::with_capacity(n);
                for i in 0..n {
                    let t = if n > 1 {
                        tmin + (tmax - tmin) * i as f64 / (n - 1) as f64
                    } else {
                        tmin
                    };
                    pts.push(nc.point_at(t));
                }
                return pts;
            }
        }
        if e.has("LINE") {
            return ends;
        }
        if let Some(sub) = e.find("CIRCLE") {
            let ax_ref = first_ref(&sub.params);
            let mut rad = 0.0;
            for p in &sub.params {
                if p.tag == StepTag::Num {
                    rad = p.num;
                }
            }
            if ax_ref < 0 || rad == 0.0 {
                return ends;
            }
            let a = self.get_axis2(ax_ref);
            if !a.ok {
                return ends;
            }
            let sa = angle_of(&a, v_start);
            let mut ea = angle_of(&a, v_end);
            if ea <= sa {
                ea += 2.0 * PI;
            }
            let mut pts = Vec::with_capacity(n);
            for i in 0..n {
                let t = if n > 1 {
                    i as f64 / (n - 1) as f64
                } else {
                    0.0
                };
                let ang = sa + t * (ea - sa);
                pts.push(Point::new(
                    a.origin[0] + rad * (ang.cos() * a.ax[0] + ang.sin() * a.ay[0]),
                    a.origin[1] + rad * (ang.cos() * a.ax[1] + ang.sin() * a.ay[1]),
                    a.origin[2] + rad * (ang.cos() * a.ax[2] + ang.sin() * a.ay[2]),
                ));
            }
            return pts;
        }
        if let Some(sub) = e.find("TRIMMED_CURVE") {
            let r = first_ref(&sub.params);
            if r >= 0 {
                return self.sample_curve(r, v_start, v_end, n);
            }
        }
        ends
    }

    fn get_projector(&mut self, surface_id: i32) -> Proj {
        let mut pr = Proj {
            kind: 0,
            a: Axis2::new(),
        };
        let Some(e) = self.get(surface_id) else {
            return pr;
        };
        if let Some(sub) = e.find("PLANE") {
            let r = first_ref(&sub.params);
            if r < 0 {
                return pr;
            }
            pr.a = self.get_axis2(r);
            pr.kind = 1;
        } else if let Some(sub) = e.find("CYLINDRICAL_SURFACE") {
            let r = first_ref(&sub.params);
            if r < 0 {
                return pr;
            }
            pr.a = self.get_axis2(r);
            pr.kind = 2;
        }
        pr
    }

    fn project(&self, pr: &Proj, pt: &Point) -> (f64, f64) {
        let dx = pt[0] - pr.a.origin[0];
        let dy = pt[1] - pr.a.origin[1];
        let dz = pt[2] - pr.a.origin[2];
        if pr.kind == 1 {
            return (
                dx * pr.a.ax[0] + dy * pr.a.ax[1] + dz * pr.a.ax[2],
                dx * pr.a.ay[0] + dy * pr.a.ay[1] + dz * pr.a.ay[2],
            );
        }
        let xl = dx * pr.a.ax[0] + dy * pr.a.ax[1] + dz * pr.a.ax[2];
        let yl = dx * pr.a.ay[0] + dy * pr.a.ay[1] + dz * pr.a.ay[2];
        let hl = dx * pr.a.az[0] + dy * pr.a.az[1] + dz * pr.a.az[2];
        (yl.atan2(xl) * 2.0 / PI, hl)
    }

    fn fill_surface(
        &mut self,
        id: i32,
        u0: f64,
        u1: f64,
        v0: f64,
        v1: f64,
    ) -> Option<NurbsSurface> {
        let e = self.get(id)?;
        let mut u0 = u0;
        let mut u1 = u1;
        let mut v0 = v0;
        let mut v1 = v1;
        let is_closed_cyl = e.has("CYLINDRICAL_SURFACE") && ((u1 - u0) - 4.0).abs() < 0.2;
        let pad_v = (1e-6f64).max(0.01 * (v1 - v0));
        v0 -= pad_v;
        v1 += pad_v;
        if !is_closed_cyl && !e.has("CYLINDRICAL_SURFACE") {
            let pad_u = (1e-6f64).max(0.01 * (u1 - u0));
            u0 -= pad_u;
            u1 += pad_u;
        }

        if e.has("B_SPLINE_SURFACE_WITH_KNOTS") {
            let out = self.get_nurbs_surface(id);
            return if out.is_valid() { Some(out) } else { None };
        }

        if let Some(sub) = e.find("PLANE") {
            let ax_ref = first_ref(&sub.params);
            if ax_ref < 0 {
                return None;
            }
            let a = self.get_axis2(ax_ref);
            if !a.ok {
                return None;
            }
            let mut out = NurbsSurface::new(3, false, 2, 2, 2, 2);
            if !out.is_valid() {
                return None;
            }
            out.m_nurbsknot[0] = vec![u0, u1];
            out.m_nurbsknot[1] = vec![v0, v1];
            out.set_cv(
                0,
                0,
                &Point::new(
                    a.origin[0] + u0 * a.ax[0] + v0 * a.ay[0],
                    a.origin[1] + u0 * a.ax[1] + v0 * a.ay[1],
                    a.origin[2] + u0 * a.ax[2] + v0 * a.ay[2],
                ),
            );
            out.set_cv(
                0,
                1,
                &Point::new(
                    a.origin[0] + u0 * a.ax[0] + v1 * a.ay[0],
                    a.origin[1] + u0 * a.ax[1] + v1 * a.ay[1],
                    a.origin[2] + u0 * a.ax[2] + v1 * a.ay[2],
                ),
            );
            out.set_cv(
                1,
                0,
                &Point::new(
                    a.origin[0] + u1 * a.ax[0] + v0 * a.ay[0],
                    a.origin[1] + u1 * a.ax[1] + v0 * a.ay[1],
                    a.origin[2] + u1 * a.ax[2] + v0 * a.ay[2],
                ),
            );
            out.set_cv(
                1,
                1,
                &Point::new(
                    a.origin[0] + u1 * a.ax[0] + v1 * a.ay[0],
                    a.origin[1] + u1 * a.ax[1] + v1 * a.ay[1],
                    a.origin[2] + u1 * a.ax[2] + v1 * a.ay[2],
                ),
            );
            return Some(out);
        }

        if let Some(sub) = e.find("CYLINDRICAL_SURFACE") {
            let mut ax_ref = -1;
            let mut radius = 1.0;
            for p in &sub.params {
                if p.tag == StepTag::Ref && ax_ref < 0 {
                    ax_ref = p.ref_id;
                } else if p.tag == StepTag::Num {
                    radius = p.num;
                }
            }
            if ax_ref < 0 {
                return None;
            }
            let a = self.get_axis2(ax_ref);
            if !a.ok {
                return None;
            }
            let n_spans = if is_closed_cyl {
                4
            } else {
                (((u1 - u0).abs() - 1e-9).ceil() as i64).max(1) as usize
            };
            if is_closed_cyl {
                u1 = u0 + 4.0;
            }
            let n_u = 2 * n_spans + 1;
            let mut out = NurbsSurface::new(3, true, 3, 2, n_u, 2);
            if !out.is_valid() {
                return None;
            }
            let mut knots = vec![u0, u0];
            for s in 1..n_spans {
                knots.push(u0 + s as f64);
                knots.push(u0 + s as f64);
            }
            knots.push(u1);
            knots.push(u1);
            out.m_nurbsknot[0] = knots;
            out.m_nurbsknot[1] = vec![v0, v1];
            let w = 2.0f64.sqrt() / 2.0;
            for i in 0..n_u {
                let (lx, ly, wi);
                if i % 2 == 0 {
                    let ang = (u0 + (i / 2) as f64) * (PI / 2.0);
                    lx = radius * ang.cos();
                    ly = radius * ang.sin();
                    wi = 1.0;
                } else {
                    let a0 = (u0 + (i / 2) as f64) * (PI / 2.0);
                    let a1 = (u0 + (i / 2) as f64 + 1.0) * (PI / 2.0);
                    lx = radius * (a0.cos() + a1.cos());
                    ly = radius * (a0.sin() + a1.sin());
                    wi = w;
                }
                for vi in 0..2 {
                    let h = if vi == 0 { v0 } else { v1 };
                    let px = a.origin[0] + lx * a.ax[0] + ly * a.ay[0] + h * a.az[0];
                    let py = a.origin[1] + lx * a.ax[1] + ly * a.ay[1] + h * a.az[1];
                    let pz = a.origin[2] + lx * a.ax[2] + ly * a.ay[2] + h * a.az[2];
                    out.set_cv_4d(i, vi, wi * px, wi * py, wi * pz, wi);
                }
            }
            return Some(out);
        }
        None
    }

    fn get_analytic_srf(&mut self, id: i32) -> AnFace {
        let mut an = AnFace {
            kind: 0,
            a: Axis2::new(),
            r: 0.0,
            r2: 0.0,
        };
        let Some(e) = self.get(id) else { return an };
        let kinds = [
            "CYLINDRICAL_SURFACE",
            "CONICAL_SURFACE",
            "SPHERICAL_SURFACE",
            "TOROIDAL_SURFACE",
        ];
        let kk = [2, 3, 4, 5];
        for i in 0..4 {
            let Some(sub) = e.find(kinds[i]) else {
                continue;
            };
            let mut ax_ref = -1;
            let mut nums = Vec::new();
            for p in &sub.params {
                if p.tag == StepTag::Ref && ax_ref < 0 {
                    ax_ref = p.ref_id;
                } else if p.tag == StepTag::Num {
                    nums.push(p.num);
                }
            }
            if ax_ref < 0 {
                return an;
            }
            an.a = self.get_axis2(ax_ref);
            if !an.a.ok {
                return an;
            }
            an.kind = kk[i];
            an.r = if nums.is_empty() { 0.0 } else { nums[0] };
            an.r2 = if nums.len() > 1 { nums[1] } else { 0.0 };
            return an;
        }
        an
    }

    fn an_local(&self, an: &AnFace, p: &Point) -> (f64, f64, f64) {
        let wx = p[0] - an.a.origin[0];
        let wy = p[1] - an.a.origin[1];
        let wz = p[2] - an.a.origin[2];
        (
            wx * an.a.ax[0] + wy * an.a.ax[1] + wz * an.a.ax[2],
            wx * an.a.ay[0] + wy * an.a.ay[1] + wz * an.a.ay[2],
            wx * an.a.az[0] + wy * an.a.az[1] + wz * an.a.az[2],
        )
    }

    fn an_st_of(&self, an: &AnFace, p: &Point) -> (f64, f64, bool) {
        let (x, y, z) = self.an_local(an, p);
        let rho = (x * x + y * y).sqrt();
        let radial_ok = rho > 1e-9;
        match an.kind {
            2 => (y.atan2(x), z, radial_ok),
            3 => {
                let ca = an.r2.cos();
                (
                    y.atan2(x),
                    if ca.abs() > 1e-12 { z / ca } else { z },
                    radial_ok,
                )
            }
            4 => (y.atan2(x), z.atan2(rho), radial_ok),
            _ => (y.atan2(x), z.atan2(rho - an.r), radial_ok),
        }
    }

    fn an_eval(&self, an: &AnFace, s: f64, t: f64) -> Point {
        let cs = s.cos();
        let sn = s.sin();
        match an.kind {
            2 => an_place(an, an.r * cs, an.r * sn, t),
            3 => {
                let r = an.r + t * an.r2.sin();
                an_place(an, r * cs, r * sn, t * an.r2.cos())
            }
            4 => {
                let ct = t.cos();
                let st = t.sin();
                an_place(an, an.r * ct * cs, an.r * ct * sn, an.r * st)
            }
            _ => {
                let ct = t.cos();
                let st = t.sin();
                let r = an.r + an.r2 * ct;
                an_place(an, r * cs, r * sn, an.r2 * st)
            }
        }
    }

    fn build_analytic_nurbs(
        &self,
        an: &AnFace,
        su0: i32,
        nsu: usize,
        t0: f64,
        t1: f64,
        sv0: i32,
        nsv: usize,
    ) -> NurbsSurface {
        let w = 2.0f64.sqrt() / 2.0;
        let nu = 2 * nsu + 1;
        let mut ca = vec![0.0; nu];
        let mut sa = vec![0.0; nu];
        let mut cw = vec![0.0; nu];
        for i in 0..nu {
            if i % 2 == 0 {
                let ang = (su0 + (i / 2) as i32) as f64 * PI_2;
                ca[i] = ang.cos();
                sa[i] = ang.sin();
                cw[i] = 1.0;
            } else {
                let a0 = (su0 + (i / 2) as i32) as f64 * PI_2;
                let a1 = (su0 + (i / 2) as i32 + 1) as f64 * PI_2;
                ca[i] = a0.cos() + a1.cos();
                sa[i] = a0.sin() + a1.sin();
                cw[i] = w;
            }
        }
        if an.kind == 2 || an.kind == 3 {
            let mut srf = NurbsSurface::new(3, true, 3, 2, nu, 2);
            if !srf.is_valid() {
                return NurbsSurface::default();
            }
            srf.m_nurbsknot[0] = quarter_knots(nsu);
            srf.m_nurbsknot[1] = vec![t0, t1];
            for j in 0..2 {
                let t = if j == 0 { t0 } else { t1 };
                let r = if an.kind == 2 {
                    an.r
                } else {
                    an.r + t * an.r2.sin()
                };
                let z = if an.kind == 2 { t } else { t * an.r2.cos() };
                for i in 0..nu {
                    let p = an_place(an, r * ca[i], r * sa[i], z);
                    srf.set_cv_4d(i, j, cw[i] * p[0], cw[i] * p[1], cw[i] * p[2], cw[i]);
                }
            }
            return srf;
        }
        let nv = 2 * nsv + 1;
        let mut cb = vec![0.0; nv];
        let mut sb = vec![0.0; nv];
        let mut vw = vec![0.0; nv];
        for j in 0..nv {
            if j % 2 == 0 {
                let ang = (sv0 + (j / 2) as i32) as f64 * PI_2;
                cb[j] = ang.cos();
                sb[j] = ang.sin();
                vw[j] = 1.0;
            } else {
                let a0 = (sv0 + (j / 2) as i32) as f64 * PI_2;
                let a1 = (sv0 + (j / 2) as i32 + 1) as f64 * PI_2;
                cb[j] = a0.cos() + a1.cos();
                sb[j] = a0.sin() + a1.sin();
                vw[j] = w;
            }
        }
        let mut srf = NurbsSurface::new(3, true, 3, 3, nu, nv);
        if !srf.is_valid() {
            return NurbsSurface::default();
        }
        srf.m_nurbsknot[0] = quarter_knots(nsu);
        srf.m_nurbsknot[1] = quarter_knots(nsv);
        for j in 0..nv {
            for i in 0..nu {
                let (r, z) = if an.kind == 4 {
                    (an.r * cb[j], an.r * sb[j])
                } else {
                    (an.r + an.r2 * cb[j], an.r2 * sb[j])
                };
                let p = an_place(an, r * ca[i], r * sa[i], z);
                let wij = cw[i] * vw[j];
                srf.set_cv_4d(i, j, wij * p[0], wij * p[1], wij * p[2], wij);
            }
        }
        srf
    }

    fn pcurve_st_samples(
        &mut self,
        ec_geom_id: i32,
        surface_ref: i32,
        forward_use: bool,
        n: usize,
    ) -> Vec<Point> {
        let Some(e) = self.get(ec_geom_id) else {
            return Vec::new();
        };
        let mut is_seam = false;
        let sc = match e.find("SURFACE_CURVE") {
            Some(sc) => Some(sc),
            None => {
                let s = e.find("SEAM_CURVE");
                is_seam = s.is_some();
                s
            }
        };
        let Some(sc) = sc else { return Vec::new() };
        let pc_refs = list_refs(&sc.params);
        let mut mine = Vec::new();
        for pid in pc_refs {
            let Some(pc) = self.get(pid).and_then(|pe| pe.find("PCURVE")) else {
                continue;
            };
            let refs = all_refs(&pc.params);
            if refs.len() < 2 || refs[0] != surface_ref {
                continue;
            }
            mine.push(refs[1]);
        }
        if mine.is_empty() {
            return Vec::new();
        }
        let pick = if is_seam && mine.len() > 1 && !forward_use {
            1
        } else {
            0
        };
        let Some(drs) = self
            .get(mine[pick])
            .and_then(|dr| dr.find("DEFINITIONAL_REPRESENTATION"))
        else {
            return Vec::new();
        };
        let mut c2_ref = -1;
        for p in &drs.params {
            if p.tag == StepTag::List {
                for v in &p.list {
                    if v.tag == StepTag::Ref {
                        c2_ref = v.ref_id;
                        break;
                    }
                }
            }
            if c2_ref >= 0 {
                break;
            }
        }
        if c2_ref < 0 {
            return Vec::new();
        }
        let c2 = self.get_nurbs_curve(c2_ref);
        if !c2.is_valid() || c2.cv_count() < c2.order() {
            return Vec::new();
        }
        let Some((tmin, tmax)) = curve_domain(&c2) else {
            return Vec::new();
        };
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let t = if n > 1 {
                tmin + (tmax - tmin) * i as f64 / (n - 1) as f64
            } else {
                tmin
            };
            let p = c2.point_at(t);
            out.push(Point::new(p[0], p[1], 0.0));
        }
        out
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BRep assembly from STEP
// ═══════════════════════════════════════════════════════════════════════════

struct PendingEdge {
    edge: usize,
    reversed: bool,
    c2d: NurbsCurve,
}

struct AEdge {
    edge_idx: usize,
    reversed: bool,
    st: Vec<Point>,
}

struct ALoop {
    is_outer: bool,
    projected: bool,
    edges: Vec<AEdge>,
}

struct LoopEdge {
    edge_idx: usize,
    reversed: bool,
    uv: Vec<Point>,
    pc2d: NurbsCurve,
    exact: bool,
}

struct Loop {
    is_outer: bool,
    edges: Vec<LoopEdge>,
}

fn pick_outer_loop_by(extents: &[(f64, f64, f64, f64)], is_outer: &mut [bool]) {
    if is_outer.iter().any(|o| *o) || is_outer.is_empty() {
        return;
    }
    let mut best = 0;
    let mut best_a = -1.0;
    for (i, (mnu, mxu, mnv, mxv)) in extents.iter().enumerate() {
        let a = if mxu > mnu && mxv > mnv {
            (mxu - mnu) * (mxv - mnv)
        } else {
            0.0
        };
        if a > best_a {
            best_a = a;
            best = i;
        }
    }
    is_outer[best] = true;
}

fn extent_of(pts: &[&Vec<Point>]) -> (f64, f64, f64, f64) {
    let mut mnu = 1e300f64;
    let mut mnv = 1e300f64;
    let mut mxu = -1e300f64;
    let mut mxv = -1e300f64;
    for v in pts {
        for q in v.iter() {
            mnu = mnu.min(q[0]);
            mxu = mxu.max(q[0]);
            mnv = mnv.min(q[1]);
            mxv = mxv.max(q[1]);
        }
    }
    (mnu, mxu, mnv, mxv)
}

struct BRepBuilder<'a, 'b> {
    r: &'b mut StepReader<'a>,
    sf: &'a StepFile,
    brep: BRep,
    vmap: HashMap<i32, usize>,
    emap: HashMap<i32, usize>,
    face_refs: Vec<BRepRef>,
}

impl<'a, 'b> BRepBuilder<'a, 'b> {
    fn new(reader: &'b mut StepReader<'a>, sf: &'a StepFile) -> Self {
        BRepBuilder {
            r: reader,
            sf,
            brep: BRep::new(),
            vmap: HashMap::new(),
            emap: HashMap::new(),
            face_refs: Vec::new(),
        }
    }

    fn vertex_at(&mut self, q: &Point, tol: f64) -> usize {
        for i in 0..self.brep.m_vertices.len() {
            if distance(&self.brep.m_vertices[i].point, q) <= tol {
                return i;
            }
        }
        self.brep.add_vertex(q, 0.0)
    }

    fn attach_pcurve(&mut self, edge: usize, si: usize, c2: usize, reversed_use: bool) {
        for pc in self.brep.m_edges[edge].pcurves.iter_mut() {
            if pc.surface_index == si as i32 {
                if reversed_use {
                    pc.curve_2d_index_2 = c2 as i32;
                } else {
                    pc.curve_2d_index_2 = pc.curve_2d_index;
                    pc.curve_2d_index = c2 as i32;
                }
                return;
            }
        }
        self.brep.add_pcurve(edge, si, c2 as i32, -1);
    }

    fn finish_face(&mut self, si: usize, reversed_face: bool, loops: &[Vec<PendingEdge>]) -> usize {
        let mut wires = Vec::new();
        for lp in loops {
            let mut refs = Vec::new();
            for pe in lp {
                let mut c = pe.c2d.duplicate();
                if pe.reversed {
                    c.reverse();
                }
                let c2 = self.brep.add_curve_2d(&c);
                self.attach_pcurve(pe.edge, si, c2, pe.reversed);
                refs.push(BRepRef::new(
                    pe.edge as i32,
                    if pe.reversed {
                        BRepOrientation::Reversed
                    } else {
                        BRepOrientation::Forward
                    },
                ));
            }
            if !refs.is_empty() {
                let wi = self.brep.add_wire(&refs);
                wires.push(BRepRef::new(wi as i32, BRepOrientation::Forward));
            }
        }
        let fi = self.brep.add_face(si as i32, &wires, 0.0);
        self.face_refs.push(BRepRef::new(
            fi as i32,
            if reversed_face {
                BRepOrientation::Reversed
            } else {
                BRepOrientation::Forward
            },
        ));
        fi
    }

    fn get_vertex(&mut self, vp_id: i32) -> usize {
        if let Some(v) = self.vmap.get(&vp_id) {
            return *v;
        }
        let sf = self.sf;
        let mut pt = Point::new(0.0, 0.0, 0.0);
        if let Some(sub) = sf.entities.get(&vp_id).and_then(|e| e.find("VERTEX_POINT")) {
            let r = first_ref(&sub.params);
            if r >= 0 {
                pt = self.r.get_point(r);
            }
        }
        let idx = self.brep.add_vertex(&pt, 0.0);
        self.vmap.insert(vp_id, idx);
        idx
    }

    fn edge_geom_id(&self, ec_ref: i32) -> i32 {
        let Some(ec) = self
            .sf
            .entities
            .get(&ec_ref)
            .and_then(|e| e.find("EDGE_CURVE"))
        else {
            return -1;
        };
        let rfs = all_refs(&ec.params);
        if rfs.len() >= 3 {
            rfs[2]
        } else {
            -1
        }
    }

    fn get_edge(&mut self, ec_id: i32) -> Option<usize> {
        if let Some(e) = self.emap.get(&ec_id) {
            return Some(*e);
        }
        let sf = self.sf;
        let ec = sf.entities.get(&ec_id)?.find("EDGE_CURVE")?;
        let refs = all_refs(&ec.params);
        if refs.len() < 3 {
            return None;
        }

        let sv = self.get_vertex(refs[0]);
        let ev = self.get_vertex(refs[1]);
        let curve_id = self.r.basis_curve_of(refs[2]);
        let vs = self.brep.m_vertices[sv].point.clone();
        let ve = self.brep.m_vertices[ev].point.clone();

        let mut crv3d = NurbsCurve::default();
        let mut got = false;
        if let Some(curve_ent) = sf.entities.get(&curve_id) {
            if curve_ent.has("B_SPLINE_CURVE_WITH_KNOTS") {
                crv3d = self.r.get_nurbs_curve(curve_id);
                got = crv3d.is_valid();
            } else if let Some(circ) = curve_ent.find("CIRCLE") {
                let mut ax_ref = -1;
                let mut rad = 0.0;
                for p in &circ.params {
                    if p.tag == StepTag::Ref && ax_ref < 0 {
                        ax_ref = p.ref_id;
                    } else if p.tag == StepTag::Num {
                        rad = p.num;
                    }
                }
                if ax_ref >= 0 && rad > 0.0 {
                    let a = self.r.get_axis2(ax_ref);
                    if a.ok {
                        let sa = angle_of(&a, &vs);
                        let mut ea = angle_of(&a, &ve);
                        let dx = ve[0] - vs[0];
                        let dy = ve[1] - vs[1];
                        let dz = ve[2] - vs[2];
                        let closed = dx * dx + dy * dy + dz * dz < 1e-20;
                        if closed {
                            ea = sa + 2.0 * PI;
                        } else if ea <= sa {
                            ea += 2.0 * PI;
                        }
                        let span = ea - sa;
                        let ns = ((span.abs() / (PI * 0.5)).ceil() as i64).max(1) as usize;
                        let n_cp = 2 * ns + 1;
                        let da = span / (2.0 * ns as f64);
                        let wm = da.cos();
                        crv3d = NurbsCurve::new(3, true, 3, n_cp);
                        let mut knots = vec![sa, sa];
                        for s in 1..ns {
                            let ak = sa + s as f64 * span / ns as f64;
                            knots.push(ak);
                            knots.push(ak);
                        }
                        knots.push(ea);
                        knots.push(ea);
                        crv3d.m_nurbsknot.copy_from_slice(&knots);
                        for i in 0..n_cp {
                            let mid = i % 2 == 1;
                            let (ang, w, r2) = if !mid {
                                (sa + (i / 2) as f64 * span / ns as f64, 1.0, rad)
                            } else {
                                (sa + ((i / 2) as f64 + 0.5) * span / ns as f64, wm, rad / wm)
                            };
                            let ca = ang.cos();
                            let sa2 = ang.sin();
                            let px = a.origin[0] + r2 * (ca * a.ax[0] + sa2 * a.ay[0]);
                            let py = a.origin[1] + r2 * (ca * a.ax[1] + sa2 * a.ay[1]);
                            let pz = a.origin[2] + r2 * (ca * a.ax[2] + sa2 * a.ay[2]);
                            crv3d.m_cv[i * 4] = w * px;
                            crv3d.m_cv[i * 4 + 1] = w * py;
                            crv3d.m_cv[i * 4 + 2] = w * pz;
                            crv3d.m_cv[i * 4 + 3] = w;
                        }
                        got = true;
                    }
                }
            }
        }
        if !got {
            let mut samples = self.r.sample_curve(curve_id, &vs, &ve, 16);
            if samples.len() < 2 {
                samples = vec![vs.clone(), ve.clone()];
            }
            crv3d = polyline_nurbs(&samples, 3);
        }

        let c3 = self.brep.add_curve_3d(&crv3d);
        let idx = self.brep.add_edge(c3 as i32, sv as i32, ev as i32);
        self.emap.insert(ec_id, idx);
        Some(idx)
    }

    fn bound_loop(&self, bid: i32) -> Option<(bool, bool, Vec<i32>)> {
        let bent = self.sf.entities.get(&bid)?;
        let is_outer = bent.has("FACE_OUTER_BOUND");
        if !is_outer && !bent.has("FACE_BOUND") {
            return None;
        }
        let bsub = bent.find(if is_outer {
            "FACE_OUTER_BOUND"
        } else {
            "FACE_BOUND"
        })?;
        let mut loop_ref = -1;
        let mut bound_orient = true;
        for p in &bsub.params {
            if p.tag == StepTag::Ref && loop_ref < 0 {
                loop_ref = p.ref_id;
            } else if p.tag == StepTag::Enum {
                bound_orient = p.str == "T";
            }
        }
        if loop_ref < 0 {
            return None;
        }
        let lp = self.sf.entities.get(&loop_ref)?.find("EDGE_LOOP")?;
        Some((is_outer, bound_orient, list_refs(&lp.params)))
    }

    fn oriented_edge(&self, oe_id: i32) -> Option<(i32, bool)> {
        let oe = self.sf.entities.get(&oe_id)?.find("ORIENTED_EDGE")?;
        let mut ec_ref = -1;
        let mut oe_orient = true;
        for p in &oe.params {
            if p.tag == StepTag::Ref {
                ec_ref = p.ref_id;
            } else if p.tag == StepTag::Enum {
                oe_orient = p.str == "T";
            }
        }
        if ec_ref < 0 {
            return None;
        }
        Some((ec_ref, oe_orient))
    }

    fn add_face_analytic(
        &mut self,
        bound_refs: &[i32],
        surface_ref: i32,
        same_sense: bool,
        an: &AnFace,
    ) -> bool {
        let period = 4.0;
        let mut loops: Vec<ALoop> = Vec::new();

        for &bid in bound_refs {
            let Some((is_outer, bound_orient, oe_refs)) = self.bound_loop(bid) else {
                continue;
            };
            let mut lp = ALoop {
                is_outer,
                projected: false,
                edges: Vec::new(),
            };
            for oe_id in oe_refs {
                let Some((ec_ref, oe_orient)) = self.oriented_edge(oe_id) else {
                    continue;
                };
                let Some(edge_idx) = self.get_edge(ec_ref) else {
                    continue;
                };
                let mut rev = !oe_orient;
                if !bound_orient {
                    rev = !rev;
                }

                let geom_id = self.edge_geom_id(ec_ref);
                let mut st = Vec::new();
                if geom_id >= 0 {
                    st = self
                        .r
                        .pcurve_st_samples(geom_id, surface_ref, oe_orient, 48);
                }
                if st.len() < 2 {
                    lp.projected = true;
                    let be = &self.brep.m_edges[edge_idx];
                    let vs = self.brep.m_vertices[be.start_vertex as usize].point.clone();
                    let ve = self.brep.m_vertices[be.end_vertex as usize].point.clone();
                    let samples = self.r.sample_curve(geom_id, &vs, &ve, 48);
                    if samples.len() < 2 {
                        return false;
                    }
                    let mut ps = 0.0;
                    let mut pt = 0.0;
                    let mut have_prev = false;
                    if let Some(pe) = lp.edges.last() {
                        if !pe.st.is_empty() {
                            let q = if pe.reversed {
                                &pe.st[0]
                            } else {
                                &pe.st[pe.st.len() - 1]
                            };
                            ps = q[0];
                            pt = q[1];
                            have_prev = true;
                        }
                    }
                    let mut ordered = samples;
                    if rev {
                        ordered.reverse();
                    }
                    if !have_prev {
                        for k in 0..ordered.len() {
                            let (s2, t2, ok2) = self.r.an_st_of(an, &ordered[k]);
                            if ok2 {
                                ps = s2;
                                pt = t2;
                                have_prev = true;
                                break;
                            }
                        }
                    }
                    let mut st_ord: Vec<Point> = Vec::with_capacity(ordered.len());
                    for k in 0..ordered.len() {
                        let (mut s, mut t, ok) = self.r.an_st_of(an, &ordered[k]);
                        if !ok && (k > 0 || have_prev) {
                            s = if k > 0 { st_ord[k - 1][0] } else { ps };
                        }
                        let rs = if k > 0 {
                            st_ord[k - 1][0]
                        } else if have_prev {
                            ps
                        } else {
                            s
                        };
                        s -= 2.0 * PI_2 * 2.0 * ((s - rs) / (2.0 * PI_2 * 2.0)).round();
                        if an.kind == 5 {
                            let rt = if k > 0 {
                                st_ord[k - 1][1]
                            } else if have_prev {
                                pt
                            } else {
                                t
                            };
                            t -= 2.0 * PI_2 * 2.0 * ((t - rt) / (2.0 * PI_2 * 2.0)).round();
                        }
                        st_ord.push(Point::new(s, t, 0.0));
                    }
                    if rev {
                        st_ord.reverse();
                    }
                    st = st_ord;
                }
                lp.edges.push(AEdge {
                    edge_idx,
                    reversed: rev,
                    st,
                });
            }
            if !lp.edges.is_empty() {
                loops.push(lp);
            }
            let extents: Vec<(f64, f64, f64, f64)> = loops
                .iter()
                .map(|l| extent_of(&l.edges.iter().map(|e| &e.st).collect::<Vec<_>>()))
                .collect();
            let mut outer: Vec<bool> = loops.iter().map(|l| l.is_outer).collect();
            pick_outer_loop_by(&extents, &mut outer);
            for (l, o) in loops.iter_mut().zip(outer) {
                l.is_outer = o;
            }
        }
        if loops.is_empty() {
            return false;
        }

        let mut smin = 1e300f64;
        let mut smax = -1e300f64;
        let mut tmin = 1e300f64;
        let mut tmax = -1e300f64;
        for lp in &loops {
            for ae in &lp.edges {
                for p in &ae.st {
                    smin = smin.min(p[0]);
                    smax = smax.max(p[0]);
                    tmin = tmin.min(p[1]);
                    tmax = tmax.max(p[1]);
                }
            }
        }
        if smin > smax {
            return false;
        }
        let su0 = (smin / PI_2 + 1e-9).floor() as i32;
        let su1 = (smax / PI_2 - 1e-9).ceil() as i32;
        let nsu = (su1 - su0).max(1) as usize;
        if nsu > 16 {
            return false;
        }
        let mut sv0 = 0i32;
        let mut nsv = 0usize;
        let t0 = tmin;
        let t1 = tmax;
        if an.kind == 4 || an.kind == 5 {
            sv0 = (tmin / PI_2 + 1e-9).floor() as i32;
            let mut sv1 = (tmax / PI_2 - 1e-9).ceil() as i32;
            if an.kind == 4 {
                sv0 = sv0.max(-1);
                sv1 = sv1.min(1);
            }
            nsv = (sv1 - sv0).max(1) as usize;
            if nsv > 16 {
                return false;
            }
        } else if t1 - t0 < 1e-12 {
            return false;
        }

        let srf = self.r.build_analytic_nurbs(an, su0, nsu, t0, t1, sv0, nsv);
        if !srf.is_valid() {
            return false;
        }
        let scale3 = an.r + an.r2.abs() + 1.0;
        let angular_v = an.kind == 4 || an.kind == 5;

        let srf_idx = self.brep.add_surface(&srf);
        loops.sort_by_key(|l| !l.is_outer);
        let mut pending: Vec<Vec<PendingEdge>> = Vec::new();

        for lp in &loops {
            let mut pl: Vec<PendingEdge> = Vec::new();
            let mut chains: Vec<Vec<Point>> = Vec::with_capacity(lp.edges.len());
            for ae in &lp.edges {
                let mut uv = Vec::with_capacity(ae.st.len());
                for p in &ae.st {
                    let u = chart_u_of_angle(p[0], su0);
                    let v = if angular_v {
                        chart_u_of_angle(p[1], sv0)
                    } else {
                        p[1]
                    };
                    uv.push(Point::new(u, v, 0.0));
                }
                if ae.reversed {
                    uv.reverse();
                }
                chains.push(uv);
            }
            for k in 1..chains.len() {
                if !lp.projected {
                    break;
                }
                if chains[k].is_empty() || chains[k - 1].is_empty() {
                    continue;
                }
                let du = chains[k - 1][chains[k - 1].len() - 1][0] - chains[k][0][0];
                let n = (du / period).round() as i32;
                if n != 0 {
                    for p in chains[k].iter_mut() {
                        p[0] += n as f64 * period;
                    }
                }
                if an.kind == 5 {
                    let dv = chains[k - 1][chains[k - 1].len() - 1][1] - chains[k][0][1];
                    let m = (dv / period).round() as i32;
                    if m != 0 {
                        for p in chains[k].iter_mut() {
                            p[1] += m as f64 * period;
                        }
                    }
                }
            }
            for k in 0..lp.edges.len() {
                let ae = &lp.edges[k];
                if chains[k].len() < 2 {
                    continue;
                }
                pl.push(PendingEdge {
                    edge: ae.edge_idx,
                    reversed: ae.reversed,
                    c2d: polyline_nurbs(&chains[k], 2),
                });
                let nxt = &chains[(k + 1) % chains.len()];
                if nxt.is_empty() {
                    continue;
                }
                let a2 = chains[k][chains[k].len() - 1].clone();
                let b2 = nxt[0].clone();
                let gap = (a2[0] - b2[0]).abs() + (a2[1] - b2[1]).abs();
                if gap > 1e-7 {
                    let ang_a = (su0 as f64 + a2[0]) * PI_2;
                    let ang_b = (su0 as f64 + b2[0]) * PI_2;
                    let angv_a = if angular_v {
                        (sv0 as f64 + a2[1]) * PI_2
                    } else {
                        a2[1]
                    };
                    let angv_b = if angular_v {
                        (sv0 as f64 + b2[1]) * PI_2
                    } else {
                        b2[1]
                    };
                    let p3a = self.r.an_eval(an, ang_a, angv_a);
                    let p3b = self.r.an_eval(an, ang_b, angv_b);
                    if distance(&p3a, &p3b) < scale3 * 1e-6 {
                        let vd = self.vertex_at(&p3a, scale3 * 1e-6);
                        let ei = self.brep.add_edge(-1, vd as i32, vd as i32);
                        pl.push(PendingEdge {
                            edge: ei,
                            reversed: false,
                            c2d: polyline_nurbs(&[a2, b2], 2),
                        });
                    }
                }
            }
            pending.push(pl);
        }
        self.finish_face(srf_idx, !same_sense, &pending);
        true
    }

    fn step_point_of(&mut self, vp_id: i32) -> Point {
        let sf = self.sf;
        if let Some(sub) = sf.entities.get(&vp_id).and_then(|e| e.find("VERTEX_POINT")) {
            let r = first_ref(&sub.params);
            if r >= 0 {
                return self.r.get_point(r);
            }
        }
        Point::new(1e300, 1e300, 1e300)
    }

    fn add_face_vertex_loop(
        &mut self,
        vl_vertex_ids: &[i32],
        surface_ref: i32,
        same_sense: bool,
    ) -> bool {
        let an = self.r.get_analytic_srf(surface_ref);
        let srf = match an.kind {
            4 => self.r.build_analytic_nurbs(&an, 0, 4, 0.0, 0.0, -1, 2),
            5 => self.r.build_analytic_nurbs(&an, 0, 4, 0.0, 0.0, 0, 4),
            0 => self.r.get_nurbs_surface(surface_ref),
            _ => NurbsSurface::default(),
        };
        if !srf.is_valid() {
            return false;
        }

        let du = srf.domain(0).unwrap_or((0.0, 1.0));
        let dv = srf.domain(1).unwrap_or((0.0, 1.0));
        const NS: usize = 17;
        let at = |i: usize, j: usize| -> Point {
            srf.point_at(
                du.0 + (du.1 - du.0) * i as f64 / (NS - 1) as f64,
                dv.0 + (dv.1 - dv.0) * j as f64 / (NS - 1) as f64,
            )
            .unwrap_or(Point::new(0.0, 0.0, 0.0))
        };
        let mut scale = 0.0f64;
        let p00 = at(0, 0);
        for i in 0..NS {
            for j in 0..NS {
                scale = scale.max(distance(&at(i, j), &p00));
            }
        }
        if scale.is_nan() || scale <= 0.0 {
            return false;
        }
        let tol = scale * 1e-7;
        let all_same = |along_u: bool, fixed: usize| -> bool {
            let p0 = if along_u { at(0, fixed) } else { at(fixed, 0) };
            for k in 1..NS {
                let q = if along_u { at(k, fixed) } else { at(fixed, k) };
                if distance(&q, &p0) > tol {
                    return false;
                }
            }
            true
        };
        let mut closed_u = true;
        let mut closed_v = true;
        for k in 0..NS {
            if distance(&at(0, k), &at(NS - 1, k)) > tol {
                closed_u = false;
            }
            if distance(&at(k, 0), &at(k, NS - 1)) > tol {
                closed_v = false;
            }
        }
        let degen_v0 = all_same(true, 0);
        let degen_v1 = all_same(true, NS - 1);
        if !closed_u {
            return false;
        }
        if !((degen_v0 && degen_v1) || closed_v) {
            return false;
        }
        let p_lo = at(0, 0);
        let p_hi = at(0, NS - 1);

        let uv_line = |u0: f64, v0: f64, u1: f64, v1: f64| -> NurbsCurve {
            NurbsCurve::create(
                false,
                1,
                &[Point::new(u0, v0, 0.0), Point::new(u1, v1, 0.0)],
            )
        };

        let si = self.brep.add_surface(&srf);
        let mut wire: Vec<PendingEdge> = Vec::new();

        if degen_v0 && degen_v1 {
            let v_lo = self.topo_vertex_at(vl_vertex_ids, &p_lo, tol);
            let v_hi = self.topo_vertex_at(vl_vertex_ids, &p_hi, tol);
            let Some(seam) = srf.iso_curve(1, du.0) else {
                return false;
            };
            if !seam.is_valid() {
                return false;
            }
            let c = self.brep.add_curve_3d(&seam);
            let ei_seam = self.brep.add_edge(c as i32, v_lo as i32, v_hi as i32);
            let ei_lo = self.brep.add_edge(-1, v_lo as i32, v_lo as i32);
            let ei_hi = self.brep.add_edge(-1, v_hi as i32, v_hi as i32);
            wire.push(PendingEdge {
                edge: ei_lo,
                reversed: false,
                c2d: uv_line(du.0, dv.0, du.1, dv.0),
            });
            wire.push(PendingEdge {
                edge: ei_seam,
                reversed: false,
                c2d: uv_line(du.1, dv.0, du.1, dv.1),
            });
            wire.push(PendingEdge {
                edge: ei_hi,
                reversed: false,
                c2d: uv_line(du.1, dv.1, du.0, dv.1),
            });
            wire.push(PendingEdge {
                edge: ei_seam,
                reversed: true,
                c2d: uv_line(du.0, dv.1, du.0, dv.0),
            });
        } else {
            let v0 = self.topo_vertex_at(vl_vertex_ids, &p_lo, tol);
            let Some(c_u) = srf.iso_curve(1, du.0) else {
                return false;
            };
            let Some(c_v) = srf.iso_curve(0, dv.0) else {
                return false;
            };
            if !c_u.is_valid() || !c_v.is_valid() {
                return false;
            }
            let cu = self.brep.add_curve_3d(&c_u);
            let cv = self.brep.add_curve_3d(&c_v);
            let ei_u = self.brep.add_edge(cu as i32, v0 as i32, v0 as i32);
            let ei_v = self.brep.add_edge(cv as i32, v0 as i32, v0 as i32);
            wire.push(PendingEdge {
                edge: ei_v,
                reversed: false,
                c2d: uv_line(du.0, dv.0, du.1, dv.0),
            });
            wire.push(PendingEdge {
                edge: ei_u,
                reversed: false,
                c2d: uv_line(du.1, dv.0, du.1, dv.1),
            });
            wire.push(PendingEdge {
                edge: ei_v,
                reversed: true,
                c2d: uv_line(du.1, dv.1, du.0, dv.1),
            });
            wire.push(PendingEdge {
                edge: ei_u,
                reversed: true,
                c2d: uv_line(du.0, dv.1, du.0, dv.0),
            });
        }
        self.finish_face(si, !same_sense, &[wire]);
        true
    }

    fn topo_vertex_at(&mut self, vl_vertex_ids: &[i32], q: &Point, tol: f64) -> usize {
        for &vid in vl_vertex_ids {
            if distance(&self.step_point_of(vid), q) <= tol {
                return self.get_vertex(vid);
            }
        }
        self.brep.add_vertex(q, 0.0)
    }

    fn uv_pts_of_sample(
        &self,
        proj: &Proj,
        proj_srf: Option<&NurbsSurface>,
        samples: &[Point],
    ) -> Vec<Point> {
        let mut uv = Vec::with_capacity(samples.len());
        if proj.kind == 0 {
            let Some(proj_srf) = proj_srf else {
                return vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)];
            };
            if samples.is_empty() {
                return vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)];
            }
            let (du0, du1) = proj_srf.domain(0).unwrap_or((0.0, 1.0));
            let (dv0, dv1) = proj_srf.domain(1).unwrap_or((0.0, 1.0));
            let wu = (du1 - du0) * 0.1;
            let wv = (dv1 - dv0) * 0.1;
            let mut d_ref = 0.0;
            let mut pu = 0.0;
            let mut pv = 0.0;
            for k in 0..samples.len() {
                let (u, v, d);
                if k == 0 {
                    let r = Closest::surface_point(proj_srf, &samples[k], 0.0, 0.0, 0.0, 0.0);
                    u = r.0;
                    v = r.1;
                    d = r.2;
                    d_ref = d;
                } else {
                    let mut r = Closest::surface_point(
                        proj_srf,
                        &samples[k],
                        pu - wu,
                        pu + wu,
                        pv - wv,
                        pv + wv,
                    );
                    if r.2 > 10.0 * d_ref + 1e-9 {
                        r = Closest::surface_point(proj_srf, &samples[k], 0.0, 0.0, 0.0, 0.0);
                    }
                    u = r.0;
                    v = r.1;
                }
                uv.push(Point::new(u, v, 0.0));
                pu = u;
                pv = v;
            }
            return uv;
        }
        for s in samples {
            let (u, v) = self.r.project(proj, s);
            uv.push(Point::new(u, v, 0.0));
        }
        uv
    }

    fn add_face(&mut self, face_id: i32) {
        let sf = self.sf;
        let Some(face) = sf
            .entities
            .get(&face_id)
            .and_then(|e| e.find("ADVANCED_FACE"))
        else {
            return;
        };

        let mut bound_refs = Vec::new();
        let mut surface_ref = -1;
        let mut same_sense = true;
        for p in &face.params {
            if p.tag == StepTag::List {
                for v in &p.list {
                    if v.tag == StepTag::Ref {
                        bound_refs.push(v.ref_id);
                    }
                }
            } else if p.tag == StepTag::Ref {
                surface_ref = p.ref_id;
            } else if p.tag == StepTag::Enum {
                same_sense = p.str == "T";
            }
        }

        if surface_ref >= 0 {
            let mut vl_verts = Vec::new();
            let mut any_edge_loop = false;
            for &bid in &bound_refs {
                let Some(bent) = sf.entities.get(&bid) else {
                    continue;
                };
                let Some(bsub) = bent
                    .find("FACE_OUTER_BOUND")
                    .or_else(|| bent.find("FACE_BOUND"))
                else {
                    continue;
                };
                let loop_ref = first_ref(&bsub.params);
                let Some(lent) = sf.entities.get(&loop_ref) else {
                    continue;
                };
                if lent.has("EDGE_LOOP") {
                    any_edge_loop = true;
                    continue;
                }
                let Some(vl) = lent.find("VERTEX_LOOP") else {
                    continue;
                };
                let r = first_ref(&vl.params);
                if r >= 0 {
                    vl_verts.push(r);
                }
            }
            if !any_edge_loop
                && !vl_verts.is_empty()
                && self.add_face_vertex_loop(&vl_verts, surface_ref, same_sense)
            {
                return;
            }
        }

        if surface_ref >= 0 {
            let an = self.r.get_analytic_srf(surface_ref);
            if an.kind >= 2 && self.add_face_analytic(&bound_refs, surface_ref, same_sense, &an) {
                return;
            }
        }

        let mut proj = Proj {
            kind: 0,
            a: Axis2::new(),
        };
        if surface_ref >= 0 {
            proj = self.r.get_projector(surface_ref);
        }

        let mut proj_srf: Option<NurbsSurface> = None;
        if proj.kind == 0 && surface_ref >= 0 {
            proj_srf = self
                .r
                .fill_surface(surface_ref, 0.0, 1.0, 0.0, 1.0)
                .filter(|s| s.is_valid());
        }
        let have_proj_srf = proj_srf.is_some();

        let mut loops: Vec<Loop> = Vec::new();
        for &bid in &bound_refs {
            let Some((is_outer, bound_orient, oe_refs)) = self.bound_loop(bid) else {
                continue;
            };
            let mut lp = Loop {
                is_outer,
                edges: Vec::new(),
            };
            for oe_id in oe_refs {
                let Some((ec_ref, oe_orient)) = self.oriented_edge(oe_id) else {
                    continue;
                };
                let Some(edge_idx) = self.get_edge(ec_ref) else {
                    continue;
                };
                let mut rev = !oe_orient;
                if !bound_orient {
                    rev = !rev;
                }

                let be = &self.brep.m_edges[edge_idx];
                let vs = self.brep.m_vertices[be.start_vertex as usize].point.clone();
                let ve = self.brep.m_vertices[be.end_vertex as usize].point.clone();
                let geom_id = self.edge_geom_id(ec_ref);
                let mut samples =
                    self.r
                        .sample_curve(geom_id, &vs, &ve, if have_proj_srf { 48 } else { 16 });
                if samples.is_empty() {
                    samples = vec![vs.clone(), ve.clone()];
                }
                let mut uv = self.uv_pts_of_sample(&proj, proj_srf.as_ref(), &samples);

                if proj.kind == 2 && uv.len() > 1 {
                    for k in 1..uv.len() {
                        let du = uv[k][0] - uv[k - 1][0];
                        if du > 2.0 {
                            uv[k] = Point::new(uv[k][0] - 4.0, uv[k][1], 0.0);
                        } else if du < -2.0 {
                            uv[k] = Point::new(uv[k][0] + 4.0, uv[k][1], 0.0);
                        }
                    }
                }
                let mut le2 = LoopEdge {
                    edge_idx,
                    reversed: rev,
                    uv,
                    pc2d: NurbsCurve::default(),
                    exact: false,
                };
                if proj.kind == 1 {
                    let be2 = &self.brep.m_edges[edge_idx];
                    if be2.curve_3d_index >= 0
                        && (be2.curve_3d_index as usize) < self.brep.m_curves_3d.len()
                    {
                        let c3 = &self.brep.m_curves_3d[be2.curve_3d_index as usize];
                        if c3.is_valid() && c3.cv_count() >= 2 {
                            let rat = c3.m_is_rat;
                            let mut p2 = NurbsCurve::new(3, rat, c3.order(), c3.cv_count());
                            p2.m_nurbsknot = c3.m_nurbsknot.clone();
                            let mut okcv = true;
                            let ost = p2.m_cv_stride;
                            for ci in 0..c3.cv_count() {
                                let base = ci * c3.m_cv_stride;
                                let wgt = if rat { c3.m_cv[base + 3] } else { 1.0 };
                                if rat && wgt.abs() < 1e-300 {
                                    okcv = false;
                                    break;
                                }
                                let e3 = Point::new(
                                    c3.m_cv[base] / wgt,
                                    c3.m_cv[base + 1] / wgt,
                                    c3.m_cv[base + 2] / wgt,
                                );
                                let (uu, vv) = self.r.project(&proj, &e3);
                                p2.m_cv[ci * ost] = uu * wgt;
                                p2.m_cv[ci * ost + 1] = vv * wgt;
                                p2.m_cv[ci * ost + 2] = 0.0;
                                if rat {
                                    p2.m_cv[ci * ost + 3] = wgt;
                                }
                            }
                            if okcv && p2.is_valid() {
                                le2.pc2d = p2;
                                le2.exact = true;
                            }
                        }
                    }
                } else if let Some(ps) = proj_srf
                    .as_ref()
                    .filter(|s| s.degree(0) == 1 && s.degree(1) == 1)
                {
                    let p00 = ps.get_cv(0, 0).unwrap_or(Point::new(0.0, 0.0, 0.0));
                    let p10 = ps.get_cv(1, 0).unwrap_or(Point::new(0.0, 0.0, 0.0));
                    let p01 = ps.get_cv(0, 1).unwrap_or(Point::new(0.0, 0.0, 0.0));
                    let eu = [p10[0] - p00[0], p10[1] - p00[1], p10[2] - p00[2]];
                    let ev = [p01[0] - p00[0], p01[1] - p00[1], p01[2] - p00[2]];
                    let eu2 = eu[0] * eu[0] + eu[1] * eu[1] + eu[2] * eu[2];
                    let ev2 = ev[0] * ev[0] + ev[1] * ev[1] + ev[2] * ev[2];
                    let be2 = &self.brep.m_edges[edge_idx];
                    if eu2 > 1e-28 && ev2 > 1e-28 && be2.curve_3d_index >= 0 {
                        let c3 = &self.brep.m_curves_3d[be2.curve_3d_index as usize];
                        if c3.is_valid() && c3.cv_count() >= 2 {
                            let mut p2 =
                                NurbsCurve::new(3, c3.is_rational(), c3.order(), c3.cv_count());
                            for k in 0..c3.nurbsknot_count() {
                                p2.set_nurbsknot(k, c3.nurbsknot(k).unwrap_or(0.0));
                            }
                            for ci in 0..c3.cv_count() {
                                let (wx, wy, wz, wgt) =
                                    c3.get_cv_4d(ci).unwrap_or((0.0, 0.0, 0.0, 1.0));
                                let dx = wx / wgt - p00[0];
                                let dy = wy / wgt - p00[1];
                                let dz = wz / wgt - p00[2];
                                let uu = (dx * eu[0] + dy * eu[1] + dz * eu[2]) / eu2;
                                let vv = (dx * ev[0] + dy * ev[1] + dz * ev[2]) / ev2;
                                if c3.is_rational() {
                                    p2.set_cv_4d(ci, uu * wgt, vv * wgt, 0.0, wgt);
                                } else {
                                    p2.set_cv(ci, &Point::new(uu, vv, 0.0));
                                }
                            }
                            if p2.is_valid() {
                                le2.pc2d = p2;
                                le2.exact = true;
                            }
                        }
                    }
                }
                lp.edges.push(le2);
            }
            if !lp.edges.is_empty() {
                loops.push(lp);
            }
            let extents: Vec<(f64, f64, f64, f64)> = loops
                .iter()
                .map(|l| extent_of(&l.edges.iter().map(|e| &e.uv).collect::<Vec<_>>()))
                .collect();
            let mut outer: Vec<bool> = loops.iter().map(|l| l.is_outer).collect();
            pick_outer_loop_by(&extents, &mut outer);
            for (l, o) in loops.iter_mut().zip(outer) {
                l.is_outer = o;
            }
        }

        let mut tau_u = 0.0;
        let mut tau_v = 0.0;
        if proj.kind == 2 {
            tau_u = 4.0;
        } else if let Some(ps) = proj_srf.as_ref() {
            let (du0, du1) = ps.domain(0).unwrap_or((0.0, 1.0));
            let (dv0, dv1) = ps.domain(1).unwrap_or((0.0, 1.0));
            let pt = |u: f64, v: f64| ps.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0));
            let scale = distance(&pt(du0, dv0), &pt(du1, dv1)) + 1e-9;
            let mut closed_u = true;
            let mut closed_v = true;
            for k in 0..=4 {
                let fu = du0 + (du1 - du0) * k as f64 / 4.0;
                let fv = dv0 + (dv1 - dv0) * k as f64 / 4.0;
                if distance(&pt(du0, fv), &pt(du1, fv)) > scale * 1e-6 {
                    closed_u = false;
                }
                if distance(&pt(fu, dv0), &pt(fu, dv1)) > scale * 1e-6 {
                    closed_v = false;
                }
            }
            if closed_u {
                tau_u = du1 - du0;
            }
            if closed_v {
                tau_v = dv1 - dv0;
            }
        }
        if tau_u > 0.0 || tau_v > 0.0 {
            let tau = tau_u;
            for lp in loops.iter_mut() {
                let mut prev_end = Point::new(0.0, 0.0, 0.0);
                let mut have_prev = false;
                for le in lp.edges.iter_mut() {
                    if le.uv.is_empty() {
                        continue;
                    }
                    let start_idx = if le.reversed { le.uv.len() - 1 } else { 0 };
                    let end_idx = if le.reversed { 0 } else { le.uv.len() - 1 };
                    if have_prev {
                        let st = le.uv[start_idx].clone();
                        let n = if tau_u > 0.0 {
                            ((prev_end[0] - st[0]) / tau_u).round() as i32
                        } else {
                            0
                        };
                        let m = if tau_v > 0.0 {
                            ((prev_end[1] - st[1]) / tau_v).round() as i32
                        } else {
                            0
                        };
                        if n != 0 || m != 0 {
                            for p in le.uv.iter_mut() {
                                *p = Point::new(
                                    p[0] + n as f64 * tau_u,
                                    p[1] + m as f64 * tau_v,
                                    0.0,
                                );
                            }
                        }
                    }
                    prev_end = le.uv[end_idx].clone();
                    have_prev = true;
                }
            }

            let mut outer_ucenter = 0.0;
            let mut have_outer = false;
            for lp in &loops {
                if !lp.is_outer {
                    continue;
                }
                let mut sum = 0.0;
                let mut cnt = 0;
                for le in &lp.edges {
                    for p in &le.uv {
                        sum += p[0];
                        cnt += 1;
                    }
                }
                if cnt > 0 {
                    outer_ucenter = sum / cnt as f64;
                    have_outer = true;
                }
                break;
            }
            if have_outer && tau > 0.0 {
                for lp in loops.iter_mut() {
                    if lp.is_outer {
                        continue;
                    }
                    let mut sum = 0.0;
                    let mut cnt = 0;
                    for le in &lp.edges {
                        for p in &le.uv {
                            sum += p[0];
                            cnt += 1;
                        }
                    }
                    if cnt == 0 {
                        continue;
                    }
                    let n = ((outer_ucenter - sum / cnt as f64) / tau).round() as i32;
                    if n != 0 {
                        for le in lp.edges.iter_mut() {
                            for p in le.uv.iter_mut() {
                                p[0] += n as f64 * tau;
                            }
                        }
                    }
                }
            }
        }

        let mut umin = 1e30f64;
        let mut umax = -1e30f64;
        let mut vmin = 1e30f64;
        let mut vmax = -1e30f64;
        for lp in &loops {
            for le in &lp.edges {
                for p in &le.uv {
                    umin = umin.min(p[0]);
                    umax = umax.max(p[0]);
                    vmin = vmin.min(p[1]);
                    vmax = vmax.max(p[1]);
                }
            }
        }
        if umin > umax {
            umin = -1.0;
            umax = 1.0;
            vmin = -1.0;
            vmax = 1.0;
        }

        let srf = match proj_srf {
            Some(s) => s,
            None => {
                if surface_ref >= 0 {
                    self.r
                        .fill_surface(surface_ref, umin, umax, vmin, vmax)
                        .unwrap_or_default()
                } else {
                    NurbsSurface::default()
                }
            }
        };
        let srf_idx = self.brep.add_surface(&srf);
        loops.sort_by_key(|l| !l.is_outer);
        let mut pending: Vec<Vec<PendingEdge>> = Vec::new();
        for lp in &loops {
            let mut pl = Vec::new();
            for le in &lp.edges {
                let crv2d = if le.exact {
                    let mut c = le.pc2d.duplicate();
                    if le.reversed {
                        c.reverse();
                    }
                    c
                } else {
                    let mut uv = le.uv.clone();
                    if le.reversed {
                        uv.reverse();
                    }
                    polyline_nurbs(&uv, 2)
                };
                pl.push(PendingEdge {
                    edge: le.edge_idx,
                    reversed: le.reversed,
                    c2d: crv2d,
                });
            }
            pending.push(pl);
        }
        self.finish_face(srf_idx, !same_sense, &pending);
    }

    fn build_from_shell(mut self, shell_id: i32) -> BRep {
        let sf = self.sf;
        let Some(sent) = sf.entities.get(&shell_id) else {
            return BRep::new();
        };
        if !sent.has("CLOSED_SHELL") && !sent.has("OPEN_SHELL") {
            return BRep::new();
        }
        let shell_sub = sent
            .find("CLOSED_SHELL")
            .or_else(|| sent.find("OPEN_SHELL"))
            .unwrap();
        let step_faces = list_refs(&shell_sub.params);
        self.brep.name = "step_brep".to_string();
        for f in step_faces {
            self.add_face(f);
        }
        if !self.face_refs.is_empty() {
            let sh = self.brep.add_shell(&self.face_refs);
            if sent.has("CLOSED_SHELL") {
                self.brep
                    .add_solid(&[BRepRef::new(sh as i32, BRepOrientation::Forward)]);
            }
        }
        self.brep
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// StepWriter
// ═══════════════════════════════════════════════════════════════════════════

/// printf("%.15g"): shortest of fixed/exponential at 15 significant digits, trailing zeros stripped.
fn fmt_g15(v: f64) -> String {
    if v == 0.0 {
        return "0".to_string();
    }
    let sci = format!("{:.14e}", v);
    let epos = sci.find('e').unwrap();
    let exp: i32 = sci[epos + 1..].parse().unwrap_or(0);
    if !(-4..15).contains(&exp) {
        let mant = sci[..epos]
            .trim_end_matches('0')
            .trim_end_matches('.')
            .to_string();
        return format!(
            "{}e{}{:02}",
            mant,
            if exp < 0 { "-" } else { "+" },
            exp.abs()
        );
    }
    let decimals = (14 - exp).max(0) as usize;
    let fixed = format!("{:.*}", decimals, v);
    if fixed.contains('.') {
        fixed
            .trim_end_matches('0')
            .trim_end_matches('.')
            .to_string()
    } else {
        fixed
    }
}

fn fmt(v: f64) -> String {
    let mut s = if v == (v as i64) as f64 && v.abs() < 1e15 {
        format!("{}.", v as i64)
    } else {
        fmt_g15(v)
    };
    if let Some(e) = s.find('e') {
        s.replace_range(e..e + 1, "E");
        if !s[..e].contains('.') {
            s.insert(e, '.');
        }
    } else if !s.contains('.') {
        s.push('.');
    }
    s
}

fn fmt_int_list(items: &[i32]) -> String {
    format!(
        "({})",
        items
            .iter()
            .map(|i| i.to_string())
            .collect::<Vec<_>>()
            .join(",")
    )
}

fn fmt_dbl_list(items: &[f64]) -> String {
    format!(
        "({})",
        items.iter().map(|v| fmt(*v)).collect::<Vec<_>>().join(",")
    )
}

fn fmt_ref_list(ids: &[i32]) -> String {
    format!(
        "({})",
        ids.iter()
            .map(|i| format!("#{}", i))
            .collect::<Vec<_>>()
            .join(",")
    )
}

struct StepWriter {
    next_id: i32,
    lines: Vec<String>,
    ctx2d_cache: i32,
}

impl StepWriter {
    fn new() -> Self {
        StepWriter {
            next_id: 1,
            lines: Vec::new(),
            ctx2d_cache: -1,
        }
    }

    fn new_id(&mut self) -> i32 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    fn write_raw(&mut self, body: &str) -> i32 {
        let id = self.new_id();
        self.lines.push(format!("#{}={};", id, body));
        id
    }

    fn write_point(&mut self, x: f64, y: f64, z: f64) -> i32 {
        self.write_raw(&format!(
            "CARTESIAN_POINT('',({},{},{}))",
            fmt(x),
            fmt(y),
            fmt(z)
        ))
    }

    fn write_point_2d(&mut self, u: f64, v: f64) -> i32 {
        self.write_raw(&format!("CARTESIAN_POINT('',({},{}))", fmt(u), fmt(v)))
    }

    fn write_nurbs_curve(&mut self, nc: &NurbsCurve, as_2d: bool) -> i32 {
        if !nc.is_valid() || nc.cv_count() < nc.order() {
            return -1;
        }
        let mut pt_ids = Vec::new();
        let stride = nc.m_cv_stride;
        let is_rat = nc.m_is_rat;
        let mut weights = Vec::new();
        let cdim = nc.m_dim;
        for i in 0..nc.cv_count() {
            let (x, y, z);
            if is_rat {
                let mut w = nc.m_cv[i * stride + cdim];
                if w.abs() < 1e-14 {
                    w = 1.0;
                }
                x = nc.m_cv[i * stride] / w;
                y = nc.m_cv[i * stride + 1] / w;
                z = if cdim > 2 {
                    nc.m_cv[i * stride + 2] / w
                } else {
                    0.0
                };
                weights.push(w);
            } else {
                x = nc.m_cv[i * stride];
                y = nc.m_cv[i * stride + 1];
                z = if cdim > 2 {
                    nc.m_cv[i * stride + 2]
                } else {
                    0.0
                };
            }
            pt_ids.push(if as_2d {
                self.write_point_2d(x, y)
            } else {
                self.write_point(x, y, z)
            });
        }

        let full = full_from_internal(&nc.m_nurbsknot);
        let (kvals, kmults) = compress_knots(&full);
        let degree = nc.m_order - 1;

        if !is_rat {
            return self.write_raw(&format!(
                "B_SPLINE_CURVE_WITH_KNOTS('',{},{},.UNSPECIFIED.,.F.,.U.,{},{},.UNSPECIFIED.)",
                degree,
                fmt_ref_list(&pt_ids),
                fmt_int_list(&kmults),
                fmt_dbl_list(&kvals)
            ));
        }
        self.write_raw(&format!(
            "(BOUNDED_CURVE()B_SPLINE_CURVE({},{},.UNSPECIFIED.,.F.,.U.)B_SPLINE_CURVE_WITH_KNOTS({},{},.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE({})REPRESENTATION_ITEM(''))",
            degree,
            fmt_ref_list(&pt_ids),
            fmt_int_list(&kmults),
            fmt_dbl_list(&kvals),
            fmt_dbl_list(&weights)
        ))
    }

    fn write_nurbs_surface(&mut self, srf: &NurbsSurface) -> i32 {
        if !srf.is_valid() {
            return -1;
        }
        let cv_u = srf.m_cv_count[0];
        let cv_v = srf.m_cv_count[1];
        let is_rat = srf.m_is_rat;
        let mut pt_ids = vec![vec![0i32; cv_v]; cv_u];
        let mut weight_grid = vec![vec![1.0f64; cv_v]; cv_u];
        for u in 0..cv_u {
            for v in 0..cv_v {
                let (mut x, mut y, mut z);
                if is_rat {
                    let (xx, yy, zz, w) = srf.get_cv_4d(u, v).unwrap_or((0.0, 0.0, 0.0, 1.0));
                    x = xx;
                    y = yy;
                    z = zz;
                    if w.abs() > 1e-14 {
                        x /= w;
                        y /= w;
                        z /= w;
                    }
                    weight_grid[u][v] = w;
                } else {
                    let pt = srf.get_cv(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0));
                    x = pt[0];
                    y = pt[1];
                    z = pt[2];
                }
                pt_ids[u][v] = self.write_point(x, y, z);
            }
        }

        let full_u = full_from_internal(&srf.m_nurbsknot[0]);
        let full_v = full_from_internal(&srf.m_nurbsknot[1]);
        let (ku_vals, ku_mults) = compress_knots(&full_u);
        let (kv_vals, kv_mults) = compress_knots(&full_v);
        let u_deg = srf.m_order[0] - 1;
        let v_deg = srf.m_order[1] - 1;

        let cpts = format!(
            "({})",
            pt_ids
                .iter()
                .map(|row| format!(
                    "({})",
                    row.iter()
                        .map(|id| format!("#{}", id))
                        .collect::<Vec<_>>()
                        .join(",")
                ))
                .collect::<Vec<_>>()
                .join(",")
        );

        if !is_rat {
            return self.write_raw(&format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',{},{},{},.UNSPECIFIED.,.F.,.F.,.U.,{},{},{},{},.UNSPECIFIED.)",
                u_deg,
                v_deg,
                cpts,
                fmt_int_list(&ku_mults),
                fmt_int_list(&kv_mults),
                fmt_dbl_list(&ku_vals),
                fmt_dbl_list(&kv_vals)
            ));
        }
        let wgrid = format!(
            "({})",
            weight_grid
                .iter()
                .map(|row| format!(
                    "({})",
                    row.iter().map(|w| fmt(*w)).collect::<Vec<_>>().join(",")
                ))
                .collect::<Vec<_>>()
                .join(",")
        );
        self.write_raw(&format!(
            "(BOUNDED_SURFACE()B_SPLINE_SURFACE({},{},{},.UNSPECIFIED.,.F.,.F.,.U.)B_SPLINE_SURFACE_WITH_KNOTS({},{},{},{},.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE({})REPRESENTATION_ITEM('')SURFACE())",
            u_deg,
            v_deg,
            cpts,
            fmt_int_list(&ku_mults),
            fmt_int_list(&kv_mults),
            fmt_dbl_list(&ku_vals),
            fmt_dbl_list(&kv_vals),
            wgrid
        ))
    }

    fn ctx2d(&mut self) -> i32 {
        if self.ctx2d_cache < 0 {
            self.ctx2d_cache = self.write_raw("(GEOMETRIC_REPRESENTATION_CONTEXT(2)PARAMETRIC_REPRESENTATION_CONTEXT()REPRESENTATION_CONTEXT('2D SPACE',''))");
        }
        self.ctx2d_cache
    }

    /// Kept for parity with the C++ writer, which exposes it for callers outside this file.
    #[allow(dead_code)]
    fn write_pcurve(&mut self, srf_id: i32, uv: &NurbsCurve) -> i32 {
        let c2 = self.write_nurbs_curve(uv, true);
        if c2 < 0 || srf_id < 0 {
            return -1;
        }
        let ctx = self.ctx2d();
        let dr = self.write_raw(&format!(
            "DEFINITIONAL_REPRESENTATION('',(#{}),#{})",
            c2, ctx
        ));
        self.write_raw(&format!("PCURVE('',#{},#{})", srf_id, dr))
    }

    fn write_loop_as_face_bound(
        &mut self,
        trimmed: &NurbsSurfaceTrimmed,
        loop_2d: &NurbsCurve,
        is_outer: bool,
    ) -> i32 {
        let Some((tmin, tmax)) = curve_domain(loop_2d) else {
            return -1;
        };
        if loop_2d.cv_count() < loop_2d.order() {
            return -1;
        }
        let n_samples = (loop_2d.cv_count() * 2).max(2);
        let mut pts3d = Vec::with_capacity(n_samples);
        for i in 0..n_samples {
            let t = if n_samples > 1 {
                tmin + (tmax - tmin) * i as f64 / (n_samples - 1) as f64
            } else {
                tmin
            };
            let uv = loop_2d.point_at(t);
            pts3d.push(
                trimmed
                    .m_surface
                    .point_at(uv[0], uv[1])
                    .unwrap_or(Point::new(0.0, 0.0, 0.0)),
            );
        }
        if pts3d.is_empty() {
            return -1;
        }
        let v0_pt = self.write_point(pts3d[0][0], pts3d[0][1], pts3d[0][2]);
        let v0 = self.write_raw(&format!("VERTEX_POINT('',#{})", v0_pt));
        let mut sample_pt_ids = Vec::with_capacity(n_samples);
        for p in &pts3d {
            sample_pt_ids.push(self.write_point(p[0], p[1], p[2]));
        }
        let kv: Vec<f64> = (0..n_samples).map(|i| i as f64).collect();
        let crv3d_id = self.write_raw(&format!(
            "B_SPLINE_CURVE_WITH_KNOTS('',1,{},.POLYLINE_FORM.,.T.,.U.,{},{},.UNSPECIFIED.)",
            fmt_ref_list(&sample_pt_ids),
            fmt_int_list(&vec![1; n_samples]),
            fmt_dbl_list(&kv)
        ));
        self.write_nurbs_curve(loop_2d, false);
        let ec_id = self.write_raw(&format!("EDGE_CURVE('',#{},#{},#{},.T.)", v0, v0, crv3d_id));
        let oe_id = self.write_raw(&format!("ORIENTED_EDGE('',*,*,#{},.T.)", ec_id));
        let el_id = self.write_raw(&format!("EDGE_LOOP('',(#{}))", oe_id));
        let fb_type = if is_outer {
            "FACE_OUTER_BOUND"
        } else {
            "FACE_BOUND"
        };
        self.write_raw(&format!("{}('',#{},.T.)", fb_type, el_id))
    }

    fn write_trimmed_face(&mut self, trimmed: &NurbsSurfaceTrimmed) -> i32 {
        let srf_id = self.write_nurbs_surface(&trimmed.m_surface);
        if srf_id < 0 {
            return -1;
        }
        let Some(outer) = trimmed.m_outer_loop.as_ref() else {
            return -1;
        };
        let outer_bound = self.write_loop_as_face_bound(trimmed, outer, true);
        if outer_bound < 0 {
            return -1;
        }
        let mut bounds = vec![outer_bound];
        for inner in &trimmed.m_inner_loops {
            let ib = self.write_loop_as_face_bound(trimmed, inner, false);
            if ib >= 0 {
                bounds.push(ib);
            }
        }
        self.write_raw(&format!(
            "ADVANCED_FACE('',{},#{},.T.)",
            fmt_ref_list(&bounds),
            srf_id
        ))
    }

    fn color_style(&mut self, r: f64, g: f64, b: f64) -> i32 {
        let c = self.write_raw(&format!("COLOUR_RGB('',{},{},{})", fmt(r), fmt(g), fmt(b)));
        let fc = self.write_raw(&format!("FILL_AREA_STYLE_COLOUR('',#{})", c));
        let fa = self.write_raw(&format!("FILL_AREA_STYLE('',(#{}))", fc));
        let sf = self.write_raw(&format!("SURFACE_STYLE_FILL_AREA(#{})", fa));
        let ss = self.write_raw(&format!("SURFACE_SIDE_STYLE('',(#{}))", sf));
        let su = self.write_raw(&format!("SURFACE_STYLE_USAGE(.BOTH.,#{})", ss));
        self.write_raw(&format!("PRESENTATION_STYLE_ASSIGNMENT((#{}))", su))
    }

    fn make_bodies(&mut self, shells: &[Vec<i32>], closed: bool) -> Vec<i32> {
        let mut bodies = Vec::new();
        for sf in shells {
            if sf.is_empty() {
                continue;
            }
            let shell = self.write_raw(&format!(
                "{}('',{})",
                if closed { "CLOSED_SHELL" } else { "OPEN_SHELL" },
                fmt_ref_list(sf)
            ));
            if closed {
                bodies.push(self.write_raw(&format!("MANIFOLD_SOLID_BREP('',#{})", shell)));
            } else {
                bodies.push(self.write_raw(&format!("SHELL_BASED_SURFACE_MODEL('',(#{}))", shell)));
            }
        }
        bodies
    }

    fn finish_shape(&mut self, shell_face_ids: &[i32], closed: bool, name: &str, uncertainty: f64) {
        let bodies = self.make_bodies(&[shell_face_ids.to_vec()], closed);
        self.finish_product(&bodies, closed, name, uncertainty, &[]);
    }

    fn finish_product(
        &mut self,
        bodies: &[i32],
        closed: bool,
        name: &str,
        uncertainty: f64,
        styled_items: &[i32],
    ) {
        let o = self.write_point(0.0, 0.0, 0.0);
        let dz = self.write_raw("DIRECTION('',(0.,0.,1.))");
        let dx = self.write_raw("DIRECTION('',(1.,0.,0.))");
        let ax = self.write_raw(&format!("AXIS2_PLACEMENT_3D('',#{},#{},#{})", o, dz, dx));
        let lu = self.write_raw("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))");
        let au = self.write_raw("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
        let su = self.write_raw("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");
        let uncertainty = if !uncertainty.is_finite() || uncertainty <= 0.0 {
            1e-6
        } else {
            uncertainty
        };
        let un = self.write_raw(&format!(
            "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{},'distance_accuracy_value','')",
            fmt(uncertainty),
            lu
        ));
        let gc = self.write_raw(&format!(
            "(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{}))GLOBAL_UNIT_ASSIGNED_CONTEXT((#{},#{},#{}))REPRESENTATION_CONTEXT('',''))",
            un, lu, au, su
        ));
        let ac = self.write_raw(
            "APPLICATION_CONTEXT('core data for automotive mechanical design processes')",
        );
        self.write_raw(&format!("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{})", ac));
        let pc = self.write_raw(&format!("PRODUCT_CONTEXT('',#{},'mechanical')", ac));
        let pr = self.write_raw(&format!("PRODUCT('{}','{}','',(#{}))", name, name, pc));
        let pf = self.write_raw(&format!("PRODUCT_DEFINITION_FORMATION('','',#{})", pr));
        let dc = self.write_raw(&format!(
            "PRODUCT_DEFINITION_CONTEXT('part definition',#{},'design')",
            ac
        ));
        let pd = self.write_raw(&format!("PRODUCT_DEFINITION('design','',#{},#{})", pf, dc));
        let ps = self.write_raw(&format!("PRODUCT_DEFINITION_SHAPE('','',#{})", pd));
        let rep_type = if bodies.is_empty() {
            "SHAPE_REPRESENTATION"
        } else if closed {
            "ADVANCED_BREP_SHAPE_REPRESENTATION"
        } else {
            "MANIFOLD_SURFACE_SHAPE_REPRESENTATION"
        };
        let mut items = vec![ax];
        items.extend_from_slice(bodies);
        let rp = self.write_raw(&format!(
            "{}('{}',{},#{})",
            rep_type,
            name,
            fmt_ref_list(&items),
            gc
        ));
        self.write_raw(&format!("SHAPE_DEFINITION_REPRESENTATION(#{},#{})", ps, rp));
        if !styled_items.is_empty() {
            self.write_raw(&format!(
                "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',{},#{})",
                fmt_ref_list(styled_items),
                gc
            ));
        }
    }

    fn emit(&self) -> String {
        let mut out = String::from("ISO-10303-21;\nHEADER;\n");
        out += "FILE_DESCRIPTION((''),'2;1');\n";
        out += "FILE_NAME('','',(''),(''),'','','');\n";
        out += "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\n";
        out += "ENDSEC;\nDATA;\n";
        for l in &self.lines {
            out += l;
            out += "\n";
        }
        out += "ENDSEC;\nEND-ISO-10303-21;\n";
        out
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════════

pub fn read_file_step_points(filepath: &str) -> Vec<Point> {
    let sf = parse_step_file(filepath);
    let mut r = StepReader::new(&sf);
    let mut out = Vec::new();
    for id in sf.ids_of_type("CARTESIAN_POINT") {
        out.push(r.get_point(id));
    }
    out
}

pub fn read_file_step_nurbscurves(filepath: &str) -> Vec<NurbsCurve> {
    let sf = parse_step_file(filepath);
    let mut r = StepReader::new(&sf);
    let mut out = Vec::new();
    for id in sf.ids_of_type("B_SPLINE_CURVE_WITH_KNOTS") {
        let nc = r.get_nurbs_curve(id);
        if nc.is_valid() {
            out.push(nc);
        }
    }
    out
}

pub fn read_file_step_nurbssurfaces(filepath: &str) -> Vec<NurbsSurface> {
    let sf = parse_step_file(filepath);
    let mut r = StepReader::new(&sf);
    let mut out = Vec::new();
    for id in sf.ids_of_type("B_SPLINE_SURFACE_WITH_KNOTS") {
        let srf = r.get_nurbs_surface(id);
        if srf.is_valid() {
            out.push(srf);
        }
    }
    out
}

pub fn read_file_step_nurbssurfaces_trimmed(filepath: &str) -> Vec<NurbsSurfaceTrimmed> {
    let sf = parse_step_file(filepath);
    let mut r = StepReader::new(&sf);
    let mut out = Vec::new();
    for face_id in sf.ids_of_type("ADVANCED_FACE") {
        let Some(face) = sf
            .entities
            .get(&face_id)
            .and_then(|e| e.find("ADVANCED_FACE"))
        else {
            continue;
        };
        let mut surface_ref = -1;
        let mut bound_refs = Vec::new();
        for p in &face.params {
            if p.tag == StepTag::List {
                for v in &p.list {
                    if v.tag == StepTag::Ref {
                        bound_refs.push(v.ref_id);
                    }
                }
            } else if p.tag == StepTag::Ref {
                surface_ref = p.ref_id;
            }
        }
        if surface_ref < 0 {
            continue;
        }
        let Some(surf_ent) = sf.entities.get(&surface_ref) else {
            continue;
        };
        if !surf_ent.has("B_SPLINE_SURFACE_WITH_KNOTS") {
            continue;
        }
        let srf = r.get_nurbs_surface(surface_ref);
        if !srf.is_valid() {
            continue;
        }

        let mut outer_loop = NurbsCurve::default();
        let mut got_outer = false;
        for bid in bound_refs {
            let Some(bent) = sf.entities.get(&bid) else {
                continue;
            };
            let is_outer = bent.has("FACE_OUTER_BOUND");
            let Some(bsub) = (if is_outer {
                bent.find("FACE_OUTER_BOUND")
            } else {
                bent.find("FACE_BOUND")
            }) else {
                continue;
            };
            let loop_ref = first_ref(&bsub.params);
            if loop_ref < 0 {
                continue;
            }
            let Some(lsub) = sf.entities.get(&loop_ref).and_then(|l| l.find("EDGE_LOOP")) else {
                continue;
            };
            let oe_refs = list_refs(&lsub.params);
            let mut uv_pts = Vec::new();
            for oe_id in oe_refs {
                let Some(oe) = sf
                    .entities
                    .get(&oe_id)
                    .and_then(|o| o.find("ORIENTED_EDGE"))
                else {
                    continue;
                };
                let mut ec_ref = -1;
                for p in &oe.params {
                    if p.tag == StepTag::Ref {
                        ec_ref = p.ref_id;
                    }
                }
                if ec_ref < 0 {
                    continue;
                }
                let Some(ec) = sf.entities.get(&ec_ref).and_then(|e| e.find("EDGE_CURVE")) else {
                    continue;
                };
                let ec_refs = all_refs(&ec.params);
                if ec_refs.len() < 3 {
                    continue;
                }
                let vs = r.get_point(ec_refs[0]);
                let ve = r.get_point(ec_refs[1]);
                let samples = r.sample_curve(ec_refs[2], &vs, &ve, 8);
                for s in samples {
                    uv_pts.push(Point::new(s[0], s[1], 0.0));
                }
            }
            if uv_pts.len() >= 2 {
                outer_loop = polyline_nurbs(&uv_pts, 2);
                got_outer = true;
            }
            break;
        }
        if !got_outer {
            continue;
        }
        let mut nst = NurbsSurfaceTrimmed::new();
        nst.m_surface = srf;
        nst.m_outer_loop = Some(outer_loop);
        out.push(nst);
    }
    out
}

pub fn read_file_step_breps(filepath: &str) -> Vec<BRep> {
    let sf = parse_step_file(filepath);
    let mut r = StepReader::new(&sf);
    let mut out = Vec::new();
    let mut roots: Vec<(i32, i32)> = Vec::new();
    let mut ids: Vec<i32> = sf.entities.keys().copied().collect();
    ids.sort();
    for id in ids {
        let e = &sf.entities[&id];
        if e.has("MANIFOLD_SOLID_BREP") || e.has("BREP_WITH_VOIDS") {
            let s = e
                .find("MANIFOLD_SOLID_BREP")
                .or_else(|| e.find("BREP_WITH_VOIDS"))
                .unwrap();
            for sh in all_refs(&s.params).into_iter().chain(list_refs(&s.params)) {
                roots.push((id, sh));
            }
        } else if let Some(s) = e.find("SHELL_BASED_SURFACE_MODEL") {
            for sh in all_refs(&s.params).into_iter().chain(list_refs(&s.params)) {
                roots.push((id, sh));
            }
        }
    }
    for (_, shell_ref0) in roots {
        let mut shell_ref = shell_ref0;
        if let Some(os) = sf
            .entities
            .get(&shell_ref)
            .and_then(|sh| sh.find("ORIENTED_CLOSED_SHELL"))
        {
            let rr = first_ref(&os.params);
            if rr >= 0 {
                shell_ref = rr;
            }
        }
        if shell_ref < 0 {
            continue;
        }
        let builder = BRepBuilder::new(&mut r, &sf);
        let b = builder.build_from_shell(shell_ref);
        if !b.m_faces.is_empty() {
            out.push(b);
        }
    }
    out
}

fn write_step_string(content: &str, filepath: &str) {
    let _ = std::fs::write(filepath, content);
}

pub fn write_file_step_nurbscurves(curves: &[NurbsCurve], filepath: &str) {
    let mut w = StepWriter::new();
    for nc in curves {
        w.write_nurbs_curve(nc, false);
    }
    write_step_string(&w.emit(), filepath);
}

pub fn write_file_step_nurbssurfaces(surfaces: &[NurbsSurface], filepath: &str) {
    let mut w = StepWriter::new();
    for srf in surfaces {
        w.write_nurbs_surface(srf);
    }
    write_step_string(&w.emit(), filepath);
}

pub fn write_file_step_nurbssurfaces_trimmed(trimmed: &[NurbsSurfaceTrimmed], filepath: &str) {
    let mut w = StepWriter::new();
    let mut face_ids = Vec::new();
    for t in trimmed {
        let fid = w.write_trimmed_face(t);
        if fid >= 0 {
            face_ids.push(fid);
        }
    }
    w.finish_shape(&face_ids, false, "trimmed", 1e-6);
    write_step_string(&w.emit(), filepath);
}

fn emit_brep_shells(w: &mut StepWriter, brep: &BRep) -> (Vec<(Vec<i32>, bool)>, f64) {
    let mut vid: HashMap<usize, i32> = HashMap::new();
    let mut eid: HashMap<usize, i32> = HashMap::new();
    let mut sid: HashMap<i32, i32> = HashMap::new();

    fn vertex_id(w: &mut StepWriter, brep: &BRep, vid: &mut HashMap<usize, i32>, vi: usize) -> i32 {
        if let Some(id) = vid.get(&vi) {
            return *id;
        }
        let p = &brep.m_vertices[vi].point;
        let pt = w.write_point(p[0], p[1], p[2]);
        let id = w.write_raw(&format!("VERTEX_POINT('',#{})", pt));
        vid.insert(vi, id);
        id
    }

    fn edge_id(
        w: &mut StepWriter,
        brep: &BRep,
        vid: &mut HashMap<usize, i32>,
        eid: &mut HashMap<usize, i32>,
        ei: usize,
    ) -> i32 {
        if let Some(id) = eid.get(&ei) {
            return *id;
        }
        let e = &brep.m_edges[ei];
        let c = w.write_nurbs_curve(&brep.m_curves_3d[e.curve_3d_index as usize], false);
        if c < 0 {
            eid.insert(ei, -1);
            return -1;
        }
        let sv = vertex_id(w, brep, vid, e.start_vertex as usize);
        let ev = vertex_id(w, brep, vid, e.end_vertex as usize);
        let id = w.write_raw(&format!("EDGE_CURVE('',#{},#{},#{},.T.)", sv, ev, c));
        eid.insert(ei, id);
        id
    }

    let mut face_id = |w: &mut StepWriter, fi: usize, fo: BRepOrientation| -> i32 {
        let f = &brep.m_faces[fi];
        let srf = match sid.get(&f.surface_index) {
            Some(s) => *s,
            None => {
                let s = w.write_nurbs_surface(&brep.m_surfaces[f.surface_index as usize]);
                sid.insert(f.surface_index, s);
                s
            }
        };
        if srf < 0 {
            return -1;
        }
        let mut bounds = Vec::new();
        for wi in 0..f.wires.len() {
            let mut oes = Vec::new();
            let mut any_vertex = -1i32;
            for er in brep.wire_edges(&f.wires[wi]) {
                let e = &brep.m_edges[er.index as usize];
                if any_vertex < 0 {
                    any_vertex = e.start_vertex;
                }
                if e.degenerated {
                    continue;
                }
                let ec = edge_id(w, brep, &mut vid, &mut eid, er.index as usize);
                if ec < 0 {
                    continue;
                }
                oes.push(w.write_raw(&format!(
                    "ORIENTED_EDGE('',*,*,#{},.{}.)",
                    ec,
                    if er.orientation == BRepOrientation::Forward {
                        "T"
                    } else {
                        "F"
                    }
                )));
            }
            let lp = if !oes.is_empty() {
                w.write_raw(&format!("EDGE_LOOP('',{})", fmt_ref_list(&oes)))
            } else if any_vertex >= 0 {
                let v = vertex_id(w, brep, &mut vid, any_vertex as usize);
                w.write_raw(&format!("VERTEX_LOOP('',#{})", v))
            } else {
                continue;
            };
            bounds.push(w.write_raw(&format!(
                "{}('',#{},.T.)",
                if wi == 0 {
                    "FACE_OUTER_BOUND"
                } else {
                    "FACE_BOUND"
                },
                lp
            )));
        }
        if bounds.is_empty() {
            return -1;
        }
        w.write_raw(&format!(
            "ADVANCED_FACE('',{},#{},.{}.)",
            fmt_ref_list(&bounds),
            srf,
            if fo == BRepOrientation::Forward {
                "T"
            } else {
                "F"
            }
        ))
    };

    let mut lo = [1e300f64; 3];
    let mut hi = [-1e300f64; 3];
    for v in &brep.m_vertices {
        for k in 0..3 {
            lo[k] = lo[k].min(v.point[k]);
            hi[k] = hi[k].max(v.point[k]);
        }
    }
    let diag = if brep.m_vertices.is_empty() {
        1.0
    } else {
        ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt()
    };

    let mut groups = Vec::new();
    let mut in_shell = vec![false; brep.m_faces.len()];
    for si in 0..brep.shell_count() {
        let mut ids = Vec::new();
        for fr in &brep.m_shells[si].faces {
            in_shell[fr.index as usize] = true;
            let id = face_id(w, fr.index as usize, fr.orientation);
            if id >= 0 {
                ids.push(id);
            }
        }
        if !ids.is_empty() {
            groups.push((ids, brep.is_closed(si)));
        }
    }
    let mut free_ids = Vec::new();
    for fi in 0..brep.face_count() {
        if in_shell[fi] {
            continue;
        }
        let id = face_id(w, fi, BRepOrientation::Forward);
        if id >= 0 {
            free_ids.push(id);
        }
    }
    if !free_ids.is_empty() {
        groups.push((free_ids, false));
    }
    (groups, diag)
}

pub fn write_file_step_brep(brep: &BRep, filepath: &str) {
    let mut w = StepWriter::new();
    let mut bodies = Vec::new();
    let mut any_closed = false;
    let (groups, diag) = emit_brep_shells(&mut w, brep);
    for (ids, closed) in groups {
        bodies.extend(w.make_bodies(&[ids], closed));
        any_closed = any_closed || closed;
    }
    let name = if brep.name.is_empty() {
        "brep"
    } else {
        &brep.name
    };
    w.finish_product(&bodies, any_closed, name, diag * 1e-4, &[]);
    write_step_string(&w.emit(), filepath);
}

pub fn write_file_step_breps(breps: &[&BRep], name: &str, filepath: &str) {
    let mut w = StepWriter::new();
    let mut bodies = Vec::new();
    let mut styled = Vec::new();
    let mut any_closed = false;
    let mut diag_all = 1.0f64;
    for b in breps {
        let (groups, diag) = emit_brep_shells(&mut w, b);
        diag_all = diag_all.max(diag);
        let col = &b.surfacecolor;
        let psa = w.color_style(col.r as f64, col.g as f64, col.b as f64);
        for (ids, closed) in groups {
            bodies.extend(w.make_bodies(std::slice::from_ref(&ids), closed));
            any_closed = any_closed || closed;
            for fid in ids {
                styled.push(w.write_raw(&format!("STYLED_ITEM('',(#{}),#{})", psa, fid)));
            }
        }
    }
    w.finish_product(&bodies, any_closed, name, diag_all * 1e-4, &styled);
    write_step_string(&w.emit(), filepath);
}
