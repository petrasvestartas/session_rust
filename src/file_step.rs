#![allow(clippy::needless_range_loop, clippy::manual_find)]
use crate::brep::BRep;
use crate::brep::BRepOrientation;
use crate::brep::BRepRef;
use crate::closest::Closest;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::nurbssurface_trimmed::NurbsSurfaceTrimmed;
use crate::point::Point;
use crate::tolerance::PI;
use crate::vector::Vector;
use std::collections::HashMap;

// ═══════════════════════════════════════════════════════════════════════════
// ISO 10303-21 parser
// ═══════════════════════════════════════════════════════════════════════════
/// Kind of value a StepParam holds.
#[derive(Clone, Copy, PartialEq)]
enum StepTag {
    Ref,  // Entity reference.
    Num,  // Real or integer.
    Str,  // Quoted string.
    Enum, // Enum literal or bare identifier.
    List, // Parenthesised list.
    Null, // Unset or derived value.
}

/// One parameter of an entity instance.
#[derive(Clone)]
struct StepParam {
    tag: StepTag,         // Which member is set.
    ref_id: i32,          // Entity id for Ref.
    num: f64,             // Value for Num.
    str: String,          // Text for Str and Enum.
    list: Vec<StepParam>, // Items for List.
}

impl StepParam {
    /// Construct a null parameter.
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

/// One TYPE(params) part of an entity instance.
struct StepSubEntity {
    type_: String,          // Entity type name.
    params: Vec<StepParam>, // Parameters in file order.
}

/// One entity: a single part for a simple instance, several for a complex one.
struct StepEntity {
    parts: Vec<StepSubEntity>, // Sub-entities of the instance.
}

impl StepEntity {
    /// Return whether any sub-entity carries type t.
    fn has(&self, t: &str) -> bool {
        for p in &self.parts {
            if p.type_ == t {
                return true;
            }
        }

        false
    }

    /// Return the first sub-entity of type t, or null.
    fn find(&self, t: &str) -> Option<&StepSubEntity> {
        for p in &self.parts {
            if p.type_ == t {
                return Some(p);
            }
        }

        None
    }
}

/// Return the first sub-entity of type t of an entity that may be missing.
fn find_in<'a>(e: Option<&'a StepEntity>, t: &str) -> Option<&'a StepSubEntity> {
    match e {
        Some(e) => e.find(t),
        None => None,
    }
}

/// Entities of a parsed file by id.
struct StepFile {
    entities: HashMap<i32, StepEntity>, // Entities by id.
}

impl StepFile {
    /// Return the sorted ids of every entity carrying type t.
    fn ids_of_type(&self, t: &str) -> Vec<i32> {
        let mut out = Vec::new();

        for (id, e) in &self.entities {
            if e.has(t) {
                out.push(*id);
            }
        }

        out.sort();

        out
    }
}

const PI_2: f64 = std::f64::consts::FRAC_PI_2; // Quarter turn.
const MAX_DEPTH: i32 = 8; // Deepest list nesting parsed recursively.
const NS: usize = 17; // Samples per side of a surface grid.

/// Read position in a STEP text.
struct Cursor<'a> {
    s: &'a [u8], // Text being parsed.
    p: usize,    // Current position.
    end: usize,  // End of text.
}

/// Advance the cursor past whitespace.
fn skip_ws(c: &mut Cursor) {
    while c.p < c.end && c.s[c.p].is_ascii_whitespace() {
        c.p += 1;
    }
}

/// Advance past ch when it is next, skipping whitespace first.
fn consume(c: &mut Cursor, ch: u8) -> bool {
    skip_ws(c);

    if c.p < c.end && c.s[c.p] == ch {
        c.p += 1;

        return true;
    }

    false
}

/// Return whether ch can start or continue an identifier.
fn isident(ch: u8) -> bool {
    ch.is_ascii_uppercase() || ch.is_ascii_digit() || ch == b'_'
}

/// Read an optionally signed integer.
fn parse_int(c: &mut Cursor) -> i32 {
    let mut id = 0i32;

    while c.p < c.end && c.s[c.p].is_ascii_digit() {
        id = id * 10 + (c.s[c.p] - b'0') as i32;
        c.p += 1;
    }

    id
}

/// Read a real, an integer or an enum literal as a double.
fn parse_number(c: &mut Cursor) -> f64 {
    let start = c.p;

    if c.p < c.end && (c.s[c.p] == b'+' || c.s[c.p] == b'-') {
        c.p += 1;
    }

    while c.p < c.end && (c.s[c.p].is_ascii_digit() || c.s[c.p] == b'.') {
        c.p += 1;
    }

    if c.p < c.end && (c.s[c.p] == b'e' || c.s[c.p] == b'E') {
        let mut q = c.p + 1;

        if q < c.end && (c.s[q] == b'+' || c.s[q] == b'-') {
            q += 1;
        }

        if q < c.end && c.s[q].is_ascii_digit() {
            c.p = q;

            while c.p < c.end && c.s[c.p].is_ascii_digit() {
                c.p += 1;
            }
        }
    }

    String::from_utf8_lossy(&c.s[start..c.p])
        .parse::<f64>()
        .unwrap_or(0.0)
}

/// Read an identifier.
fn parse_ident(c: &mut Cursor) -> String {
    let start = c.p;

    while c.p < c.end && isident(c.s[c.p]) {
        c.p += 1;
    }

    String::from_utf8_lossy(&c.s[start..c.p]).into_owned()
}

/// Read a quoted string, unescaping doubled quotes.
fn parse_string(c: &mut Cursor) -> String {
    let mut out = Vec::new();
    c.p += 1;

    while c.p < c.end {
        if c.s[c.p] != b'\'' {
            out.push(c.s[c.p]);
            c.p += 1;
            continue;
        }

        c.p += 1;

        if c.p >= c.end || c.s[c.p] != b'\'' {
            break;
        }

        out.push(b'\'');
        c.p += 1;
    }

    String::from_utf8_lossy(&out).into_owned()
}

/// Read a parenthesised parameter list, recursing one level deeper.
fn parse_params(c: &mut Cursor, depth: i32) -> Vec<StepParam> {
    let mut out = Vec::new();

    if !consume(c, b'(') {
        return out;
    }

    while c.p < c.end {
        skip_ws(c);

        if c.p >= c.end || c.s[c.p] == b')' {
            break;
        }

        out.push(parse_param(c, depth));
        skip_ws(c);

        if c.p < c.end && c.s[c.p] == b',' {
            c.p += 1;
        }
    }

    if c.p < c.end {
        c.p += 1;
    }

    out
}

/// Skip a parenthesised group without recursing, for lists nested deeper than MAX_DEPTH.
fn skip_list(c: &mut Cursor) {
    let mut open = 0;

    while c.p < c.end {
        if c.s[c.p] == b'(' {
            open += 1;
        }

        if c.s[c.p] == b')' {
            open -= 1;
        }

        c.p += 1;

        if open == 0 {
            return;
        }
    }
}

/// Read one parameter: reference, number, string, enum, list or sub-entity.
fn parse_param(c: &mut Cursor, depth: i32) -> StepParam {
    skip_ws(c);

    let mut r = StepParam::new();

    if c.p >= c.end {
        return r;
    }

    let ch = c.s[c.p];

    if ch == b'#' {
        c.p += 1;
        r.tag = StepTag::Ref;
        r.ref_id = parse_int(c);
    } else if ch == b'$' || ch == b'*' {
        c.p += 1;
    } else if ch == b'(' {
        r.tag = StepTag::List;

        if depth < MAX_DEPTH {
            r.list = parse_params(c, depth + 1);
        } else {
            skip_list(c);
        }
    } else if ch == b'\'' {
        r.tag = StepTag::Str;
        r.str = parse_string(c);
    } else if ch == b'.' {
        c.p += 1;

        let start = c.p;

        while c.p < c.end && c.s[c.p] != b'.' {
            c.p += 1;
        }

        r.tag = StepTag::Enum;
        r.str = String::from_utf8_lossy(&c.s[start..c.p]).into_owned();

        if c.p < c.end {
            c.p += 1;
        }
    } else if ch.is_ascii_digit() || ch == b'-' || ch == b'+' {
        r.tag = StepTag::Num;
        r.num = parse_number(c);
    } else if ch.is_ascii_uppercase() {
        r.tag = StepTag::Enum;
        r.str = parse_ident(c);
        skip_ws(c);

        if c.p < c.end && c.s[c.p] == b'(' {
            parse_params(c, depth + 1);
        }
    } else {
        c.p += 1;
    }

    r
}

/// Read one TYPE(params) instance.
fn parse_sub_entity(c: &mut Cursor) -> StepSubEntity {
    let mut sub = StepSubEntity {
        type_: parse_ident(c),
        params: Vec::new(),
    };
    skip_ws(c);

    if c.p < c.end && c.s[c.p] == b'(' {
        sub.params = parse_params(c, 0);
    }

    sub
}

/// Advance past the next semicolon.
fn skip_statement(c: &mut Cursor) {
    let mut in_str = false;

    while c.p < c.end {
        let ch = c.s[c.p];
        c.p += 1;

        if ch == b'\'' {
            in_str = !in_str;
        }

        if ch == b';' && !in_str {
            return;
        }
    }
}

/// Fill sf from the DATA section of a STEP text.
fn parse_step_string(content: &str, sf: &mut StepFile) {
    let mut c = Cursor {
        s: content.as_bytes(),
        p: 0,
        end: content.len(),
    };

    while c.p < c.end {
        skip_ws(&mut c);

        if c.p >= c.end {
            break;
        }

        if c.s[c.p] != b'#' {
            while c.p < c.end && c.s[c.p] != b'\n' {
                c.p += 1;
            }

            continue;
        }

        c.p += 1;

        let id = parse_int(&mut c);

        if !consume(&mut c, b'=') {
            continue;
        }

        skip_ws(&mut c);

        if c.p >= c.end {
            break;
        }

        let mut ent = StepEntity { parts: Vec::new() };

        if c.s[c.p] == b'(' {
            c.p += 1;

            while c.p < c.end {
                skip_ws(&mut c);

                if c.p >= c.end || !c.s[c.p].is_ascii_uppercase() {
                    break;
                }

                ent.parts.push(parse_sub_entity(&mut c));
            }

            if !consume(&mut c, b')') {
                skip_statement(&mut c);
                continue;
            }
        } else {
            ent.parts.push(parse_sub_entity(&mut c));
        }

        sf.entities.entry(id).or_insert(ent);
        skip_statement(&mut c);
    }
}

/// Remove /* */ comments from the raw text.
fn strip_comments(raw: &[u8]) -> Vec<u8> {
    let mut text = Vec::with_capacity(raw.len());

    let mut i = 0;

    while i < raw.len() {
        if i + 1 < raw.len() && raw[i] == b'/' && raw[i + 1] == b'*' {
            i += 2;

            while i + 1 < raw.len() && !(raw[i] == b'*' && raw[i + 1] == b'/') {
                i += 1;
            }

            i += 2;
        } else {
            text.push(raw[i]);
            i += 1;
        }
    }

    text
}

/// Read and parse a STEP file.
fn parse_step_file(filepath: &str) -> StepFile {
    let mut sf = StepFile {
        entities: HashMap::new(),
    };

    let Ok(raw) = std::fs::read(filepath) else {
        return sf;
    };

    let text = String::from_utf8_lossy(&strip_comments(&raw)).into_owned();

    let Some(lo) = text.find("DATA") else {
        return sf;
    };

    let Some(semi) = text[lo..].find(';') else {
        return sf;
    };

    let semi = semi + lo;

    let Some(endsec) = text[semi..].find("ENDSEC") else {
        return sf;
    };

    parse_step_string(&text[semi + 1..endsec + semi], &mut sf);

    sf
}

// ═══════════════════════════════════════════════════════════════════════════
// Parameter access
// ═══════════════════════════════════════════════════════════════════════════
/// Return the first reference parameter, or -1.
fn first_ref(params: &[StepParam]) -> i32 {
    for p in params {
        if p.tag == StepTag::Ref {
            return p.ref_id;
        }
    }

    -1
}

/// Return every reference parameter in order.
fn all_refs(params: &[StepParam]) -> Vec<i32> {
    let mut out = Vec::new();

    for p in params {
        if p.tag == StepTag::Ref {
            out.push(p.ref_id);
        }
    }

    out
}

/// Return every reference inside the list parameters.
fn list_refs(params: &[StepParam]) -> Vec<i32> {
    let mut out = Vec::new();

    for p in params {
        for id in all_refs(&p.list) {
            out.push(id);
        }
    }

    out
}

/// Return every numeric parameter in order.
fn nums(params: &[StepParam]) -> Vec<f64> {
    let mut out = Vec::new();

    for p in params {
        if p.tag == StepTag::Num {
            out.push(p.num);
        }
    }

    out
}

/// Return the numbers of a list parameter as integers.
fn int_list(p: &StepParam) -> Vec<i32> {
    let mut out = Vec::new();

    for v in nums(&p.list) {
        out.push(v as i32);
    }

    out
}

/// Return the numbers of a list parameter.
fn dbl_list(p: &StepParam) -> Vec<f64> {
    nums(&p.list)
}

/// Return the numbers of a list-of-lists parameter.
fn dbl_list_list(p: &StepParam) -> Vec<Vec<f64>> {
    let mut out = Vec::new();

    for row in &p.list {
        out.push(dbl_list(row));
    }

    out
}

/// Return the references of a list-of-lists parameter.
fn ref_list_list(p: &StepParam) -> Vec<Vec<i32>> {
    let mut out = Vec::new();

    for row in &p.list {
        out.push(all_refs(&row.list));
    }

    out
}

/// Numbers of the first list parameter that holds any.
fn coords(params: &[StepParam]) -> Vec<f64> {
    for p in params {
        let out = dbl_list(p);

        if !out.is_empty() {
            return out;
        }
    }

    Vec::new()
}

/// Last enum parameter as a flag (.T. is true), fallback when there is none.
fn last_flag(params: &[StepParam], fallback: bool) -> bool {
    let mut out = fallback;

    for p in params {
        if p.tag == StepTag::Enum {
            out = p.str == "T";
        }
    }

    out
}

/// Degree, control point ids and knots of a B-spline curve entity.
#[derive(Default)]
struct CurveParams {
    degree: i32,       // Polynomial degree.
    pt_refs: Vec<i32>, // CARTESIAN_POINT ids.
    mults: Vec<i32>,   // Knot multiplicities.
    knots: Vec<f64>,   // Distinct knot values.
}

/// Degrees, control point id grid and knots of a B-spline surface entity.
#[derive(Default)]
struct SurfaceParams {
    u_deg: i32,              // Degree in u.
    v_deg: i32,              // Degree in v.
    ctrl_pts: Vec<Vec<i32>>, // CARTESIAN_POINT ids, rows along u.
    u_mults: Vec<i32>,       // Knot multiplicities in u.
    v_mults: Vec<i32>,       // Knot multiplicities in v.
    u_knots: Vec<f64>,       // Distinct knot values in u.
    v_knots: Vec<f64>,       // Distinct knot values in v.
}

/// B_SPLINE_CURVE_WITH_KNOTS parameters, simple or split across a complex instance; none when missing, short or empty.
fn curve_params(e: &StepEntity) -> Option<CurveParams> {
    let bsc = e.find("B_SPLINE_CURVE_WITH_KNOTS")?;
    let mut cp = CurveParams::default();

    match e.find("B_SPLINE_CURVE") {
        None => {
            let pp = &bsc.params;

            if pp.len() < 8 {
                return None;
            }

            cp.degree = pp[1].num as i32;
            cp.pt_refs = all_refs(&pp[2].list);
            cp.mults = int_list(&pp[6]);
            cp.knots = dbl_list(&pp[7]);
        }
        Some(base) => {
            let bp = &base.params;
            let kp = &bsc.params;

            if bp.len() < 2 || kp.len() < 2 {
                return None;
            }

            cp.degree = bp[0].num as i32;
            cp.pt_refs = all_refs(&bp[1].list);
            cp.mults = int_list(&kp[0]);
            cp.knots = dbl_list(&kp[1]);
        }
    }

    if cp.pt_refs.is_empty() || cp.mults.is_empty() || cp.knots.is_empty() {
        return None;
    }

    Some(cp)
}

/// B_SPLINE_SURFACE_WITH_KNOTS parameters, simple or split across a complex instance; none when missing, short or empty.
fn surface_params(e: &StepEntity) -> Option<SurfaceParams> {
    let bss = e.find("B_SPLINE_SURFACE_WITH_KNOTS")?;
    let mut sp = SurfaceParams::default();

    match e.find("B_SPLINE_SURFACE") {
        None => {
            let pp = &bss.params;

            if pp.len() < 12 {
                return None;
            }

            sp.u_deg = pp[1].num as i32;
            sp.v_deg = pp[2].num as i32;
            sp.ctrl_pts = ref_list_list(&pp[3]);
            sp.u_mults = int_list(&pp[8]);
            sp.v_mults = int_list(&pp[9]);
            sp.u_knots = dbl_list(&pp[10]);
            sp.v_knots = dbl_list(&pp[11]);
        }
        Some(base) => {
            let bp = &base.params;
            let kp = &bss.params;

            if bp.len() < 3 || kp.len() < 4 {
                return None;
            }

            sp.u_deg = bp[0].num as i32;
            sp.v_deg = bp[1].num as i32;
            sp.ctrl_pts = ref_list_list(&bp[2]);
            sp.u_mults = int_list(&kp[0]);
            sp.v_mults = int_list(&kp[1]);
            sp.u_knots = dbl_list(&kp[2]);
            sp.v_knots = dbl_list(&kp[3]);
        }
    }

    if sp.ctrl_pts.is_empty()
        || sp.ctrl_pts[0].is_empty()
        || sp.u_mults.is_empty()
        || sp.v_mults.is_empty()
    {
        return None;
    }

    Some(sp)
}

// ═══════════════════════════════════════════════════════════════════════════
// Knot utilities
// ═══════════════════════════════════════════════════════════════════════════
/// Repeat each knot value by its multiplicity.
fn expand_knots(vals: &[f64], mults: &[i32]) -> Vec<f64> {
    let mut flat = Vec::new();

    for i in 0..vals.len().min(mults.len()) {
        for _ in 0..mults[i] {
            flat.push(vals[i]);
        }
    }

    flat
}

/// Collapse a flat knot vector into values and multiplicities.
fn compress_knots(flat: &[f64]) -> (Vec<f64>, Vec<i32>) {
    let mut vals: Vec<f64> = Vec::new();
    let mut mults: Vec<i32> = Vec::new();

    for &v in flat {
        if vals.is_empty() || (v - vals[vals.len() - 1]).abs() > 1e-12 {
            vals.push(v);
            mults.push(1);
        } else {
            let last = mults.len() - 1;
            mults[last] += 1;
        }
    }

    (vals, mults)
}

/// Add the two clamped end knots to an internal knot vector.
fn full_from_internal(internal: &[f64]) -> Vec<f64> {
    if internal.is_empty() {
        return Vec::new();
    }

    let mut full = vec![internal[0]];
    full.extend_from_slice(internal);
    full.push(internal[internal.len() - 1]);

    full
}

/// Drop the two clamped end knots of a full knot vector.
fn internal_from_full(full: &[f64]) -> Vec<f64> {
    if full.len() < 2 {
        return full.to_vec();
    }

    full[1..full.len() - 1].to_vec()
}

// ═══════════════════════════════════════════════════════════════════════════
// Analytic geometry
// ═══════════════════════════════════════════════════════════════════════════
/// Orthonormal frame of an AXIS2_PLACEMENT_3D.
#[derive(Clone)]
struct Axis2 {
    origin: Point, // Frame origin.
    ax: Vector,    // Frame x axis.
    ay: Vector,    // Frame y axis.
    az: Vector,    // Frame z axis.
    ok: bool,      // Whether the frame was read.
}

impl Axis2 {
    /// Construct the world frame.
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

/// Parameter projector of a surface: a plane or a cylinder on the quarter-arc chart.
#[derive(Clone)]
struct Proj {
    kind: i32, // 0 none, 1 plane, 2 cylinder.
    a: Axis2,  // Surface frame.
}

impl Proj {
    /// Construct a projector of no kind.
    fn new() -> Self {
        Proj {
            kind: 0,
            a: Axis2::new(),
        }
    }
}

/// Analytic surface of a face.
#[derive(Clone)]
struct AnFace {
    kind: i32,   // 2 cylinder, 3 cone, 4 sphere, 5 torus.
    a: Axis2,    // Surface frame.
    radius: f64, // Main radius.
    r2: f64,     // Cone semi-angle or torus minor radius.
}

impl AnFace {
    /// Construct a face of no kind.
    fn new() -> Self {
        AnFace {
            kind: 0,
            a: Axis2::new(),
            radius: 0.0,
            r2: 0.0,
        }
    }
}

/// Return the point at local coordinates in the axis frame.
fn axis_point(a: &Axis2, lx: f64, ly: f64, lz: f64) -> Point {
    &a.origin + &a.ax * lx + &a.ay * ly + &a.az * lz
}

/// Return the angle of pt around the axis in radians.
fn angle_of(a: &Axis2, pt: &Point) -> f64 {
    let d = pt - &a.origin;

    d.dot(&a.ay).atan2(d.dot(&a.ax))
}

/// Parameter within one quarter-arc rational span (w = sqrt(2)/2) whose angle is theta.
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
        let df = (x * dy - y * dx) / (x * x + y * y).max(1e-30);

        if df.abs() < 1e-30 {
            break;
        }

        let step = f / df;
        tau = (tau - step).clamp(0.0, 1.0);

        if step.abs() < 1e-15 {
            break;
        }
    }

    tau
}

/// Chart coordinate of an angle: quarter-arc spans counted from quarter q0, corrected for the projective span parameterization.
fn chart_u_of_angle(ang: f64, q0: i32) -> f64 {
    let q = ang / PI_2 - q0 as f64;
    let spanf = (q + 1e-12).floor();

    spanf + arc_param_of_angle((q - spanf) * PI_2)
}

/// Cos, sin and weight of the quarter-arc chart nodes from quarter q0: even nodes on the arc, odd nodes at the tangent corners.
fn arc_nodes(q0: f64, nspans: i32) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let n = (2 * nspans + 1) as usize;
    let mut ca = vec![0.0; n];
    let mut sa = vec![0.0; n];
    let mut cw = vec![0.0; n];

    for i in 0..n {
        let mid = i % 2 == 1;
        let a0 = (q0 + (i / 2) as f64) * PI_2;
        let a1 = (q0 + (i / 2) as f64 + 1.0) * PI_2;
        ca[i] = if mid { a0.cos() + a1.cos() } else { a0.cos() };
        sa[i] = if mid { a0.sin() + a1.sin() } else { a0.sin() };
        cw[i] = if mid { 2.0f64.sqrt() / 2.0 } else { 1.0 };
    }

    (ca, sa, cw)
}

/// Integer knots of nspans quarter arcs: 0, 0, 1, 1, ..., nspans, nspans.
fn quarter_knots(nspans: i32) -> Vec<f64> {
    let mut knots = vec![0.0, 0.0];

    for s in 1..nspans {
        knots.push(s as f64);
        knots.push(s as f64);
    }

    knots.push(nspans as f64);
    knots.push(nspans as f64);

    knots
}

/// Canonical (s, t) of a 3D point; radial_ok is false at a pole or apex where the angle is undefined.
fn an_st_of(an: &AnFace, p: &Point) -> (f64, f64, bool) {
    let d = p - &an.a.origin;
    let x = d.dot(&an.a.ax);
    let y = d.dot(&an.a.ay);
    let z = d.dot(&an.a.az);
    let rho = (x * x + y * y).sqrt();
    let s = y.atan2(x);
    let radial_ok = rho > 1e-9;

    if an.kind == 2 {
        return (s, z, radial_ok);
    }

    if an.kind == 3 {
        let ca = an.r2.cos();

        return (s, if ca.abs() > 1e-12 { z / ca } else { z }, radial_ok);
    }

    if an.kind == 4 {
        return (s, z.atan2(rho), radial_ok);
    }

    (s, z.atan2(rho - an.radius), radial_ok)
}

/// Evaluate the analytic surface at chart parameters s, t.
fn an_eval(an: &AnFace, s: f64, t: f64) -> Point {
    let cs = s.cos();
    let sn = s.sin();

    if an.kind == 2 {
        return axis_point(&an.a, an.radius * cs, an.radius * sn, t);
    }

    if an.kind == 3 {
        let r = an.radius + t * an.r2.sin();

        return axis_point(&an.a, r * cs, r * sn, t * an.r2.cos());
    }

    let ct = t.cos();
    let st = t.sin();

    if an.kind == 4 {
        return axis_point(
            &an.a,
            an.radius * ct * cs,
            an.radius * ct * sn,
            an.radius * st,
        );
    }

    let r = an.radius + an.r2 * ct;

    axis_point(&an.a, r * cs, r * sn, an.r2 * st)
}

/// Kernel NURBS window of an analytic surface: nsu quarter arcs from quarter su0 in u; v is linear on [t0, t1] for cylinder and cone, nsv quarter arcs from sv0 for sphere and torus.
fn build_analytic_nurbs(
    an: &AnFace,
    su0: i32,
    nsu: i32,
    t0: f64,
    t1: f64,
    sv0: i32,
    nsv: i32,
) -> NurbsSurface {
    let (ca, sa, cw) = arc_nodes(su0 as f64, nsu);
    let nu = (2 * nsu + 1) as usize;

    if an.kind == 2 || an.kind == 3 {
        let mut srf = NurbsSurface::new(3, true, 3, 2, nu, 2);
        srf.m_nurbsknot[0] = quarter_knots(nsu);
        srf.m_nurbsknot[1] = vec![t0, t1];

        for j in 0..2 {
            let t = if j == 0 { t0 } else { t1 };
            let r = if an.kind == 2 {
                an.radius
            } else {
                an.radius + t * an.r2.sin()
            };
            let z = if an.kind == 2 { t } else { t * an.r2.cos() };

            for i in 0..nu {
                let p = axis_point(&an.a, r * ca[i], r * sa[i], z);

                if !srf.set_cv_4d(i, j, cw[i] * p[0], cw[i] * p[1], cw[i] * p[2], cw[i]) {
                    return NurbsSurface::default();
                }
            }
        }

        return srf;
    }

    let (cb, sb, vw) = arc_nodes(sv0 as f64, nsv);
    let nv = (2 * nsv + 1) as usize;
    let mut srf = NurbsSurface::new(3, true, 3, 3, nu, nv);
    srf.m_nurbsknot[0] = quarter_knots(nsu);
    srf.m_nurbsknot[1] = quarter_knots(nsv);

    for j in 0..nv {
        let r = if an.kind == 4 {
            an.radius * cb[j]
        } else {
            an.radius + an.r2 * cb[j]
        };
        let z = if an.kind == 4 {
            an.radius * sb[j]
        } else {
            an.r2 * sb[j]
        };

        for i in 0..nu {
            let p = axis_point(&an.a, r * ca[i], r * sa[i], z);
            let wij = cw[i] * vw[j];

            if !srf.set_cv_4d(i, j, wij * p[0], wij * p[1], wij * p[2], wij) {
                return NurbsSurface::default();
            }
        }
    }

    srf
}

/// Bilinear patch of the plane with axis a over [u0, u1] x [v0, v1].
fn plane_surface(a: &Axis2, u0: f64, u1: f64, v0: f64, v1: f64) -> NurbsSurface {
    let mut out = NurbsSurface::new(3, false, 2, 2, 2, 2);
    out.m_nurbsknot[0] = vec![u0, u1];
    out.m_nurbsknot[1] = vec![v0, v1];

    let ok = out.set_cv(0, 0, &axis_point(a, u0, v0, 0.0))
        && out.set_cv(0, 1, &axis_point(a, u0, v1, 0.0))
        && out.set_cv(1, 0, &axis_point(a, u1, v0, 0.0))
        && out.set_cv(1, 1, &axis_point(a, u1, v1, 0.0));

    if ok {
        out
    } else {
        NurbsSurface::default()
    }
}

/// Rational cylinder patch on the quarter-arc chart (1 unit = 90 degrees) over [u0, u1] x [v0, v1]; a span of 4 closes it.
fn cylinder_surface(
    a: &Axis2,
    radius: f64,
    u0: f64,
    mut u1: f64,
    v0: f64,
    v1: f64,
) -> NurbsSurface {
    let closed = ((u1 - u0) - 4.0).abs() < 0.2;
    let n_spans = if closed {
        4
    } else {
        1.max(((u1 - u0).abs() - 1e-9).ceil() as i32)
    };

    if closed {
        u1 = u0 + 4.0;
    }

    let n_u = (2 * n_spans + 1) as usize;
    let mut out = NurbsSurface::new(3, true, 3, 2, n_u, 2);
    let mut knots = vec![u0, u0];

    for s in 1..n_spans {
        knots.push(u0 + s as f64);
        knots.push(u0 + s as f64);
    }

    knots.push(u1);
    knots.push(u1);
    out.m_nurbsknot[0] = knots;
    out.m_nurbsknot[1] = vec![v0, v1];

    let (ca, sa, cw) = arc_nodes(u0, n_spans);

    for i in 0..n_u {
        for j in 0..2 {
            let p = axis_point(
                a,
                radius * ca[i],
                radius * sa[i],
                if j == 0 { v0 } else { v1 },
            );

            if !out.set_cv_4d(i, j, cw[i] * p[0], cw[i] * p[1], cw[i] * p[2], cw[i]) {
                return NurbsSurface::default();
            }
        }
    }

    out
}

/// Parameter-space image of a 3D point: plane coordinates, or cylinder (angle in quarter turns, height).
fn project(pr: &Proj, pt: &Point) -> (f64, f64) {
    let d = pt - &pr.a.origin;

    if pr.kind == 1 {
        return (d.dot(&pr.a.ax), d.dot(&pr.a.ay));
    }

    (
        d.dot(&pr.a.ay).atan2(d.dot(&pr.a.ax)) * 2.0 / PI,
        d.dot(&pr.a.az),
    )
}

/// Affine projector of a bilinear patch from its corner p00; kind 0 when the patch is not bilinear or degenerate.
fn bilinear_projector(srf: &NurbsSurface) -> Proj {
    let mut pr = Proj::new();

    if !srf.is_valid() || srf.degree(0) != 1 || srf.degree(1) != 1 {
        return pr;
    }

    let p00 = srf.get_cv(0, 0).unwrap_or_default();
    let eu = srf.get_cv(1, 0).unwrap_or_default() - p00.clone();
    let ev = srf.get_cv(0, 1).unwrap_or_default() - p00.clone();
    let eu2 = eu.dot(&eu);
    let ev2 = ev.dot(&ev);

    if eu2 <= 1e-28 || ev2 <= 1e-28 {
        return pr;
    }

    pr.kind = 1;
    pr.a.origin = p00;
    pr.a.ax = eu * (1.0 / eu2);
    pr.a.ay = ev * (1.0 / ev2);
    pr.a.ok = true;

    pr
}

// ═══════════════════════════════════════════════════════════════════════════
// Curve helpers
// ═══════════════════════════════════════════════════════════════════════════
/// n points evenly spaced in parameter over the curve domain.
fn sample_nurbs(nc: &NurbsCurve, n: i32) -> Vec<Point> {
    let (tmin, tmax) = nc.domain();
    let mut pts = Vec::new();

    for i in 0..n {
        pts.push(nc.point_at(if n > 1 {
            tmin + (tmax - tmin) * i as f64 / (n - 1) as f64
        } else {
            tmin
        }));
    }

    pts
}

/// Degree-1 curve through the points with integer knots, dim 2 or 3; invalid for fewer than two points.
fn polyline_nurbs(pts: &[Point], dim: usize) -> NurbsCurve {
    let n = pts.len();

    if n < 2 {
        return NurbsCurve::default();
    }

    let mut nc = NurbsCurve::new(dim, false, 2, n);

    for i in 0..n {
        nc.m_nurbsknot[i] = i as f64;

        if !nc.set_cv(i, &pts[i]) {
            return NurbsCurve::default();
        }
    }

    nc
}

/// Exact rational arc on the circle (axis a, radius rad) from vs to ve, the full circle when they coincide.
fn circle_nurbs(a: &Axis2, rad: f64, vs: &Point, ve: &Point) -> NurbsCurve {
    let sa = angle_of(a, vs);
    let mut ea = angle_of(a, ve);

    if vs.distance(ve, None) < 1e-10 {
        ea = sa + 2.0 * PI;
    } else if ea <= sa {
        ea += 2.0 * PI;
    }

    let span = ea - sa;
    let ns = 1.max((span.abs() / PI_2).ceil() as i32);
    let n_cp = (2 * ns + 1) as usize;
    let wm = (span / (2.0 * ns as f64)).cos();
    let mut crv = NurbsCurve::new(3, true, 3, n_cp);
    crv.m_nurbsknot[0] = sa;
    crv.m_nurbsknot[1] = sa;

    for s in 1..ns {
        crv.m_nurbsknot[(2 * s) as usize] = sa + s as f64 * span / ns as f64;
        crv.m_nurbsknot[(2 * s + 1) as usize] = sa + s as f64 * span / ns as f64;
    }

    crv.m_nurbsknot[(2 * ns) as usize] = ea;
    crv.m_nurbsknot[(2 * ns + 1) as usize] = ea;

    for i in 0..n_cp {
        let mid = i % 2 == 1;
        let ang = sa + ((i / 2) as f64 + if mid { 0.5 } else { 0.0 }) * span / ns as f64;
        let w = if mid { wm } else { 1.0 };
        let r2 = if mid { rad / wm } else { rad };
        let p = &a.origin + (&a.ax * ang.cos() + &a.ay * ang.sin()) * r2;

        if !crv.set_cv_4d(i, w * p[0], w * p[1], w * p[2], w) {
            return NurbsCurve::default();
        }
    }

    crv
}

/// Degree-1 pcurve from (u0, v0) to (u1, v1).
fn uv_line(u0: f64, v0: f64, u1: f64, v1: f64) -> NurbsCurve {
    NurbsCurve::create(
        false,
        1,
        &[Point::new(u0, v0, 0.0), Point::new(u1, v1, 0.0)],
    )
}

/// Exact pcurve of a 3D curve under an affine projector: control points map one to one, weights unchanged.
fn exact_pcurve(proj: &Proj, c3: &NurbsCurve) -> NurbsCurve {
    if proj.kind != 1 || !c3.is_valid() || c3.cv_count() < 2 {
        return NurbsCurve::default();
    }

    let mut p2 = NurbsCurve::new(3, c3.is_rational(), c3.order(), c3.cv_count());
    p2.m_nurbsknot = c3.m_nurbsknot.clone();

    for ci in 0..c3.cv_count() {
        let (wx, wy, wz, w) = c3.get_cv_4d(ci).unwrap_or_default();

        if w.abs() < 1e-300 {
            return NurbsCurve::default();
        }

        let (u, v) = project(proj, &Point::new(wx / w, wy / w, wz / w));

        if !p2.set_cv_4d(ci, u * w, v * w, 0.0, w) {
            return NurbsCurve::default();
        }
    }

    if p2.is_valid() {
        p2
    } else {
        NurbsCurve::default()
    }
}

/// Keep consecutive cylinder samples on one branch of the quarter-arc chart (period 4).
fn unwrap_seam(uv: &mut [Point]) {
    for k in 1..uv.len() {
        let du = uv[k][0] - uv[k - 1][0];

        if du > 2.0 {
            uv[k][0] -= 4.0;
        } else if du < -2.0 {
            uv[k][0] += 4.0;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Surface grid
// ═══════════════════════════════════════════════════════════════════════════
/// ns x ns surface points over the domain, row-major with u slowest.
fn surface_grid(srf: &NurbsSurface, ns: usize) -> Vec<Point> {
    let (u0, u1) = srf.domain(0).unwrap_or_default();
    let (v0, v1) = srf.domain(1).unwrap_or_default();
    let mut grid = Vec::new();

    for i in 0..ns {
        for j in 0..ns {
            grid.push(
                srf.point_at(
                    u0 + (u1 - u0) * i as f64 / (ns - 1) as f64,
                    v0 + (v1 - v0) * j as f64 / (ns - 1) as f64,
                )
                .unwrap_or_default(),
            );
        }
    }

    grid
}

/// Return the largest distance from the first grid point.
fn grid_scale(grid: &[Point]) -> f64 {
    let mut scale = 0.0f64;

    for p in grid {
        scale = scale.max(p.distance(&grid[0], None));
    }

    scale
}

/// True when the first and last row (along_u) or column coincide within tol.
fn grid_closed(grid: &[Point], ns: usize, tol: f64, along_u: bool) -> bool {
    for k in 0..ns {
        let a = if along_u { &grid[k] } else { &grid[k * ns] };
        let b = if along_u {
            &grid[(ns - 1) * ns + k]
        } else {
            &grid[k * ns + ns - 1]
        };

        if a.distance(b, None) > tol {
            return false;
        }
    }

    true
}

/// True when column j collapses to one point (a pole or apex).
fn grid_degenerate(grid: &[Point], ns: usize, tol: f64, j: usize) -> bool {
    for k in 1..ns {
        if grid[k * ns + j].distance(&grid[j], None) > tol {
            return false;
        }
    }

    true
}

// ═══════════════════════════════════════════════════════════════════════════
// StepReader
// ═══════════════════════════════════════════════════════════════════════════
/// Entity access over a parsed file with points, directions and frames cached by id.
struct StepReader<'a> {
    sf: &'a StepFile,                // Parsed file.
    pt_cache: HashMap<i32, Point>,   // Points by id.
    dir_cache: HashMap<i32, Vector>, // Directions by id.
    ax_cache: HashMap<i32, Axis2>,   // Frames by id.
}

impl<'a> StepReader<'a> {
    /// Construct over a parsed file.
    fn new(sf: &'a StepFile) -> Self {
        StepReader {
            sf,
            pt_cache: HashMap::new(),
            dir_cache: HashMap::new(),
            ax_cache: HashMap::new(),
        }
    }

    /// Return the entity with this id, or null.
    fn get(&self, id: i32) -> Option<&'a StepEntity> {
        self.sf.entities.get(&id)
    }

    /// Read a CARTESIAN_POINT, caching by id.
    fn get_point(&mut self, id: i32) -> Point {
        if let Some(pt) = self.pt_cache.get(&id) {
            return pt.clone();
        }

        let mut pt = Point::new(0.0, 0.0, 0.0);

        if let Some(sub) = find_in(self.get(id), "CARTESIAN_POINT") {
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

    /// Read the CARTESIAN_POINT of a VERTEX_POINT, none when missing.
    fn get_vertex_point(&mut self, id: i32) -> Option<Point> {
        let sub = find_in(self.get(id), "VERTEX_POINT")?;
        let pt_ref = first_ref(&sub.params);

        if pt_ref < 0 {
            return None;
        }

        Some(self.get_point(pt_ref))
    }

    /// Read a DIRECTION as a unit vector, caching by id.
    fn get_direction(&mut self, id: i32) -> Vector {
        if let Some(v) = self.dir_cache.get(&id) {
            return v.clone();
        }

        let mut v = Vector::new(0.0, 0.0, 1.0);

        if let Some(sub) = find_in(self.get(id), "DIRECTION") {
            let c = coords(&sub.params);

            if c.len() >= 3 {
                v = Vector::new(c[0], c[1], c[2]);
            }
        }

        self.dir_cache.insert(id, v.clone());

        v
    }

    /// AXIS2_PLACEMENT_3D as an orthonormal frame: az normalized, ax made orthogonal to it, ay = az x ax.
    fn get_axis2(&mut self, id: i32) -> Axis2 {
        if let Some(a) = self.ax_cache.get(&id) {
            return a.clone();
        }

        let mut a = Axis2::new();
        let mut refs = Vec::new();

        if let Some(sub) = find_in(self.get(id), "AXIS2_PLACEMENT_3D") {
            refs = all_refs(&sub.params);
        }

        if refs.is_empty() {
            self.ax_cache.insert(id, a.clone());

            return a;
        }

        a.origin = self.get_point(refs[0]);
        a.az = if refs.len() > 1 {
            self.get_direction(refs[1])
        } else {
            Vector::new(0.0, 0.0, 1.0)
        };

        let ln = a.az.magnitude();

        if ln > 1e-12 {
            a.az = &a.az * (1.0 / ln);
        }

        if refs.len() > 2 {
            a.ax = self.get_direction(refs[2]);
        } else if a.az[0].abs() < 0.9 {
            a.ax = Vector::new(1.0, 0.0, 0.0);
        } else {
            a.ax = Vector::new(0.0, 1.0, 0.0);
        }

        a.ax = &a.ax - &a.az * a.ax.dot(&a.az);

        let xn = a.ax.magnitude();

        if xn > 1e-12 {
            a.ax = &a.ax * (1.0 / xn);
        }

        a.ay = a.az.cross(&a.ax);
        a.ok = true;
        self.ax_cache.insert(id, a.clone());

        a
    }

    /// B_SPLINE_CURVE_WITH_KNOTS, simple or complex with RATIONAL_B_SPLINE_CURVE; invalid when malformed.
    fn get_nurbs_curve(&mut self, id: i32) -> NurbsCurve {
        let Some(e) = self.get(id) else {
            return NurbsCurve::default();
        };

        let Some(cp) = curve_params(e) else {
            return NurbsCurve::default();
        };

        let order = cp.degree + 1;
        let cv_count = cp.pt_refs.len() as i32;
        let full = expand_knots(&cp.knots, &cp.mults);

        if full.len() as i32 != cv_count + order {
            return NurbsCurve::default();
        }

        let internal = internal_from_full(&full);
        let rat = e.find("RATIONAL_B_SPLINE_CURVE");
        let is_rat = rat.is_some();
        let weights = match rat {
            Some(rat) if !rat.params.is_empty() => dbl_list(&rat.params[0]),
            _ => Vec::new(),
        };

        let mut nc = NurbsCurve::new(3, is_rat, order as usize, cv_count as usize);

        if nc.m_nurbsknot.len() != internal.len() {
            return NurbsCurve::default();
        }

        nc.m_nurbsknot = internal;

        for i in 0..cv_count as usize {
            let pt = self.get_point(cp.pt_refs[i]);
            let w = if is_rat && i < weights.len() {
                weights[i]
            } else {
                1.0
            };

            if !nc.set_cv_4d(i, w * pt[0], w * pt[1], w * pt[2], w) {
                return NurbsCurve::default();
            }
        }

        nc
    }

    /// B_SPLINE_SURFACE_WITH_KNOTS, simple or complex with RATIONAL_B_SPLINE_SURFACE; invalid when malformed.
    fn get_nurbs_surface(&mut self, id: i32) -> NurbsSurface {
        let Some(e) = self.get(id) else {
            return NurbsSurface::default();
        };

        let Some(sp) = surface_params(e) else {
            return NurbsSurface::default();
        };

        let cv_u = sp.ctrl_pts.len() as i32;
        let cv_v = sp.ctrl_pts[0].len() as i32;
        let full_u = expand_knots(&sp.u_knots, &sp.u_mults);
        let full_v = expand_knots(&sp.v_knots, &sp.v_mults);

        if full_u.len() as i32 != cv_u + sp.u_deg + 1 || full_v.len() as i32 != cv_v + sp.v_deg + 1
        {
            return NurbsSurface::default();
        }

        let rat = e.find("RATIONAL_B_SPLINE_SURFACE");
        let is_rat = rat.is_some();
        let weights = match rat {
            Some(rat) if !rat.params.is_empty() => dbl_list_list(&rat.params[0]),
            _ => Vec::new(),
        };

        let mut srf = NurbsSurface::new(
            3,
            is_rat,
            (sp.u_deg + 1) as usize,
            (sp.v_deg + 1) as usize,
            cv_u as usize,
            cv_v as usize,
        );
        srf.m_nurbsknot[0] = internal_from_full(&full_u);
        srf.m_nurbsknot[1] = internal_from_full(&full_v);

        for u in 0..cv_u as usize {
            for v in 0..(cv_v as usize).min(sp.ctrl_pts[u].len()) {
                let pt = self.get_point(sp.ctrl_pts[u][v]);
                let w = if is_rat && u < weights.len() && v < weights[u].len() {
                    weights[u][v]
                } else {
                    1.0
                };

                if !srf.set_cv_4d(u, v, w * pt[0], w * pt[1], w * pt[2], w) {
                    return NurbsSurface::default();
                }
            }
        }

        if srf.is_valid() {
            srf
        } else {
            NurbsSurface::default()
        }
    }

    /// The 3D basis curve behind a SURFACE_CURVE or SEAM_CURVE, the id itself otherwise.
    fn basis_curve_of(&self, curve_id: i32) -> i32 {
        let e = self.get(curve_id);
        let mut sc = find_in(e, "SURFACE_CURVE");

        if sc.is_none() {
            sc = find_in(e, "SEAM_CURVE");
        }

        let id = match sc {
            Some(sc) => first_ref(&sc.params),
            None => -1,
        };

        if id >= 0 {
            id
        } else {
            curve_id
        }
    }

    /// n points along a curve entity: a B-spline or a circle arc between the vertices, else the two vertices.
    fn sample_curve(
        &mut self,
        curve_id: i32,
        v_start: &Point,
        v_end: &Point,
        n: i32,
    ) -> Vec<Point> {
        let ends = vec![v_start.clone(), v_end.clone()];
        let mut id = self.basis_curve_of(curve_id);

        for _ in 0..MAX_DEPTH {
            let Some(tc) = find_in(self.get(id), "TRIMMED_CURVE") else {
                break;
            };

            id = self.basis_curve_of(first_ref(&tc.params));
        }

        let Some(e) = self.get(id) else {
            return ends;
        };

        if e.has("B_SPLINE_CURVE_WITH_KNOTS") {
            let nc = self.get_nurbs_curve(id);

            return if nc.is_valid() {
                sample_nurbs(&nc, n)
            } else {
                ends
            };
        }

        let Some(circle) = e.find("CIRCLE") else {
            return ends;
        };

        let ax_ref = first_ref(&circle.params);
        let rr = nums(&circle.params);
        let rad = if rr.is_empty() { 0.0 } else { rr[0] };
        let a = self.get_axis2(ax_ref);

        if ax_ref < 0 || rad == 0.0 || !a.ok {
            return ends;
        }

        let sa = angle_of(&a, v_start);
        let mut ea = angle_of(&a, v_end);

        if ea <= sa {
            ea += 2.0 * PI;
        }

        let mut pts = Vec::new();

        for i in 0..n {
            let ang = if n > 1 {
                sa + (ea - sa) * i as f64 / (n - 1) as f64
            } else {
                sa
            };
            pts.push(&a.origin + (&a.ax * ang.cos() + &a.ay * ang.sin()) * rad);
        }

        pts
    }

    /// Return the parameter projector of a surface, caching by id.
    fn get_projector(&mut self, surface_id: i32) -> Proj {
        let mut pr = Proj::new();
        let e = self.get(surface_id);
        let plane = find_in(e, "PLANE");
        let cyl = find_in(e, "CYLINDRICAL_SURFACE");
        let sub = if plane.is_some() { plane } else { cyl };

        let id = match sub {
            Some(sub) => first_ref(&sub.params),
            None => -1,
        };

        if id < 0 {
            return pr;
        }

        pr.a = self.get_axis2(id);
        pr.kind = if plane.is_some() { 1 } else { 2 };

        pr
    }

    /// Kernel surface of a surface entity: the B-spline itself, or a plane or cylinder patch over the padded uv window.
    fn fill_surface(&mut self, id: i32, u0: f64, u1: f64, v0: f64, v1: f64) -> NurbsSurface {
        let Some(e) = self.get(id) else {
            return NurbsSurface::default();
        };

        if e.has("B_SPLINE_SURFACE_WITH_KNOTS") {
            return self.get_nurbs_surface(id);
        }

        let plane = e.find("PLANE");
        let cyl = e.find("CYLINDRICAL_SURFACE");
        let sub = if plane.is_some() { plane } else { cyl };

        let ax_ref = match sub {
            Some(sub) => first_ref(&sub.params),
            None => -1,
        };

        let a = self.get_axis2(ax_ref);

        if ax_ref < 0 || !a.ok {
            return NurbsSurface::default();
        }

        let pad_v = 1e-6f64.max(0.01 * (v1 - v0));

        if plane.is_some() {
            let pad_u = 1e-6f64.max(0.01 * (u1 - u0));

            return plane_surface(&a, u0 - pad_u, u1 + pad_u, v0 - pad_v, v1 + pad_v);
        }

        let rr = match sub {
            Some(sub) => nums(&sub.params),
            None => Vec::new(),
        };

        cylinder_surface(
            &a,
            if rr.is_empty() { 1.0 } else { rr[0] },
            u0,
            u1,
            v0 - pad_v,
            v1 + pad_v,
        )
    }

    /// CYLINDRICAL, CONICAL, SPHERICAL or TOROIDAL_SURFACE as an analytic face; kind 0 otherwise.
    fn get_analytic_srf(&mut self, id: i32) -> AnFace {
        let mut an = AnFace::new();

        let Some(e) = self.get(id) else {
            return an;
        };

        let kinds = [
            "CYLINDRICAL_SURFACE",
            "CONICAL_SURFACE",
            "SPHERICAL_SURFACE",
            "TOROIDAL_SURFACE",
        ];

        for i in 0..kinds.len() {
            let Some(sub) = e.find(kinds[i]) else {
                continue;
            };

            let ax_ref = first_ref(&sub.params);

            if ax_ref < 0 {
                return an;
            }

            an.a = self.get_axis2(ax_ref);

            if !an.a.ok {
                return an;
            }

            let rr = nums(&sub.params);
            an.kind = i as i32 + 2;
            an.radius = if rr.is_empty() { 0.0 } else { rr[0] };
            an.r2 = if rr.len() > 1 { rr[1] } else { 0.0 };

            return an;
        }

        an
    }

    /// Canonical (s, t) samples of the pcurve an edge carries on a surface; a SEAM_CURVE holds two, the second for the reversed use.
    fn pcurve_st_samples(
        &mut self,
        ec_geom_id: i32,
        surface_ref: i32,
        forward_use: bool,
        n: i32,
    ) -> Vec<Point> {
        let e = self.get(ec_geom_id);
        let mut sc = find_in(e, "SURFACE_CURVE");
        let is_seam = sc.is_none() && find_in(e, "SEAM_CURVE").is_some();

        if is_seam {
            sc = find_in(e, "SEAM_CURVE");
        }

        let Some(sc) = sc else {
            return Vec::new();
        };

        let mut mine = Vec::new();

        for pid in list_refs(&sc.params) {
            let Some(pc) = find_in(self.get(pid), "PCURVE") else {
                continue;
            };

            let refs = all_refs(&pc.params);

            if refs.len() >= 2 && refs[0] == surface_ref {
                mine.push(refs[1]);
            }
        }

        if mine.is_empty() {
            return Vec::new();
        }

        let pick = if is_seam && mine.len() > 1 && !forward_use {
            1
        } else {
            0
        };

        let Some(drs) = find_in(self.get(mine[pick]), "DEFINITIONAL_REPRESENTATION") else {
            return Vec::new();
        };

        let c2_refs = list_refs(&drs.params);

        if c2_refs.is_empty() {
            return Vec::new();
        }

        let c2 = self.get_nurbs_curve(c2_refs[0]);

        if c2.is_valid() {
            sample_nurbs(&c2, n)
        } else {
            Vec::new()
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Topology access
// ═══════════════════════════════════════════════════════════════════════════
/// One face bound: outer flag, orientation and the ORIENTED_EDGE ids of its EDGE_LOOP.
struct Bound {
    is_outer: bool,    // Whether the bound is FACE_OUTER_BOUND.
    orient: bool,      // Bound orientation flag.
    oe_refs: Vec<i32>, // ORIENTED_EDGE ids.
}

/// Read a FACE_BOUND or FACE_OUTER_BOUND with its EDGE_LOOP.
fn bound_loop(r: &StepReader, bid: i32) -> Option<Bound> {
    let bent = r.get(bid)?;
    let mut bsub = bent.find("FACE_OUTER_BOUND");

    if bsub.is_none() {
        bsub = bent.find("FACE_BOUND");
    }

    let bsub = bsub?;
    let lp = find_in(r.get(first_ref(&bsub.params)), "EDGE_LOOP")?;

    Some(Bound {
        is_outer: bent.has("FACE_OUTER_BOUND"),
        orient: last_flag(&bsub.params, true),
        oe_refs: list_refs(&lp.params),
    })
}

/// EDGE_CURVE id (-1 when missing) and orientation of an ORIENTED_EDGE.
fn oriented_edge(r: &StepReader, oe_id: i32) -> (i32, bool) {
    let Some(oe) = find_in(r.get(oe_id), "ORIENTED_EDGE") else {
        return (-1, true);
    };

    let refs = all_refs(&oe.params);

    (
        if refs.is_empty() {
            -1
        } else {
            refs[refs.len() - 1]
        },
        last_flag(&oe.params, true),
    )
}

/// Start vertex, end vertex and geometry ids of an EDGE_CURVE; empty when missing.
fn edge_refs(r: &StepReader, ec_ref: i32) -> Vec<i32> {
    match find_in(r.get(ec_ref), "EDGE_CURVE") {
        Some(ec) => all_refs(&ec.params),
        None => Vec::new(),
    }
}

/// Return the geometry id of an EDGE_CURVE, or -1.
fn edge_geom_id(r: &StepReader, ec_ref: i32) -> i32 {
    let refs = edge_refs(r, ec_ref);

    if refs.len() >= 3 {
        refs[2]
    } else {
        -1
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BRep assembly from STEP
// ═══════════════════════════════════════════════════════════════════════════
/// One edge use in loop-traversal order; c2d is flipped into the edge direction when stored.
struct PendingEdge {
    edge: usize,     // Brep edge index.
    reversed: bool,  // Whether the edge runs against the loop.
    c2d: NurbsCurve, // Parameter-space curve.
}

/// One edge use with its parameter-space samples in curve order; pc2d is an exact pcurve when exact.
#[derive(Clone)]
struct LoopEdge {
    edge_idx: usize,  // Brep edge index.
    reversed: bool,   // Whether the edge runs against the loop.
    uv: Vec<Point>,   // Sampled uv points.
    pc2d: NurbsCurve, // Parameter-space curve.
    exact: bool,      // Whether pc2d is exact rather than sampled.
}

/// One face loop with its edge uses in traversal order.
#[derive(Clone)]
struct Loop {
    is_outer: bool,       // Whether the loop is the outer boundary.
    projected: bool,      // Whether the uv came from projection.
    edges: Vec<LoopEdge>, // Edges in traversal order.
}

/// Chart window of an analytic face: quarter arcs from su0 in u, from sv0 in v for sphere and torus, [t0, t1] otherwise.
struct Window {
    su0: i32, // First quarter arc in u.
    nsu: i32, // Quarter arcs in u.
    sv0: i32, // First quarter arc in v.
    nsv: i32, // Quarter arcs in v.
    t0: f64,  // Start of the linear domain.
    t1: f64,  // End of the linear domain.
}

/// (umin, umax, vmin, vmax) over the samples of one loop; umin > umax when there are none.
fn loop_bounds(lp: &Loop) -> (f64, f64, f64, f64) {
    let mut umin = 1e300f64;
    let mut umax = -1e300f64;
    let mut vmin = 1e300f64;
    let mut vmax = -1e300f64;

    for le in &lp.edges {
        for p in &le.uv {
            umin = umin.min(p[0]);
            umax = umax.max(p[0]);
            vmin = vmin.min(p[1]);
            vmax = vmax.max(p[1]);
        }
    }

    (umin, umax, vmin, vmax)
}

/// Return the uv bounds over every loop.
fn loops_bounds(loops: &[Loop]) -> (f64, f64, f64, f64) {
    let mut umin = 1e300f64;
    let mut umax = -1e300f64;
    let mut vmin = 1e300f64;
    let mut vmax = -1e300f64;

    for lp in loops {
        let (u0, u1, v0, v1) = loop_bounds(lp);
        umin = umin.min(u0);
        umax = umax.max(u1);
        vmin = vmin.min(v0);
        vmax = vmax.max(v1);
    }

    (umin, umax, vmin, vmax)
}

/// Mark the loop with the largest uv extent as outer when none is marked (OCCT and FreeCAD write FACE_BOUND for the outer boundary).
fn pick_outer_loop(loops: &mut [Loop]) {
    for l in loops.iter() {
        if l.is_outer {
            return;
        }
    }

    if loops.is_empty() {
        return;
    }

    let mut best = 0;
    let mut best_a = -1.0;

    for i in 0..loops.len() {
        let (u0, u1, v0, v1) = loop_bounds(&loops[i]);
        let a = if u1 > u0 && v1 > v0 {
            (u1 - u0) * (v1 - v0)
        } else {
            0.0
        };

        if a > best_a {
            best_a = a;
            best = i;
        }
    }

    loops[best].is_outer = true;
}

/// Reorder so outer loops come before inner ones.
fn outer_first(loops: &mut Vec<Loop>) {
    let mut ordered = Vec::new();

    for l in loops.iter() {
        if l.is_outer {
            ordered.push(l.clone());
        }
    }

    for l in loops.iter() {
        if !l.is_outer {
            ordered.push(l.clone());
        }
    }

    *loops = ordered;
}

/// Mean u of the samples of a loop, none when it has no samples.
fn loop_ucenter(lp: &Loop) -> Option<f64> {
    let mut sum = 0.0;
    let mut cnt = 0;

    for le in &lp.edges {
        for p in &le.uv {
            sum += p[0];
            cnt += 1;
        }
    }

    if cnt == 0 {
        return None;
    }

    Some(sum / cnt as f64)
}

/// Periods of the parameter chart: 4 in u for the analytic cylinder, the domain span of each closed direction of a B-spline surface, 0 when open.
fn surface_periods(proj: &Proj, proj_srf: &NurbsSurface) -> (f64, f64) {
    if proj.kind == 2 {
        return (4.0, 0.0);
    }

    if !proj_srf.is_valid() {
        return (0.0, 0.0);
    }

    let (du0, du1) = proj_srf.domain(0).unwrap_or_default();
    let (dv0, dv1) = proj_srf.domain(1).unwrap_or_default();
    let scale = proj_srf
        .point_at(du0, dv0)
        .unwrap_or_default()
        .distance(&proj_srf.point_at(du1, dv1).unwrap_or_default(), None)
        + 1e-9;
    let mut closed_u = true;
    let mut closed_v = true;

    for k in 0..=4 {
        let fu = du0 + (du1 - du0) * k as f64 / 4.0;
        let fv = dv0 + (dv1 - dv0) * k as f64 / 4.0;
        let a = proj_srf.point_at(du0, fv).unwrap_or_default();
        let b = proj_srf.point_at(du1, fv).unwrap_or_default();

        if a.distance(&b, None) > scale * 1e-6 {
            closed_u = false;
        }

        let c = proj_srf.point_at(fu, dv0).unwrap_or_default();
        let d = proj_srf.point_at(fu, dv1).unwrap_or_default();

        if c.distance(&d, None) > scale * 1e-6 {
            closed_v = false;
        }
    }

    (
        if closed_u { du1 - du0 } else { 0.0 },
        if closed_v { dv1 - dv0 } else { 0.0 },
    )
}

/// Shift each edge by whole periods so its traversal start meets the previous edge's end.
fn chain_loops(loops: &mut [Loop], tau_u: f64, tau_v: f64) {
    if tau_u <= 0.0 && tau_v <= 0.0 {
        return;
    }

    for lp in loops.iter_mut() {
        let mut prev_end = Point::new(0.0, 0.0, 0.0);
        let mut have_prev = false;

        for le in lp.edges.iter_mut() {
            if le.uv.is_empty() {
                continue;
            }

            let last = le.uv.len() - 1;
            let st = if le.reversed {
                le.uv[last].clone()
            } else {
                le.uv[0].clone()
            };
            let n = if have_prev && tau_u > 0.0 {
                ((prev_end[0] - st[0]) / tau_u).round() as i32
            } else {
                0
            };
            let m = if have_prev && tau_v > 0.0 {
                ((prev_end[1] - st[1]) / tau_v).round() as i32
            } else {
                0
            };

            for p in le.uv.iter_mut() {
                p[0] += n as f64 * tau_u;
                p[1] += m as f64 * tau_v;
            }

            prev_end = if le.reversed {
                le.uv[0].clone()
            } else {
                le.uv[last].clone()
            };
            have_prev = true;
        }
    }
}

/// Shift each inner loop by whole u periods onto the outer loop's u window.
fn center_inner_loops(loops: &mut [Loop], tau_u: f64) {
    if tau_u <= 0.0 {
        return;
    }

    let mut outer = None;

    for lp in loops.iter() {
        if lp.is_outer {
            outer = loop_ucenter(lp);
            break;
        }
    }

    let Some(outer) = outer else {
        return;
    };

    for lp in loops.iter_mut() {
        let center = if lp.is_outer { None } else { loop_ucenter(lp) };

        let Some(center) = center else {
            continue;
        };

        let n = ((outer - center) / tau_u).round() as i32;

        for le in lp.edges.iter_mut() {
            for p in le.uv.iter_mut() {
                p[0] += n as f64 * tau_u;
            }
        }
    }
}

/// Pending edges of a loop in traversal order: the exact pcurve when there is one, else the sampled polyline.
fn pending_of(lp: &Loop) -> Vec<PendingEdge> {
    let mut pl = Vec::new();

    for le in &lp.edges {
        let mut crv2d = le.pc2d.clone();

        if !le.exact || (le.reversed && !crv2d.reverse()) {
            let mut uv = le.uv.clone();

            if le.reversed {
                uv.reverse();
            }

            crv2d = polyline_nurbs(&uv, 2);
        }

        pl.push(PendingEdge {
            edge: le.edge_idx,
            reversed: le.reversed,
            c2d: crv2d,
        });
    }

    pl
}

/// Parameter-space images of 3D samples: the analytic projection, or a warm-started closest-point search on proj_srf.
fn uv_of_samples(proj: &Proj, proj_srf: &NurbsSurface, samples: &[Point]) -> Vec<Point> {
    let mut uv = Vec::new();

    if proj.kind != 0 {
        for s in samples {
            let (u, v) = project(proj, s);
            uv.push(Point::new(u, v, 0.0));
        }

        return uv;
    }

    if !proj_srf.is_valid() || samples.is_empty() {
        return vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)];
    }

    let (du0, du1) = proj_srf.domain(0).unwrap_or_default();
    let (dv0, dv1) = proj_srf.domain(1).unwrap_or_default();
    let wu = (du1 - du0) * 0.1;
    let wv = (dv1 - dv0) * 0.1;
    let mut d_ref = 0.0;
    let mut pu = 0.0;
    let mut pv = 0.0;

    for k in 0..samples.len() {
        let hit;

        if k == 0 {
            hit = Closest::surface_point(proj_srf, &samples[k], 0.0, 0.0, 0.0, 0.0);
            d_ref = hit.2;
        } else {
            let near =
                Closest::surface_point(proj_srf, &samples[k], pu - wu, pu + wu, pv - wv, pv + wv);

            hit = if near.2 > 10.0 * d_ref + 1e-9 {
                Closest::surface_point(proj_srf, &samples[k], 0.0, 0.0, 0.0, 0.0)
            } else {
                near
            };
        }

        let (u, v, _) = hit;
        uv.push(Point::new(u, v, 0.0));
        pu = u;
        pv = v;
    }

    uv
}

/// Map a surface parameter point into the window chart.
fn chart_point(an: &AnFace, w: &Window, p: &Point) -> Point {
    let angular = an.kind == 4 || an.kind == 5;

    Point::new(
        chart_u_of_angle(p[0], w.su0),
        if angular {
            chart_u_of_angle(p[1], w.sv0)
        } else {
            p[1]
        },
        0.0,
    )
}

/// Evaluate the analytic surface at a window chart point.
fn chart_eval(an: &AnFace, w: &Window, q: &Point) -> Point {
    let angular = an.kind == 4 || an.kind == 5;

    an_eval(
        an,
        (w.su0 as f64 + q[0]) * PI_2,
        if angular {
            (w.sv0 as f64 + q[1]) * PI_2
        } else {
            q[1]
        },
    )
}

/// Chart window of the loops; none when they are empty or wider than 16 quarter arcs.
fn analytic_window(loops: &[Loop], an: &AnFace) -> Option<Window> {
    let (smin, smax, tmin, tmax) = loops_bounds(loops);

    if smin > smax {
        return None;
    }

    let su0 = (smin / PI_2 + 1e-9).floor() as i32;
    let mut w = Window {
        su0,
        nsu: 1.max((smax / PI_2 - 1e-9).ceil() as i32 - su0),
        sv0: 0,
        nsv: 0,
        t0: tmin,
        t1: tmax,
    };

    if w.nsu > 16 {
        return None;
    }

    if an.kind == 4 || an.kind == 5 {
        w.sv0 = (tmin / PI_2 + 1e-9).floor() as i32;

        let mut sv1 = (tmax / PI_2 - 1e-9).ceil() as i32;

        if an.kind == 4 {
            w.sv0 = w.sv0.max(-1);
            sv1 = sv1.min(1);
        }

        w.nsv = 1.max(sv1 - w.sv0);

        if w.nsv > 16 {
            return None;
        }
    } else if tmax - tmin < 1e-12 {
        return None;
    }

    Some(w)
}

/// Canonical (s, t) where the loop left off: the end of its last edge, else the first sample that projects; false when neither exists.
fn st_start(an: &AnFace, lp: &Loop, ordered: &[Point]) -> (f64, f64, bool) {
    if !lp.edges.is_empty() && !lp.edges[lp.edges.len() - 1].uv.is_empty() {
        let pe = &lp.edges[lp.edges.len() - 1];
        let q = if pe.reversed {
            &pe.uv[0]
        } else {
            &pe.uv[pe.uv.len() - 1]
        };

        return (q[0], q[1], true);
    }

    for k in 0..ordered.len() {
        let (s, t, ok) = an_st_of(an, &ordered[k]);

        if ok {
            return (s, t, true);
        }
    }

    (0.0, 0.0, false)
}

/// Canonical (s, t) of 3D samples, s (and t on a torus) shifted by whole turns next to the sample before, the first next to (ps, pt).
fn st_unwrapped(an: &AnFace, ordered: &[Point], ps: f64, pt: f64, have_prev: bool) -> Vec<Point> {
    let mut st: Vec<Point> = Vec::new();

    for k in 0..ordered.len() {
        let (mut s, mut t, ok) = an_st_of(an, &ordered[k]);

        if !ok && (k > 0 || have_prev) {
            s = if k > 0 { st[k - 1][0] } else { ps };
        }

        let rs = if k > 0 {
            st[k - 1][0]
        } else if have_prev {
            ps
        } else {
            s
        };
        s -= 2.0 * PI * ((s - rs) / (2.0 * PI)).round();

        if an.kind == 5 {
            let rt = if k > 0 {
                st[k - 1][1]
            } else if have_prev {
                pt
            } else {
                t
            };
            t -= 2.0 * PI * ((t - rt) / (2.0 * PI)).round();
        }

        st.push(Point::new(s, t, 0.0));
    }

    st
}

/// BRep of one STEP shell, built face by face.
struct BRepBuilder<'a, 'b> {
    r: &'b mut StepReader<'a>, // Entity reader.
    brep: BRep,                // Brep under construction.
    vmap: HashMap<i32, usize>, // Brep vertex by VERTEX_POINT id.
    emap: HashMap<i32, usize>, // Brep edge by EDGE_CURVE id.
    face_refs: Vec<BRepRef>,   // Face references in file order.
}

impl<'a, 'b> BRepBuilder<'a, 'b> {
    /// Construct over an entity reader.
    fn new(reader: &'b mut StepReader<'a>) -> Self {
        BRepBuilder {
            r: reader,
            brep: BRep::new(),
            vmap: HashMap::new(),
            emap: HashMap::new(),
            face_refs: Vec::new(),
        }
    }

    /// Existing vertex within tol of q, else a new one.
    fn vertex_at(&mut self, q: &Point, tol: f64) -> usize {
        for i in 0..self.brep.m_vertices.len() {
            if self.brep.m_vertices[i].point.distance(q, None) <= tol {
                return i;
            }
        }

        self.brep.add_vertex(q, 0.0)
    }

    /// The second use of an edge on the same surface is a seam: the forward use keeps curve_2d_index, the reversed one curve_2d_index_2.
    fn attach_pcurve(&mut self, edge: usize, si: usize, c2: usize, reversed_use: bool) {
        for pc in self.brep.m_edges[edge].pcurves.iter_mut() {
            if pc.surface_index != si as i32 {
                continue;
            }

            if reversed_use {
                pc.curve_2d_index_2 = c2 as i32;
            } else {
                pc.curve_2d_index_2 = pc.curve_2d_index;
                pc.curve_2d_index = c2 as i32;
            }

            return;
        }

        self.brep.add_pcurve(edge, si, c2 as i32, -1);
    }

    /// Face from its surface and loops (outer first), oriented in the shell by reversed_face.
    fn finish_face(&mut self, si: usize, reversed_face: bool, loops: &[Vec<PendingEdge>]) {
        let mut wires = Vec::new();

        for lp in loops {
            let mut refs = Vec::new();

            for pe in lp {
                let mut c = pe.c2d.clone();

                if !pe.reversed || c.reverse() {
                    let c2 = self.brep.add_curve_2d(&c);
                    self.attach_pcurve(pe.edge, si, c2, pe.reversed);
                }

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
                wires.push(BRepRef::new(
                    self.brep.add_wire(&refs) as i32,
                    BRepOrientation::Forward,
                ));
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
    }

    /// Return the brep vertex of a VERTEX_POINT, creating it once.
    fn get_vertex(&mut self, vp_id: i32) -> usize {
        if let Some(vi) = self.vmap.get(&vp_id) {
            return *vi;
        }

        let pt = self
            .r
            .get_vertex_point(vp_id)
            .unwrap_or(Point::new(0.0, 0.0, 0.0));
        let vi = self.brep.add_vertex(&pt, 0.0);
        self.vmap.insert(vp_id, vi);

        vi
    }

    /// Exact 3D curve of an edge basis: the B-spline itself or a rational arc of a CIRCLE, invalid otherwise.
    fn edge_curve(&mut self, curve_id: i32, vs: &Point, ve: &Point) -> NurbsCurve {
        let Some(e) = self.r.get(curve_id) else {
            return NurbsCurve::default();
        };

        if e.has("B_SPLINE_CURVE_WITH_KNOTS") {
            return self.r.get_nurbs_curve(curve_id);
        }

        let Some(circle) = e.find("CIRCLE") else {
            return NurbsCurve::default();
        };

        let ax_ref = first_ref(&circle.params);
        let rr = nums(&circle.params);
        let rad = if rr.is_empty() { 0.0 } else { rr[0] };
        let a = self.r.get_axis2(ax_ref);

        if ax_ref < 0 || rad <= 0.0 || !a.ok {
            return NurbsCurve::default();
        }

        circle_nurbs(&a, rad, vs, ve)
    }

    /// BRep edge of an EDGE_CURVE, made once: exact curve when possible, else a sampled polyline.
    fn get_edge(&mut self, ec_id: i32) -> Option<usize> {
        if let Some(ei) = self.emap.get(&ec_id) {
            return Some(*ei);
        }

        let refs = edge_refs(self.r, ec_id);

        if refs.len() < 3 {
            return None;
        }

        let sv = self.get_vertex(refs[0]);
        let ev = self.get_vertex(refs[1]);
        let curve_id = self.r.basis_curve_of(refs[2]);
        let vs = self.brep.m_vertices[sv].point.clone();
        let ve = self.brep.m_vertices[ev].point.clone();
        let mut crv3d = self.edge_curve(curve_id, &vs, &ve);

        if !crv3d.is_valid() {
            crv3d = polyline_nurbs(&self.r.sample_curve(curve_id, &vs, &ve, 16), 3);
        }

        let c3 = self.brep.add_curve_3d(&crv3d);
        let ei = self.brep.add_edge(c3 as i32, sv as i32, ev as i32);
        self.emap.insert(ec_id, ei);

        Some(ei)
    }

    /// Projection fallback: 3D samples of an edge mapped to canonical (s, t), branch-unwrapped along the loop traversal.
    fn st_projected(
        &mut self,
        an: &AnFace,
        geom_id: i32,
        edge_idx: usize,
        rev: bool,
        lp: &Loop,
    ) -> Vec<Point> {
        let be = &self.brep.m_edges[edge_idx];
        let vs = self.brep.m_vertices[be.start_vertex as usize].point.clone();
        let ve = self.brep.m_vertices[be.end_vertex as usize].point.clone();
        let mut ordered = self.r.sample_curve(geom_id, &vs, &ve, 48);

        if ordered.len() < 2 {
            return Vec::new();
        }

        if rev {
            ordered.reverse();
        }

        let (ps, pt, have_prev) = st_start(an, lp, &ordered);
        let mut st = st_unwrapped(an, &ordered, ps, pt, have_prev);

        if rev {
            st.reverse();
        }

        st
    }

    /// Loops of an analytic face with canonical (s, t) samples from the file pcurves or from projection.
    fn analytic_loops(
        &mut self,
        bound_refs: &[i32],
        surface_ref: i32,
        an: &AnFace,
        loops: &mut Vec<Loop>,
    ) -> bool {
        for &bid in bound_refs {
            let Some(b) = bound_loop(self.r, bid) else {
                continue;
            };

            let mut lp = Loop {
                is_outer: b.is_outer,
                projected: false,
                edges: Vec::new(),
            };

            for &oe_id in &b.oe_refs {
                let (ec_ref, oe_orient) = oriented_edge(self.r, oe_id);

                let Some(edge_idx) = self.get_edge(ec_ref) else {
                    continue;
                };

                let geom_id = edge_geom_id(self.r, ec_ref);
                let mut le = LoopEdge {
                    edge_idx,
                    reversed: oe_orient != b.orient,
                    uv: Vec::new(),
                    pc2d: NurbsCurve::default(),
                    exact: false,
                };

                if geom_id >= 0 {
                    le.uv = self
                        .r
                        .pcurve_st_samples(geom_id, surface_ref, oe_orient, 48);
                }

                if le.uv.len() < 2 {
                    lp.projected = true;
                    le.uv = self.st_projected(an, geom_id, edge_idx, le.reversed, &lp);

                    if le.uv.is_empty() {
                        return false;
                    }
                }

                lp.edges.push(le);
            }

            if !lp.edges.is_empty() {
                loops.push(lp);
            }

            pick_outer_loop(loops);
        }

        !loops.is_empty()
    }

    /// Pending edges of one loop in the chart, plus a degenerated edge across each pole or apex gap between consecutive edges.
    fn analytic_pending(
        &mut self,
        lp: &Loop,
        an: &AnFace,
        w: &Window,
        scale3: f64,
    ) -> Vec<PendingEdge> {
        let period = 4.0;
        let mut chains: Vec<Vec<Point>> = Vec::new();

        for le in &lp.edges {
            let mut uv = Vec::new();

            for p in &le.uv {
                uv.push(chart_point(an, w, p));
            }

            if le.reversed {
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

            let prev = &chains[k - 1][chains[k - 1].len() - 1];
            let n = ((prev[0] - chains[k][0][0]) / period).round() as i32;
            let m = if an.kind == 5 {
                ((prev[1] - chains[k][0][1]) / period).round() as i32
            } else {
                0
            };

            for p in chains[k].iter_mut() {
                p[0] += n as f64 * period;
                p[1] += m as f64 * period;
            }
        }

        let mut pl = Vec::new();

        for k in 0..lp.edges.len() {
            if chains[k].len() < 2 {
                continue;
            }

            pl.push(PendingEdge {
                edge: lp.edges[k].edge_idx,
                reversed: lp.edges[k].reversed,
                c2d: polyline_nurbs(&chains[k], 2),
            });

            let nxt = &chains[(k + 1) % chains.len()];

            if nxt.is_empty() {
                continue;
            }

            let a2 = chains[k][chains[k].len() - 1].clone();
            let b2 = nxt[0].clone();

            if (a2[0] - b2[0]).abs() + (a2[1] - b2[1]).abs() <= 1e-7 {
                continue;
            }

            let p3a = chart_eval(an, w, &a2);
            let p3b = chart_eval(an, w, &b2);

            if p3a.distance(&p3b, None) >= scale3 * 1e-6 {
                continue;
            }

            let vd = self.vertex_at(&p3a, scale3 * 1e-6);
            pl.push(PendingEdge {
                edge: self.brep.add_edge(-1, vd as i32, vd as i32),
                reversed: false,
                c2d: polyline_nurbs(&[a2, b2], 2),
            });
        }

        pl
    }

    /// Face on a cylinder, cone, sphere or torus: the exact kernel window with the file pcurves bound in it; false falls back to projection.
    fn add_face_analytic(
        &mut self,
        bound_refs: &[i32],
        surface_ref: i32,
        same_sense: bool,
        an: &AnFace,
    ) -> bool {
        let mut loops = Vec::new();

        if !self.analytic_loops(bound_refs, surface_ref, an, &mut loops) {
            return false;
        }

        let Some(w) = analytic_window(&loops, an) else {
            return false;
        };

        let srf = build_analytic_nurbs(an, w.su0, w.nsu, w.t0, w.t1, w.sv0, w.nsv);

        if !srf.is_valid() {
            return false;
        }

        let scale3 = an.radius + an.r2.abs() + 1.0;
        let srf_idx = self.brep.add_surface(&srf);
        outer_first(&mut loops);

        let mut pending = Vec::new();

        for lp in &loops {
            pending.push(self.analytic_pending(lp, an, &w, scale3));
        }

        self.finish_face(srf_idx, !same_sense, &pending);

        true
    }

    /// Point of a VERTEX_POINT, far away when missing.
    fn step_point_of(&mut self, vp_id: i32) -> Point {
        self.r
            .get_vertex_point(vp_id)
            .unwrap_or(Point::new(1e300, 1e300, 1e300))
    }

    /// The file vertex at q when one of the given VERTEX_POINTs sits there, else a new vertex.
    fn topo_vertex_at(&mut self, vl_vertex_ids: &[i32], q: &Point, tol: f64) -> usize {
        for &vid in vl_vertex_ids {
            if self.step_point_of(vid).distance(q, None) <= tol {
                return self.get_vertex(vid);
            }
        }

        self.brep.add_vertex(q, 0.0)
    }

    /// Kernel surface of a VERTEX_LOOP face: the whole sphere or torus, the B-spline itself, invalid otherwise.
    fn vertex_loop_surface(&mut self, surface_ref: i32) -> NurbsSurface {
        let an = self.r.get_analytic_srf(surface_ref);

        if an.kind == 4 {
            return build_analytic_nurbs(&an, 0, 4, 0.0, 0.0, -1, 2);
        }

        if an.kind == 5 {
            return build_analytic_nurbs(&an, 0, 4, 0.0, 0.0, 0, 4);
        }

        if an.kind == 0 {
            return self.r.get_nurbs_surface(surface_ref);
        }

        NurbsSurface::default()
    }

    /// Wire of a sphere-like surface: a degenerated edge at each pole and the seam used both ways; empty when the seam is invalid.
    fn pole_wire(
        &mut self,
        srf: &NurbsSurface,
        grid: &[Point],
        vl_vertex_ids: &[i32],
        tol: f64,
    ) -> Vec<PendingEdge> {
        let (u0, u1) = srf.domain(0).unwrap_or_default();
        let (v0, v1) = srf.domain(1).unwrap_or_default();
        let v_lo = self.topo_vertex_at(vl_vertex_ids, &grid[0], tol) as i32;
        let v_hi = self.topo_vertex_at(vl_vertex_ids, &grid[NS - 1], tol) as i32;
        let seam = srf.iso_curve(1, u0).unwrap_or_default();

        if !seam.is_valid() {
            return Vec::new();
        }

        let c_seam = self.brep.add_curve_3d(&seam);
        let ei_seam = self.brep.add_edge(c_seam as i32, v_lo, v_hi);
        let ei_lo = self.brep.add_edge(-1, v_lo, v_lo);
        let ei_hi = self.brep.add_edge(-1, v_hi, v_hi);

        vec![
            PendingEdge {
                edge: ei_lo,
                reversed: false,
                c2d: uv_line(u0, v0, u1, v0),
            },
            PendingEdge {
                edge: ei_seam,
                reversed: false,
                c2d: uv_line(u1, v0, u1, v1),
            },
            PendingEdge {
                edge: ei_hi,
                reversed: false,
                c2d: uv_line(u1, v1, u0, v1),
            },
            PendingEdge {
                edge: ei_seam,
                reversed: true,
                c2d: uv_line(u0, v1, u0, v0),
            },
        ]
    }

    /// Wire of a torus-like surface: the u seam and the v seam each used both ways; empty when a seam is invalid.
    fn seam_wire(
        &mut self,
        srf: &NurbsSurface,
        grid: &[Point],
        vl_vertex_ids: &[i32],
        tol: f64,
    ) -> Vec<PendingEdge> {
        let (u0, u1) = srf.domain(0).unwrap_or_default();
        let (v0, v1) = srf.domain(1).unwrap_or_default();
        let vtx = self.topo_vertex_at(vl_vertex_ids, &grid[0], tol) as i32;
        let c_u = srf.iso_curve(1, u0).unwrap_or_default();
        let c_v = srf.iso_curve(0, v0).unwrap_or_default();

        if !c_u.is_valid() || !c_v.is_valid() {
            return Vec::new();
        }

        let cu = self.brep.add_curve_3d(&c_u);
        let cv = self.brep.add_curve_3d(&c_v);
        let ei_u = self.brep.add_edge(cu as i32, vtx, vtx);
        let ei_v = self.brep.add_edge(cv as i32, vtx, vtx);

        vec![
            PendingEdge {
                edge: ei_v,
                reversed: false,
                c2d: uv_line(u0, v0, u1, v0),
            },
            PendingEdge {
                edge: ei_u,
                reversed: false,
                c2d: uv_line(u1, v0, u1, v1),
            },
            PendingEdge {
                edge: ei_v,
                reversed: true,
                c2d: uv_line(u1, v1, u0, v1),
            },
            PendingEdge {
                edge: ei_u,
                reversed: true,
                c2d: uv_line(u0, v1, u0, v0),
            },
        ]
    }

    /// Face bounded only by VERTEX_LOOPs: the whole surface, with seam and pole edges read off the surface (sphere-like or torus-like).
    fn add_face_vertex_loop(
        &mut self,
        vl_vertex_ids: &[i32],
        surface_ref: i32,
        same_sense: bool,
    ) -> bool {
        let srf = self.vertex_loop_surface(surface_ref);

        if !srf.is_valid() {
            return false;
        }

        let grid = surface_grid(&srf, NS);
        let tol = grid_scale(&grid) * 1e-7;

        if tol.is_nan() || tol <= 0.0 {
            return false;
        }

        let closed_u = grid_closed(&grid, NS, tol, true);
        let closed_v = grid_closed(&grid, NS, tol, false);
        let degen_v0 = grid_degenerate(&grid, NS, tol, 0);
        let degen_v1 = grid_degenerate(&grid, NS, tol, NS - 1);

        if !closed_u || !((degen_v0 && degen_v1) || closed_v) {
            return false;
        }

        let si = self.brep.add_surface(&srf);
        let wire = if degen_v0 && degen_v1 {
            self.pole_wire(&srf, &grid, vl_vertex_ids, tol)
        } else {
            self.seam_wire(&srf, &grid, vl_vertex_ids, tol)
        };

        if wire.is_empty() {
            return false;
        }

        self.finish_face(si, !same_sense, &[wire]);

        true
    }

    /// VERTEX_POINT ids of the VERTEX_LOOP bounds; empty when any bound is an EDGE_LOOP.
    fn vertex_loop_ids(&self, bound_refs: &[i32]) -> Vec<i32> {
        let mut ids = Vec::new();

        for &bid in bound_refs {
            let bent = self.r.get(bid);
            let mut bsub = find_in(bent, "FACE_OUTER_BOUND");

            if bsub.is_none() {
                bsub = find_in(bent, "FACE_BOUND");
            }

            let lent = match bsub {
                Some(bsub) => self.r.get(first_ref(&bsub.params)),
                None => None,
            };

            let Some(lent) = lent else {
                continue;
            };

            if lent.has("EDGE_LOOP") {
                return Vec::new();
            }

            let id = match lent.find("VERTEX_LOOP") {
                Some(vl) => first_ref(&vl.params),
                None => -1,
            };

            if id >= 0 {
                ids.push(id);
            }
        }

        ids
    }

    /// Loops of a face on a projected surface: uv samples in curve order, exact pcurves under an affine projector.
    fn projected_loops(
        &mut self,
        bound_refs: &[i32],
        proj: &Proj,
        proj_srf: &NurbsSurface,
        loops: &mut Vec<Loop>,
    ) {
        let n = if proj_srf.is_valid() { 48 } else { 16 };
        let mut exact = Proj::new();

        if proj.kind == 1 {
            exact = proj.clone();
        } else if proj.kind == 0 {
            exact = bilinear_projector(proj_srf);
        }

        for &bid in bound_refs {
            let Some(b) = bound_loop(self.r, bid) else {
                continue;
            };

            let mut lp = Loop {
                is_outer: b.is_outer,
                projected: false,
                edges: Vec::new(),
            };

            for &oe_id in &b.oe_refs {
                let (ec_ref, oe_orient) = oriented_edge(self.r, oe_id);

                let Some(edge_idx) = self.get_edge(ec_ref) else {
                    continue;
                };

                let be = &self.brep.m_edges[edge_idx];
                let vs = self.brep.m_vertices[be.start_vertex as usize].point.clone();
                let ve = self.brep.m_vertices[be.end_vertex as usize].point.clone();
                let c3 = be.curve_3d_index;
                let samples = self
                    .r
                    .sample_curve(edge_geom_id(self.r, ec_ref), &vs, &ve, n);
                let mut le = LoopEdge {
                    edge_idx,
                    reversed: oe_orient != b.orient,
                    uv: uv_of_samples(proj, proj_srf, &samples),
                    pc2d: NurbsCurve::default(),
                    exact: false,
                };

                if proj.kind == 2 {
                    unwrap_seam(&mut le.uv);
                }

                if c3 >= 0 {
                    le.pc2d = exact_pcurve(&exact, &self.brep.m_curves_3d[c3 as usize]);
                }

                le.exact = le.pc2d.is_valid();
                lp.edges.push(le);
            }

            if !lp.edges.is_empty() {
                loops.push(lp);
            }

            pick_outer_loop(loops);
        }
    }

    /// ADVANCED_FACE: vertex-loop face, analytic face, or projection onto the plane, cylinder chart or B-spline surface.
    fn add_face(&mut self, face_id: i32) {
        let Some(face) = find_in(self.r.get(face_id), "ADVANCED_FACE") else {
            return;
        };

        let bound_refs = list_refs(&face.params);
        let surface_ref = first_ref(&face.params);
        let same_sense = last_flag(&face.params, true);
        let vl_ids = self.vertex_loop_ids(&bound_refs);

        if !vl_ids.is_empty() && self.add_face_vertex_loop(&vl_ids, surface_ref, same_sense) {
            return;
        }

        let an = self.r.get_analytic_srf(surface_ref);

        if an.kind >= 2 && self.add_face_analytic(&bound_refs, surface_ref, same_sense, &an) {
            return;
        }

        let proj = self.r.get_projector(surface_ref);
        let proj_srf = if proj.kind == 0 {
            self.r.fill_surface(surface_ref, 0.0, 1.0, 0.0, 1.0)
        } else {
            NurbsSurface::default()
        };

        let mut loops = Vec::new();
        self.projected_loops(&bound_refs, &proj, &proj_srf, &mut loops);

        let (tau_u, tau_v) = surface_periods(&proj, &proj_srf);
        chain_loops(&mut loops, tau_u, tau_v);
        center_inner_loops(&mut loops, tau_u);

        let mut srf = proj_srf;

        if !srf.is_valid() {
            let (mut umin, mut umax, mut vmin, mut vmax) = loops_bounds(&loops);

            if umin > umax {
                umin = -1.0;
                umax = 1.0;
                vmin = -1.0;
                vmax = 1.0;
            }

            srf = self.r.fill_surface(surface_ref, umin, umax, vmin, vmax);
        }

        let srf_idx = self.brep.add_surface(&srf);
        outer_first(&mut loops);

        let mut pending = Vec::new();

        for lp in &loops {
            pending.push(pending_of(lp));
        }

        self.finish_face(srf_idx, !same_sense, &pending);
    }

    /// BRep of a CLOSED_SHELL (one solid) or OPEN_SHELL (one shell), empty for anything else.
    fn build_from_shell(mut self, shell_id: i32) -> BRep {
        let sent = self.r.get(shell_id);
        let mut shell = find_in(sent, "CLOSED_SHELL");

        if shell.is_none() {
            shell = find_in(sent, "OPEN_SHELL");
        }

        let Some(shell) = shell else {
            return BRep::new();
        };

        self.brep.name = "step_brep".to_string();

        for f in list_refs(&shell.params) {
            self.add_face(f);
        }

        if !self.face_refs.is_empty() {
            let sh = self.brep.add_shell(&self.face_refs);

            if find_in(sent, "CLOSED_SHELL").is_some() {
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
    let Some(epos) = sci.find('e') else {
        return sci;
    };

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

/// ISO 10303-21 REAL: a decimal point in the mantissa and an uppercase E.
fn fmt(v: f64) -> String {
    let mut s = if v.abs() < 1e15 && v == (v as i64) as f64 {
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

/// Format integers as a STEP list.
fn fmt_int_list(items: &[i32]) -> String {
    let mut s = String::from("(");

    for i in 0..items.len() {
        s += &format!("{}{}", if i > 0 { "," } else { "" }, items[i]);
    }

    s + ")"
}

/// Format doubles as a STEP list.
fn fmt_dbl_list(items: &[f64]) -> String {
    let mut s = String::from("(");

    for i in 0..items.len() {
        s += &format!("{}{}", if i > 0 { "," } else { "" }, fmt(items[i]));
    }

    s + ")"
}

/// Format ids as a STEP reference list.
fn fmt_ref_list(ids: &[i32]) -> String {
    let mut s = String::from("(");

    for i in 0..ids.len() {
        s += &format!("{}{}", if i > 0 { ",#" } else { "#" }, ids[i]);
    }

    s + ")"
}

/// Format id rows as a STEP list of reference lists.
fn fmt_ref_grid(rows: &[Vec<i32>]) -> String {
    let mut s = String::from("(");

    for i in 0..rows.len() {
        s += &format!("{}{}", if i > 0 { "," } else { "" }, fmt_ref_list(&rows[i]));
    }

    s + ")"
}

/// Format double rows as a STEP list of lists.
fn fmt_dbl_grid(rows: &[Vec<f64>]) -> String {
    let mut s = String::from("(");

    for i in 0..rows.len() {
        s += &format!("{}{}", if i > 0 { "," } else { "" }, fmt_dbl_list(&rows[i]));
    }

    s + ")"
}

/// Entity lines of a STEP file under construction.
struct StepWriter {
    next_id: i32,       // Next free entity id.
    lines: Vec<String>, // Emitted entity lines.
}

impl StepWriter {
    /// Construct with no entities.
    fn new() -> Self {
        StepWriter {
            next_id: 1,
            lines: Vec::new(),
        }
    }

    /// Return the next free entity id.
    fn new_id(&mut self) -> i32 {
        let id = self.next_id;
        self.next_id += 1;

        id
    }

    /// Emit "#id=body;" with a fresh id and return the id.
    fn write_raw(&mut self, body: &str) -> i32 {
        let id = self.new_id();
        self.lines.push(format!("#{}={};", id, body));

        id
    }

    /// Emit a CARTESIAN_POINT and return its id.
    fn write_point(&mut self, x: f64, y: f64, z: f64) -> i32 {
        self.write_raw(&format!(
            "CARTESIAN_POINT('',({},{},{}))",
            fmt(x),
            fmt(y),
            fmt(z)
        ))
    }

    /// B_SPLINE_CURVE_WITH_KNOTS, as a complete complex instance when rational; -1 for an invalid curve.
    fn write_nurbs_curve(&mut self, nc: &NurbsCurve) -> i32 {
        if !nc.is_valid() {
            return -1;
        }

        let mut pt_ids = Vec::new();
        let mut weights = Vec::new();

        for i in 0..nc.cv_count() {
            let (x, y, z, mut w) = nc.get_cv_4d(i).unwrap_or_default();

            if w.abs() < 1e-14 {
                w = 1.0;
            }

            pt_ids.push(self.write_point(x / w, y / w, z / w));
            weights.push(w);
        }

        let (kvals, kmults) = compress_knots(&full_from_internal(&nc.m_nurbsknot));
        let degree = nc.m_order - 1;

        if !nc.is_rational() {
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

    /// B_SPLINE_SURFACE_WITH_KNOTS, as a complete complex instance when rational; -1 for an invalid surface.
    fn write_nurbs_surface(&mut self, srf: &NurbsSurface) -> i32 {
        if !srf.is_valid() {
            return -1;
        }

        let cv_u = srf.m_cv_count[0];
        let cv_v = srf.m_cv_count[1];
        let mut pt_ids = vec![vec![0i32; cv_v]; cv_u];
        let mut weight_grid = vec![vec![1.0f64; cv_v]; cv_u];

        for u in 0..cv_u {
            for v in 0..cv_v {
                let Some((x, y, z, mut w)) = srf.get_cv_4d(u, v) else {
                    return -1;
                };

                if w.abs() < 1e-14 {
                    w = 1.0;
                }

                pt_ids[u][v] = self.write_point(x / w, y / w, z / w);
                weight_grid[u][v] = w;
            }
        }

        let (ku_vals, ku_mults) = compress_knots(&full_from_internal(&srf.m_nurbsknot[0]));
        let (kv_vals, kv_mults) = compress_knots(&full_from_internal(&srf.m_nurbsknot[1]));
        let degrees = format!("{},{}", srf.m_order[0] - 1, srf.m_order[1] - 1);
        let knots = format!(
            "{},{},{},{}",
            fmt_int_list(&ku_mults),
            fmt_int_list(&kv_mults),
            fmt_dbl_list(&ku_vals),
            fmt_dbl_list(&kv_vals)
        );

        if !srf.is_rational() {
            return self.write_raw(&format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',{},{},.UNSPECIFIED.,.F.,.F.,.U.,{},.UNSPECIFIED.)",
                degrees,
                fmt_ref_grid(&pt_ids),
                knots
            ));
        }

        self.write_raw(&format!(
            "(BOUNDED_SURFACE()B_SPLINE_SURFACE({},{},.UNSPECIFIED.,.F.,.F.,.U.)B_SPLINE_SURFACE_WITH_KNOTS({},.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE({})REPRESENTATION_ITEM('')SURFACE())",
            degrees,
            fmt_ref_grid(&pt_ids),
            knots,
            fmt_dbl_grid(&weight_grid)
        ))
    }

    /// FACE_OUTER_BOUND or FACE_BOUND of one closed trim loop: its 3D image sampled as a polyline edge on one vertex.
    fn write_loop_as_face_bound(
        &mut self,
        trimmed: &NurbsSurfaceTrimmed,
        loop_2d: &NurbsCurve,
        is_outer: bool,
    ) -> i32 {
        if !loop_2d.is_valid() {
            return -1;
        }

        let mut pts3d = Vec::new();

        for uv in sample_nurbs(loop_2d, 2.max(loop_2d.cv_count() as i32 * 2)) {
            pts3d.push(trimmed.m_surface.point_at(uv[0], uv[1]).unwrap_or_default());
        }

        let pt = self.write_point(pts3d[0][0], pts3d[0][1], pts3d[0][2]);
        let v0 = self.write_raw(&format!("VERTEX_POINT('',#{})", pt));

        let crv3d = self.write_nurbs_curve(&polyline_nurbs(&pts3d, 3));

        if crv3d < 0 {
            return -1;
        }

        if self.write_nurbs_curve(loop_2d) < 0 {
            return -1;
        }

        let ec = self.write_raw(&format!("EDGE_CURVE('',#{},#{},#{},.T.)", v0, v0, crv3d));
        let oe = self.write_raw(&format!("ORIENTED_EDGE('',*,*,#{},.T.)", ec));
        let el = self.write_raw(&format!("EDGE_LOOP('',(#{}))", oe));

        self.write_raw(&format!(
            "{}('',#{},.T.)",
            if is_outer {
                "FACE_OUTER_BOUND"
            } else {
                "FACE_BOUND"
            },
            el
        ))
    }

    /// ADVANCED_FACE of a trimmed surface; -1 when the surface or the outer loop cannot be written.
    fn write_trimmed_face(&mut self, trimmed: &NurbsSurfaceTrimmed) -> i32 {
        let srf_id = self.write_nurbs_surface(&trimmed.m_surface);

        if srf_id < 0 {
            return -1;
        }

        let outer = trimmed.m_outer_loop.clone().unwrap_or_default();
        let mut bounds = vec![self.write_loop_as_face_bound(trimmed, &outer, true)];

        if bounds[0] < 0 {
            return -1;
        }

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

    /// AP214 surface-color chain; returns the PRESENTATION_STYLE_ASSIGNMENT for STYLED_ITEMs.
    fn color_style(&mut self, r: f64, g: f64, b: f64) -> i32 {
        let c = self.write_raw(&format!("COLOUR_RGB('',{},{},{})", fmt(r), fmt(g), fmt(b)));
        let fc = self.write_raw(&format!("FILL_AREA_STYLE_COLOUR('',#{})", c));
        let fa = self.write_raw(&format!("FILL_AREA_STYLE('',(#{}))", fc));
        let sf = self.write_raw(&format!("SURFACE_STYLE_FILL_AREA(#{})", fa));
        let ss = self.write_raw(&format!("SURFACE_SIDE_STYLE('',(#{}))", sf));
        let su = self.write_raw(&format!("SURFACE_STYLE_USAGE(.BOTH.,#{})", ss));

        self.write_raw(&format!("PRESENTATION_STYLE_ASSIGNMENT((#{}))", su))
    }

    /// CLOSED_SHELL + MANIFOLD_SOLID_BREP or OPEN_SHELL + SHELL_BASED_SURFACE_MODEL over the faces; -1 when there are none.
    fn write_body(&mut self, faces: &[i32], closed: bool) -> i32 {
        if faces.is_empty() {
            return -1;
        }

        let shell = self.write_raw(&format!(
            "{}('',{})",
            if closed { "CLOSED_SHELL" } else { "OPEN_SHELL" },
            fmt_ref_list(faces)
        ));

        if closed {
            return self.write_raw(&format!("MANIFOLD_SOLID_BREP('',#{})", shell));
        }

        self.write_raw(&format!("SHELL_BASED_SURFACE_MODEL('',(#{}))", shell))
    }

    /// AP214 PRODUCT and SHAPE_DEFINITION_REPRESENTATION skeleton importers need to find the bodies; uncertainty is the sewing tolerance.
    fn finish_product(
        &mut self,
        bodies: &[i32],
        closed: bool,
        name: &str,
        mut uncertainty: f64,
        styled_items: &[i32],
    ) {
        let o = self.write_point(0.0, 0.0, 0.0);
        let dz = self.write_raw("DIRECTION('',(0.,0.,1.))");
        let dx = self.write_raw("DIRECTION('',(1.,0.,0.))");
        let ax = self.write_raw(&format!("AXIS2_PLACEMENT_3D('',#{},#{},#{})", o, dz, dx));
        let lu = self.write_raw("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))");
        let au = self.write_raw("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
        let su = self.write_raw("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");

        if !uncertainty.is_finite() || uncertainty <= 0.0 {
            uncertainty = 1e-6;
        }

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
        self.write_raw(&format!(
            "APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{})",
            ac
        ));

        let pc = self.write_raw(&format!("PRODUCT_CONTEXT('',#{},'mechanical')", ac));
        let pr = self.write_raw(&format!("PRODUCT('{}','{}','',(#{}))", name, name, pc));
        let pf = self.write_raw(&format!("PRODUCT_DEFINITION_FORMATION('','',#{})", pr));
        let dc = self.write_raw(&format!(
            "PRODUCT_DEFINITION_CONTEXT('part definition',#{},'design')",
            ac
        ));
        let pd = self.write_raw(&format!("PRODUCT_DEFINITION('design','',#{},#{})", pf, dc));

        let ps = self.write_raw(&format!("PRODUCT_DEFINITION_SHAPE('','',#{})", pd));
        let mut rep_type = "MANIFOLD_SURFACE_SHAPE_REPRESENTATION";

        if bodies.is_empty() {
            rep_type = "SHAPE_REPRESENTATION";
        } else if closed {
            rep_type = "ADVANCED_BREP_SHAPE_REPRESENTATION";
        }

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

    /// Return the complete STEP text with header and data sections.
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

/// Write one BRep into a StepWriter: vertices, edges and surfaces once each, degenerated edges omitted, a wire of only degenerated edges as a VERTEX_LOOP.
struct BRepEmitter<'a> {
    w: &'a mut StepWriter,  // Entity writer.
    brep: &'a BRep,         // Brep being emitted.
    vid: HashMap<i32, i32>, // VERTEX_POINT id by vertex.
    eid: HashMap<i32, i32>, // EDGE_CURVE id by edge.
    sid: HashMap<i32, i32>, // Surface id by surface.
}

impl<'a> BRepEmitter<'a> {
    /// Construct over a writer and the brep to emit.
    fn new(writer: &'a mut StepWriter, b: &'a BRep) -> Self {
        BRepEmitter {
            w: writer,
            brep: b,
            vid: HashMap::new(),
            eid: HashMap::new(),
            sid: HashMap::new(),
        }
    }

    /// Return the VERTEX_POINT id of a brep vertex, emitting it once.
    fn vertex_id(&mut self, vi: i32) -> i32 {
        if let Some(id) = self.vid.get(&vi) {
            return *id;
        }

        let p = &self.brep.m_vertices[vi as usize].point;
        let pt = self.w.write_point(p[0], p[1], p[2]);
        let id = self.w.write_raw(&format!("VERTEX_POINT('',#{})", pt));
        self.vid.insert(vi, id);

        id
    }

    /// Return the EDGE_CURVE id of a brep edge, emitting it once.
    fn edge_id(&mut self, ei: i32) -> i32 {
        if let Some(id) = self.eid.get(&ei) {
            return *id;
        }

        let brep = self.brep;
        let e = &brep.m_edges[ei as usize];
        let c = self
            .w
            .write_nurbs_curve(&brep.m_curves_3d[e.curve_3d_index as usize]);

        if c < 0 {
            self.eid.insert(ei, -1);

            return -1;
        }

        let sv = self.vertex_id(e.start_vertex);
        let ev = self.vertex_id(e.end_vertex);
        let id = self
            .w
            .write_raw(&format!("EDGE_CURVE('',#{},#{},#{},.T.)", sv, ev, c));
        self.eid.insert(ei, id);

        id
    }

    /// Return the surface id of a brep surface, emitting it once.
    fn surface_id(&mut self, si: i32) -> i32 {
        if let Some(id) = self.sid.get(&si) {
            return *id;
        }

        let id = self
            .w
            .write_nurbs_surface(&self.brep.m_surfaces[si as usize]);
        self.sid.insert(si, id);

        id
    }

    /// EDGE_LOOP of the non-degenerated edges, a VERTEX_LOOP when there are none, -1 for an empty wire.
    fn wire_id(&mut self, wire: &BRepRef) -> i32 {
        let brep = self.brep;
        let mut oes = Vec::new();
        let mut any_vertex = -1;

        for er in brep.wire_edges(wire) {
            let e = &brep.m_edges[er.index as usize];

            if any_vertex < 0 {
                any_vertex = e.start_vertex;
            }

            if e.degenerated {
                continue;
            }

            let ec = self.edge_id(er.index);

            if ec < 0 {
                continue;
            }

            let sense = if er.orientation == BRepOrientation::Forward {
                "T"
            } else {
                "F"
            };
            oes.push(
                self.w
                    .write_raw(&format!("ORIENTED_EDGE('',*,*,#{},.{}.)", ec, sense)),
            );
        }

        if !oes.is_empty() {
            return self
                .w
                .write_raw(&format!("EDGE_LOOP('',{})", fmt_ref_list(&oes)));
        }

        if any_vertex >= 0 {
            let v = self.vertex_id(any_vertex);

            return self.w.write_raw(&format!("VERTEX_LOOP('',#{})", v));
        }

        -1
    }

    /// Return the ADVANCED_FACE id of a brep face, emitting it once.
    fn face_id(&mut self, fi: i32, fo: BRepOrientation) -> i32 {
        let brep = self.brep;
        let f = &brep.m_faces[fi as usize];
        let srf = self.surface_id(f.surface_index);

        if srf < 0 {
            return -1;
        }

        let mut bounds = Vec::new();

        for wi in 0..f.wires.len() {
            let lp = self.wire_id(&f.wires[wi]);

            if lp < 0 {
                continue;
            }

            bounds.push(self.w.write_raw(&format!(
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

        let sense = if fo == BRepOrientation::Forward {
            "T"
        } else {
            "F"
        };

        self.w.write_raw(&format!(
            "ADVANCED_FACE('',{},#{},.{}.)",
            fmt_ref_list(&bounds),
            srf,
            sense
        ))
    }
}

/// Face-id groups of a brep written into w: one per shell with its closed flag, then the free faces as an open group.
fn emit_brep_shells(w: &mut StepWriter, brep: &BRep) -> Vec<(Vec<i32>, bool)> {
    let mut em = BRepEmitter::new(w, brep);
    let mut groups = Vec::new();
    let mut in_shell = vec![false; brep.m_faces.len()];

    for si in 0..brep.shell_count() {
        let mut ids = Vec::new();

        for fr in &brep.m_shells[si].faces {
            in_shell[fr.index as usize] = true;

            let id = em.face_id(fr.index, fr.orientation);

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
        let id = if in_shell[fi] {
            -1
        } else {
            em.face_id(fi as i32, BRepOrientation::Forward)
        };

        if id >= 0 {
            free_ids.push(id);
        }
    }

    if !free_ids.is_empty() {
        groups.push((free_ids, false));
    }

    groups
}

/// Bounding-box diagonal of the vertices, 1 when there are none.
fn vertex_diagonal(brep: &BRep) -> f64 {
    if brep.m_vertices.is_empty() {
        return 1.0;
    }

    let mut lo = Point::new(1e300, 1e300, 1e300);
    let mut hi = Point::new(-1e300, -1e300, -1e300);

    for v in &brep.m_vertices {
        for k in 0..3 {
            lo[k] = lo[k].min(v.point[k]);
            hi[k] = hi[k].max(v.point[k]);
        }
    }

    lo.distance(&hi, None)
}

/// Write the STEP text to a file; false when it cannot be written.
fn write_step_string(content: &str, filepath: &str) -> bool {
    std::fs::write(filepath, content).is_ok()
}

// ═══════════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════════
/// Every CARTESIAN_POINT of the file in entity-id order.
pub fn read_file_step_points(filepath: &str) -> Vec<Point> {
    let sf = parse_step_file(filepath);
    let mut r = StepReader::new(&sf);
    let mut out = Vec::new();

    for id in sf.ids_of_type("CARTESIAN_POINT") {
        out.push(r.get_point(id));
    }

    out
}

/// Every B_SPLINE_CURVE_WITH_KNOTS of the file that reads as a valid curve.
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

/// Every B_SPLINE_SURFACE_WITH_KNOTS of the file that reads as a valid surface.
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

/// Outer trim of a face as a dim-2 polyline of sampled 3D edge points (x, y), from the first bound with an EDGE_LOOP.
fn trimmed_outer_loop(r: &mut StepReader, bound_refs: &[i32]) -> NurbsCurve {
    for &bid in bound_refs {
        let Some(b) = bound_loop(r, bid) else {
            continue;
        };

        let mut uv_pts = Vec::new();

        for &oe_id in &b.oe_refs {
            let ecr = edge_refs(r, oriented_edge(r, oe_id).0);

            if ecr.len() < 3 {
                continue;
            }

            let vs = r
                .get_vertex_point(ecr[0])
                .unwrap_or(Point::new(0.0, 0.0, 0.0));
            let ve = r
                .get_vertex_point(ecr[1])
                .unwrap_or(Point::new(0.0, 0.0, 0.0));

            for s in r.sample_curve(ecr[2], &vs, &ve, 8) {
                uv_pts.push(Point::new(s[0], s[1], 0.0));
            }
        }

        return polyline_nurbs(&uv_pts, 2);
    }

    NurbsCurve::default()
}

/// Every ADVANCED_FACE on a B-spline surface with its first edge loop sampled as the outer trim.
pub fn read_file_step_nurbssurfaces_trimmed(filepath: &str) -> Vec<NurbsSurfaceTrimmed> {
    let sf = parse_step_file(filepath);
    let mut r = StepReader::new(&sf);
    let mut out = Vec::new();

    for face_id in sf.ids_of_type("ADVANCED_FACE") {
        let Some(face) = find_in(r.get(face_id), "ADVANCED_FACE") else {
            continue;
        };

        let surface_ref = first_ref(&face.params);

        let Some(surf) = r.get(surface_ref) else {
            continue;
        };

        if !surf.has("B_SPLINE_SURFACE_WITH_KNOTS") {
            continue;
        }

        let mut nst = NurbsSurfaceTrimmed::new();
        nst.m_surface = r.get_nurbs_surface(surface_ref);

        let outer_loop = trimmed_outer_loop(&mut r, &list_refs(&face.params));

        if nst.m_surface.is_valid() && outer_loop.is_valid() {
            nst.m_outer_loop = Some(outer_loop);
            out.push(nst);
        }
    }

    out
}

/// One BRep per shell of every MANIFOLD_SOLID_BREP, BREP_WITH_VOIDS and SHELL_BASED_SURFACE_MODEL, in file order.
pub fn read_file_step_breps(filepath: &str) -> Vec<BRep> {
    let sf = parse_step_file(filepath);
    let mut r = StepReader::new(&sf);
    let mut ids = Vec::new();

    for id in sf.entities.keys() {
        ids.push(*id);
    }

    ids.sort();

    let mut shell_refs = Vec::new();

    for id in ids {
        let e = r.get(id);
        let mut root = find_in(e, "MANIFOLD_SOLID_BREP");

        if root.is_none() {
            root = find_in(e, "BREP_WITH_VOIDS");
        }

        if root.is_none() {
            root = find_in(e, "SHELL_BASED_SURFACE_MODEL");
        }

        let Some(root) = root else {
            continue;
        };

        for sh in all_refs(&root.params) {
            shell_refs.push(sh);
        }

        for sh in list_refs(&root.params) {
            shell_refs.push(sh);
        }
    }

    let mut out = Vec::new();

    for shell_ref in shell_refs {
        let inner = match find_in(r.get(shell_ref), "ORIENTED_CLOSED_SHELL") {
            Some(os) => first_ref(&os.params),
            None => -1,
        };

        let builder = BRepBuilder::new(&mut r);
        let b = builder.build_from_shell(if inner >= 0 { inner } else { shell_ref });

        if !b.m_faces.is_empty() {
            out.push(b);
        }
    }

    out
}

/// One file holding the curves as bare B_SPLINE_CURVE_WITH_KNOTS entities; false when the file cannot be written.
pub fn write_file_step_nurbscurves(curves: &[NurbsCurve], filepath: &str) -> bool {
    let mut w = StepWriter::new();

    for nc in curves {
        w.write_nurbs_curve(nc);
    }

    write_step_string(&w.emit(), filepath)
}

/// One file holding the surfaces as bare B_SPLINE_SURFACE_WITH_KNOTS entities; false when the file cannot be written.
pub fn write_file_step_nurbssurfaces(surfaces: &[NurbsSurface], filepath: &str) -> bool {
    let mut w = StepWriter::new();

    for srf in surfaces {
        w.write_nurbs_surface(srf);
    }

    write_step_string(&w.emit(), filepath)
}

/// One file holding the trimmed surfaces as ADVANCED_FACEs of an open shell; false when the file cannot be written.
pub fn write_file_step_nurbssurfaces_trimmed(
    trimmed: &[NurbsSurfaceTrimmed],
    filepath: &str,
) -> bool {
    let mut w = StepWriter::new();
    let mut face_ids = Vec::new();

    for t in trimmed {
        let fid = w.write_trimmed_face(t);

        if fid >= 0 {
            face_ids.push(fid);
        }
    }

    let mut bodies = Vec::new();
    let body = w.write_body(&face_ids, false);

    if body >= 0 {
        bodies.push(body);
    }

    w.finish_product(&bodies, false, "trimmed", 1e-6, &[]);
    write_step_string(&w.emit(), filepath)
}

/// One AP214 file holding the brep, one body per shell; false when the file cannot be written.
pub fn write_file_step_brep(brep: &BRep, filepath: &str) -> bool {
    let mut w = StepWriter::new();
    let mut bodies = Vec::new();
    let mut any_closed = false;

    for (ids, closed) in emit_brep_shells(&mut w, brep) {
        let body = w.write_body(&ids, closed);

        if body >= 0 {
            bodies.push(body);
        }

        any_closed = any_closed || closed;
    }

    w.finish_product(
        &bodies,
        any_closed,
        if brep.name.is_empty() {
            "brep"
        } else {
            &brep.name
        },
        vertex_diagonal(brep) * 1e-4,
        &[],
    );
    write_step_string(&w.emit(), filepath)
}

/// One AP214 file holding several breps side by side, each face colored from its brep's surfacecolor; false when the file cannot be written.
pub fn write_file_step_breps(breps: &[&BRep], name: &str, filepath: &str) -> bool {
    let mut w = StepWriter::new();
    let mut bodies = Vec::new();
    let mut styled = Vec::new();
    let mut any_closed = false;
    let mut diag = 1.0f64;

    for b in breps {
        let groups = emit_brep_shells(&mut w, b);
        diag = diag.max(vertex_diagonal(b));

        let psa = w.color_style(
            b.surfacecolor.r as f64,
            b.surfacecolor.g as f64,
            b.surfacecolor.b as f64,
        );

        for (ids, closed) in groups {
            let body = w.write_body(&ids, closed);

            if body >= 0 {
                bodies.push(body);
            }

            any_closed = any_closed || closed;

            for fid in ids {
                styled.push(w.write_raw(&format!("STYLED_ITEM('',(#{}),#{})", psa, fid)));
            }
        }
    }

    w.finish_product(&bodies, any_closed, name, diag * 1e-4, &styled);
    write_step_string(&w.emit(), filepath)
}
