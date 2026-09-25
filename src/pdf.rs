use crate::tree::TreeNode;
use crate::Color;
use crate::Line;
use crate::Mesh;
use crate::NurbsCurve;
use crate::Point;
use crate::Polyline;
use crate::Session;
use mupdf::device::NativeDevice;
use mupdf::path::PathWalker;
use mupdf::pdf::PdfDocument;
use mupdf::pdf::PdfObject;
use mupdf::ColorParams;
use mupdf::Colorspace;
use mupdf::Device;
use mupdf::Document;
use mupdf::Matrix;
use mupdf::Path;
use mupdf::Rect;
use mupdf::StrokeState;
use mupdf::Text;
use rayon::iter::IntoParallelRefIterator;
use rayon::iter::ParallelIterator;
use std::cell::RefCell;
use std::collections::btree_map;
use std::collections::hash_map;
use std::collections::BTreeMap;
use std::collections::HashMap;
use std::rc::Rc;

// ═══════════════════════════════════════════════════════════════════════════
// Rust-only PDF import
// ═══════════════════════════════════════════════════════════════════════════
const BEZ_CHORD: f64 = 0.25; // Control-polygon length in points per flattening sample, squared-scaled.
const HAIRLINE: f64 = 0.1; // Width in mm of a PDF width 0.
const WHITE: f32 = 0.99; // A colour this close to white is a knockout mask, not ink.
const KIND_REGION: u8 = 0; // Bucket kind of a path fill, painted first.
const KIND_TEXT: u8 = 1; // Bucket kind of a glyph fill, painted on top.

/// One pen-down run: 2 points is a Line, more is a Polyline.
struct Stroke {
    layer: usize,       // Layer index.
    w: f64,             // Width in mm.
    c: Color,           // Stroke colour.
    pts: Vec<[f64; 2]>, // Points in device space.
}

/// One cubic, kept analytic as a NurbsCurve.
struct Curve {
    layer: usize,      // Layer index.
    w: f64,            // Width in mm.
    c: Color,          // Stroke colour.
    cv: [[f64; 2]; 4], // Bezier control points.
}

/// One filled region: border and holes in any order.
struct Fill {
    layer: usize,              // Layer index.
    c: Color,                  // Fill colour.
    loops: Vec<Vec<[f64; 2]>>, // Closed contours.
}

/// One glyph, triangulated once in glyph space and reused per occurrence.
struct GlyphMesh {
    islands: Vec<(Vec<[f64; 2]>, Vec<usize>)>, // Vertices and triangle indices per island.
}

/// One placed occurrence of a cached glyph.
struct GlyphRef {
    layer: usize,     // Layer index.
    c: Color,         // Fill colour.
    m: [f64; 6],      // Text matrix with pen position and ctm.
    g: Rc<GlyphMesh>, // Cached glyph.
}

/// Merged triangles of one (layer, kind, colour).
struct Bucket {
    c: Color,             // Fill colour.
    verts: Vec<[f64; 2]>, // Vertices in device space.
    tris: Vec<usize>,     // Triangle indices.
}

/// Everything collected from one page.
#[derive(Default)]
struct State {
    flip: f64,               // y' = flip - y, PDF device space is y-down.
    layers: Vec<String>,     // Layer names, index 0 unlayered.
    layer_stack: Vec<usize>, // Open layer indices.
    strokes: Vec<Stroke>,    // Straight pen-down runs.
    curves: Vec<Curve>,      // Analytic cubics.
    fills: Vec<Fill>,        // Filled regions.
    glyph_cache: HashMap<(String, i32), Option<Rc<GlyphMesh>>>, // Glyph meshes per (font, glyph).
    glyph_refs: Vec<GlyphRef>, // Placed glyphs.
}

impl State {
    /// Current layer index.
    fn layer(&self) -> usize {
        *self.layer_stack.last().unwrap_or(&0)
    }

    /// Store one straight run.
    fn add_stroke(&mut self, layer: usize, w: f64, c: &Color, pts: Vec<[f64; 2]>) {
        self.strokes.push(Stroke {
            layer,
            w,
            c: c.clone(),
            pts,
        });
    }

    /// Store the dash on-runs of a stroked path.
    fn add_dashes(
        &mut self,
        segs: &[Seg],
        layer: usize,
        w: f64,
        c: &Color,
        pat: &[f64],
        phase: f64,
    ) {
        for chain in flatten_chains(segs) {
            for run in dash_runs(&chain, pat, phase) {
                self.add_stroke(layer, w, c, run);
            }
        }
    }

    /// Store a solid stroked path as straight runs and analytic cubics.
    fn add_chains(&mut self, segs: &[Seg], layer: usize, w: f64, c: &Color) {
        let mut chain: Vec<[f64; 2]> = Vec::new();
        let mut start: Option<[f64; 2]> = None;

        for s in segs {
            match *s {
                Seg::Move(p) => {
                    if let Some(pts) = take_chain(&mut chain) {
                        self.add_stroke(layer, w, c, pts);
                    }

                    chain.push(p);
                    start = Some(p);
                }
                Seg::Line(p) => chain.push(p),
                Seg::Curve(c1, c2, e) => {
                    let a = *chain.last().unwrap_or(&c1);

                    if let Some(pts) = take_chain(&mut chain) {
                        self.add_stroke(layer, w, c, pts);
                    }

                    self.curves.push(Curve {
                        layer,
                        w,
                        c: c.clone(),
                        cv: [a, c1, c2, e],
                    });
                    chain.push(e);
                }
                Seg::Close => {
                    close_chain(&mut chain, start);

                    if let Some(pts) = take_chain(&mut chain) {
                        self.add_stroke(layer, w, c, pts);
                    }

                    if let Some(s0) = start {
                        chain.push(s0);
                    }
                }
            }
        }

        if chain.len() >= 2 {
            self.add_stroke(layer, w, c, chain);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Path walking
// ═══════════════════════════════════════════════════════════════════════════

/// One path op in device space, ctm applied and y flipped.
enum Seg {
    Move([f64; 2]),                      // Start a subpath.
    Line([f64; 2]),                      // Straight segment.
    Curve([f64; 2], [f64; 2], [f64; 2]), // Cubic with two control points and an end.
    Close,                               // Close the subpath.
}

/// Collects a mupdf `Path` into ops in device space.
struct Walk {
    ctm: Matrix,   // User to device transform.
    flip: f64,     // y' = flip - y.
    out: Vec<Seg>, // Collected ops.
}

impl Walk {
    /// Map a user-space point to device space.
    fn pt(&self, x: f32, y: f32) -> [f64; 2] {
        let (x, y) = (x as f64, y as f64);
        let (a, b, c, d, e, f) = (
            self.ctm.a as f64,
            self.ctm.b as f64,
            self.ctm.c as f64,
            self.ctm.d as f64,
            self.ctm.e as f64,
            self.ctm.f as f64,
        );
        [a * x + c * y + e, self.flip - (b * x + d * y + f)]
    }
}

impl PathWalker for &mut Walk {
    fn move_to(&mut self, x: f32, y: f32) {
        let p = self.pt(x, y);
        self.out.push(Seg::Move(p));
    }

    fn line_to(&mut self, x: f32, y: f32) {
        let p = self.pt(x, y);
        self.out.push(Seg::Line(p));
    }

    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x3: f32, y3: f32) {
        let (c1, c2, e) = (self.pt(x1, y1), self.pt(x2, y2), self.pt(x3, y3));
        self.out.push(Seg::Curve(c1, c2, e));
    }

    fn close(&mut self) {
        self.out.push(Seg::Close);
    }
}

/// Walk a path into device-space ops.
fn walk(path: &Path, ctm: Matrix, flip: f64) -> Vec<Seg> {
    let mut w = Walk {
        ctm,
        flip,
        out: Vec::new(),
    };
    let _ = path.walk(&mut w);
    w.out
}

// ═══════════════════════════════════════════════════════════════════════════
// Flattening
// ═══════════════════════════════════════════════════════════════════════════

/// Distance between two points.
fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

/// Samples for one cubic from its control-polygon length, 4 to 16.
fn bez_steps(a: [f64; 2], c1: [f64; 2], c2: [f64; 2], b: [f64; 2]) -> usize {
    let d = dist(a, c1) + dist(c1, c2) + dist(c2, b);
    ((d / BEZ_CHORD).sqrt() as usize).clamp(4, 16)
}

/// Append the samples of one cubic, excluding its start point.
fn bezier(a: [f64; 2], c1: [f64; 2], c2: [f64; 2], b: [f64; 2], out: &mut Vec<[f64; 2]>) {
    let n = bez_steps(a, c1, c2, b);

    for i in 1..=n {
        let t = i as f64 / n as f64;
        let u = 1.0 - t;
        let (w0, w1, w2, w3) = (u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t);
        out.push([
            w0 * a[0] + w1 * c1[0] + w2 * c2[0] + w3 * b[0],
            w0 * a[1] + w1 * c1[1] + w2 * c2[1] + w3 * b[1],
        ]);
    }
}

/// Close the current contour into `loops` when it has more than two points.
fn close_loop(cur: &mut Vec<[f64; 2]>, loops: &mut Vec<Vec<[f64; 2]>>) {
    if cur.len() <= 2 {
        cur.clear();
        return;
    }

    if cur[0] != cur[cur.len() - 1] {
        cur.push(cur[0]);
    }

    loops.push(std::mem::take(cur));
}

/// Drop consecutive points closer than 1e-9.
fn dedup(points: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(points.len());

    for p in points {
        if !out.is_empty() && dist(*p, out[out.len() - 1]) < 1e-9 {
            continue;
        }

        out.push(*p);
    }

    out
}

/// Closed contours with beziers flattened.
fn contours(segs: &[Seg]) -> Vec<Vec<[f64; 2]>> {
    let mut loops: Vec<Vec<[f64; 2]>> = Vec::new();
    let mut cur: Vec<[f64; 2]> = Vec::new();

    for s in segs {
        match *s {
            Seg::Move(p) => {
                close_loop(&mut cur, &mut loops);
                cur.push(p);
            }
            Seg::Line(p) => cur.push(p),
            Seg::Curve(c1, c2, e) => {
                let a = *cur.last().unwrap_or(&c1);
                bezier(a, c1, c2, e, &mut cur);
            }
            Seg::Close => close_loop(&mut cur, &mut loops),
        }
    }

    close_loop(&mut cur, &mut loops);
    let mut out: Vec<Vec<[f64; 2]>> = Vec::with_capacity(loops.len());

    for lp in &loops {
        let clean = dedup(lp);

        if clean.len() > 3 {
            out.push(clean);
        }
    }

    out
}

/// Take the chain when it holds a segment, leaving it empty either way.
fn take_chain(chain: &mut Vec<[f64; 2]>) -> Option<Vec<[f64; 2]>> {
    if chain.len() < 2 {
        chain.clear();
        return None;
    }

    Some(std::mem::take(chain))
}

/// Close the chain back to its subpath start.
fn close_chain(chain: &mut Vec<[f64; 2]>, start: Option<[f64; 2]>) {
    if let Some(s0) = start {
        if !chain.is_empty() && chain[chain.len() - 1] != s0 {
            chain.push(s0);
        }
    }
}

/// Open pen-down chains with beziers flattened.
fn flatten_chains(segs: &[Seg]) -> Vec<Vec<[f64; 2]>> {
    let mut out: Vec<Vec<[f64; 2]>> = Vec::new();
    let mut cur: Vec<[f64; 2]> = Vec::new();
    let mut start: Option<[f64; 2]> = None;

    for s in segs {
        match *s {
            Seg::Move(p) => {
                if let Some(chain) = take_chain(&mut cur) {
                    out.push(chain);
                }

                cur.push(p);
                start = Some(p);
            }
            Seg::Line(p) => cur.push(p),
            Seg::Curve(c1, c2, e) => {
                let a = *cur.last().unwrap_or(&c1);

                if cur.is_empty() {
                    cur.push(a);
                }

                bezier(a, c1, c2, e, &mut cur);
            }
            Seg::Close => {
                close_chain(&mut cur, start);

                if let Some(chain) = take_chain(&mut cur) {
                    out.push(chain);
                }

                if let Some(s0) = start {
                    cur.push(s0);
                }
            }
        }
    }

    if cur.len() >= 2 {
        out.push(cur);
    }

    out
}

/// Split one flattened chain into its dash on-runs.
fn dash_runs(pts: &[[f64; 2]], pat: &[f64], phase: f64) -> Vec<Vec<[f64; 2]>> {
    let cycle: f64 = pat.iter().sum();
    let mut idx = 0;
    let mut pos = phase.rem_euclid(cycle);

    while pos >= pat[idx] {
        pos -= pat[idx];
        idx = (idx + 1) % pat.len();
    }

    let mut rem = pat[idx] - pos;
    let mut on = idx % 2 == 0;
    let mut runs: Vec<Vec<[f64; 2]>> = Vec::new();
    let mut cur: Vec<[f64; 2]> = Vec::new();

    if on {
        cur.push(pts[0]);
    }

    for w2 in pts.windows(2) {
        let (a, b) = (w2[0], w2[1]);
        let len = dist(a, b);

        if len < 1e-12 {
            continue;
        }

        let mut done = 0.0;

        while rem < len - done {
            done += rem;
            let t = done / len;
            let p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];

            if on {
                cur.push(p);

                if let Some(run) = take_chain(&mut cur) {
                    runs.push(run);
                }
            } else {
                cur.clear();
                cur.push(p);
            }

            on = !on;
            idx = (idx + 1) % pat.len();
            rem = pat[idx];
        }

        rem -= len - done;

        if on {
            cur.push(b);
        }
    }

    if on && cur.len() >= 2 {
        runs.push(cur);
    }

    runs
}

// ═══════════════════════════════════════════════════════════════════════════
// Islands
// ═══════════════════════════════════════════════════════════════════════════

/// Signed shoelace area of a closed contour.
fn area_signed(loop_: &[[f64; 2]]) -> f64 {
    let mut a = 0.0;

    for i in 0..loop_.len() {
        let (p, q) = (loop_[i], loop_[(i + 1) % loop_.len()]);
        a += p[0] * q[1] - q[0] * p[1];
    }

    a * 0.5
}

/// Is `p` inside the closed polygon `poly` by ray casting?
fn inside(p: [f64; 2], poly: &[[f64; 2]]) -> bool {
    let mut hit = false;
    let n = poly.len();

    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);

        if (a[1] > p[1]) != (b[1] > p[1]) {
            let t = (p[1] - a[1]) / (b[1] - a[1]);

            if p[0] < a[0] + t * (b[0] - a[0]) {
                hit = !hit;
            }
        }
    }

    hit
}

/// Winding number of `p` in the closed polygon `poly`.
fn winding(p: [f64; 2], poly: &[[f64; 2]]) -> i32 {
    let mut wn = 0;
    let n = poly.len();

    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        let cross = (b[0] - a[0]) * (p[1] - a[1]) - (p[0] - a[0]) * (b[1] - a[1]);

        if a[1] <= p[1] {
            if b[1] > p[1] && cross > 0.0 {
                wn += 1;
            }
        } else if b[1] <= p[1] && cross < 0.0 {
            wn -= 1;
        }
    }

    wn
}

/// Number of other contours that contain each contour.
fn containment_depth(loops: &[Vec<[f64; 2]>]) -> Vec<usize> {
    let n = loops.len();
    let mut depth = vec![0usize; n];

    for i in 0..n {
        let probe = loops[i][0];

        for (j, other) in loops.iter().enumerate() {
            if i != j && inside(probe, other) {
                depth[i] += 1;
            }
        }
    }

    depth
}

/// Classify contour `i` under the nonzero rule: 0 island, 1 hole, 2 dropped.
fn nonzero_class(loops: &[Vec<[f64; 2]>], i: usize) -> u8 {
    let probe = loops[i][0];
    let mut wn_out = 0;

    for (j, other) in loops.iter().enumerate() {
        if j != i {
            wn_out += winding(probe, other);
        }
    }

    let wn_in = wn_out + if area_signed(&loops[i]) >= 0.0 { 1 } else { -1 };

    match (wn_in != 0, wn_out != 0) {
        (true, false) => 0,
        (false, true) => 1,
        _ => 2,
    }
}

/// Split contours into islands, each with the holes it contains, under the even-odd or nonzero rule.
fn islands(loops: Vec<Vec<[f64; 2]>>, even_odd: bool) -> Vec<Vec<Vec<[f64; 2]>>> {
    let n = loops.len();

    if n == 0 {
        return Vec::new();
    }

    if n == 1 {
        return vec![loops];
    }

    let depth = containment_depth(&loops);
    let mut class: Vec<u8> = Vec::with_capacity(n);

    for (i, d) in depth.iter().enumerate() {
        if even_odd {
            class.push((d % 2) as u8);
        } else {
            class.push(nonzero_class(&loops, i));
        }
    }

    let mut out: Vec<Vec<Vec<[f64; 2]>>> = Vec::new();
    let mut index = vec![0usize; n];

    for i in 0..n {
        if class[i] == 0 {
            index[i] = out.len();
            out.push(vec![loops[i].clone()]);
        }
    }

    for i in 0..n {
        if class[i] != 1 {
            continue;
        }

        let probe = loops[i][0];
        let mut best: Option<(usize, usize)> = None;

        for j in 0..n {
            if i == j || class[j] != 0 || !inside(probe, &loops[j]) {
                continue;
            }

            let deeper = match best {
                Some((d, _)) => depth[j] > d,
                None => true,
            };

            if deeper {
                best = Some((depth[j], index[j]));
            }
        }

        if let Some((_, k)) = best {
            out[k].push(loops[i].clone());
        }
    }

    out
}

// ═══════════════════════════════════════════════════════════════════════════
// Triangulation
// ═══════════════════════════════════════════════════════════════════════════

/// Triangulate one island (border + holes) with earcut into vertices and triangle indices.
fn earcut_raw(loops: &[Vec<[f64; 2]>]) -> (Vec<[f64; 2]>, Vec<usize>) {
    let mut flat: Vec<f64> = Vec::new();
    let mut holes: Vec<usize> = Vec::new();

    for (i, lp) in loops.iter().enumerate() {
        let pts = if lp.len() > 1 && lp[0] == lp[lp.len() - 1] {
            &lp[..lp.len() - 1]
        } else {
            &lp[..]
        };

        if pts.len() < 3 {
            continue;
        }

        if i > 0 {
            holes.push(flat.len() / 2);
        }

        for p in pts {
            flat.push(p[0]);
            flat.push(p[1]);
        }
    }

    if flat.len() < 6 {
        return (Vec::new(), Vec::new());
    }

    let Ok(tris) = earcutr::earcut(&flat, &holes, 2) else {
        return (Vec::new(), Vec::new());
    };
    let mut verts: Vec<[f64; 2]> = Vec::with_capacity(flat.len() / 2);

    for c in flat.chunks_exact(2) {
        verts.push([c[0], c[1]]);
    }

    (verts, tris)
}

/// Triangulate one filled region.
fn triangulate_fill(fill: &Fill) -> (Vec<[f64; 2]>, Vec<usize>) {
    earcut_raw(&fill.loops)
}

/// Outline and triangulate one glyph in glyph space, None when mupdf has no outline for it.
fn build_glyph(font: &mupdf::Font, gid: i32) -> Option<GlyphMesh> {
    let outline = font
        .outline_glyph_with_ctm(gid, &Matrix::IDENTITY)
        .ok()
        .flatten()?;
    let loops = contours(&walk(&outline, Matrix::IDENTITY, 0.0));
    let mut out: Vec<(Vec<[f64; 2]>, Vec<usize>)> = Vec::new();

    for island in islands(loops, false) {
        let (v, t) = earcut_raw(&island);

        if !t.is_empty() {
            out.push((v, t));
        }
    }

    Some(GlyphMesh { islands: out })
}

/// Glyph vertices placed by the text matrix `m` into device space.
fn place_glyph(verts: &[[f64; 2]], m: [f64; 6], flip: f64) -> Vec<[f64; 2]> {
    let [a, b, c, d, e, f] = m;
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(verts.len());

    for q in verts {
        let (x, y) = (q[0], -q[1]);
        out.push([a * x + c * y + e, flip - (b * x + d * y + f)]);
    }

    out
}

// ═══════════════════════════════════════════════════════════════════════════
// Device
// ═══════════════════════════════════════════════════════════════════════════

/// Convert a device colour of any colorspace to RGB.
fn to_color(cs: &Colorspace, color: &[f32], alpha: f32, cp: ColorParams) -> Color {
    let rgb = match cs.convert_color(color, &Colorspace::device_rgb(), None, cp) {
        Ok(rgb) => rgb,
        Err(_) => vec![0.0, 0.0, 0.0],
    };
    Color::new(
        rgb.first().copied().unwrap_or(0.0),
        rgb.get(1).copied().unwrap_or(0.0),
        rgb.get(2).copied().unwrap_or(0.0),
        alpha,
    )
}

/// Is the colour a knockout white?
fn is_white(c: &Color) -> bool {
    c.r >= WHITE && c.g >= WHITE && c.b >= WHITE
}

/// A mupdf device that records a page into `State`.
struct Collector(Rc<RefCell<State>>);

impl NativeDevice for Collector {
    fn fill_path(
        &mut self,
        path: &Path,
        even_odd: bool,
        ctm: Matrix,
        cs: &Colorspace,
        color: &[f32],
        alpha: f32,
        cp: ColorParams,
    ) {
        let c = to_color(cs, color, alpha, cp);

        if is_white(&c) {
            return;
        }

        let mut st = self.0.borrow_mut();
        let loops = contours(&walk(path, ctm, st.flip));
        let layer = st.layer();

        for island in islands(loops, even_odd) {
            st.fills.push(Fill {
                layer,
                c: c.clone(),
                loops: island,
            });
        }
    }

    fn stroke_path(
        &mut self,
        path: &Path,
        stroke: &StrokeState,
        ctm: Matrix,
        cs: &Colorspace,
        color: &[f32],
        alpha: f32,
        cp: ColorParams,
    ) {
        let c = to_color(cs, color, alpha, cp);

        if is_white(&c) {
            return;
        }

        let mut st = self.0.borrow_mut();
        let exp = ctm.expansion() as f64;
        let w = stroke.line_width() as f64 * exp;
        let w = if w > 0.0 { w } else { HAIRLINE };
        let layer = st.layer();
        let segs = walk(path, ctm, st.flip);
        let mut pat: Vec<f64> = Vec::new();

        for d in stroke.dashes() {
            pat.push(d as f64 * exp);
        }

        if !pat.is_empty() && pat.iter().sum::<f64>() > 1e-9 {
            if pat.len() % 2 == 1 {
                pat.extend_from_within(..);
            }

            let phase = stroke.dash_phase() as f64 * exp;
            st.add_dashes(&segs, layer, w, &c, &pat, phase);
            return;
        }

        st.add_chains(&segs, layer, w, &c);
    }

    fn fill_text(
        &mut self,
        text: &Text,
        ctm: Matrix,
        cs: &Colorspace,
        color: &[f32],
        alpha: f32,
        cp: ColorParams,
    ) {
        let c = to_color(cs, color, alpha, cp);

        if is_white(&c) {
            return;
        }

        let mut st = self.0.borrow_mut();
        let layer = st.layer();

        for span in text.spans() {
            let font = span.font();
            let name = font.name().to_string();

            for item in span.items() {
                if item.gid() < 0 {
                    continue;
                }

                let mut m = span.trm();
                m.e = item.x();
                m.f = item.y();
                m.concat(ctm.clone());
                let glyph = match st.glyph_cache.entry((name.clone(), item.gid())) {
                    hash_map::Entry::Occupied(entry) => entry.get().clone(),
                    hash_map::Entry::Vacant(entry) => entry
                        .insert(build_glyph(&font, item.gid()).map(Rc::new))
                        .clone(),
                };

                if let Some(g) = glyph {
                    st.glyph_refs.push(GlyphRef {
                        layer,
                        c: c.clone(),
                        m: [
                            m.a as f64, m.b as f64, m.c as f64, m.d as f64, m.e as f64, m.f as f64,
                        ],
                        g,
                    });
                }
            }
        }
    }

    fn stroke_text(
        &mut self,
        text: &Text,
        _s: &StrokeState,
        ctm: Matrix,
        cs: &Colorspace,
        color: &[f32],
        alpha: f32,
        cp: ColorParams,
    ) {
        self.fill_text(text, ctm, cs, color, alpha, cp);
    }

    fn begin_layer(&mut self, name: &str) {
        let mut st = self.0.borrow_mut();
        let mut id = st.layers.len();

        for (i, layer) in st.layers.iter().enumerate() {
            if layer == name {
                id = i;
                break;
            }
        }

        if id == st.layers.len() {
            st.layers.push(name.to_string());
        }

        st.layer_stack.push(id);
    }

    fn end_layer(&mut self) {
        self.0.borrow_mut().layer_stack.pop();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Fonts
// ═══════════════════════════════════════════════════════════════════════════

/// The face name of a font descriptor.
fn font_name(obj: &PdfObject) -> Option<String> {
    let name = obj.get_dict("FontName").ok().flatten()?;
    let bytes = name.as_name().ok()?;
    Some(String::from_utf8_lossy(&bytes).to_string())
}

/// A file-system-safe copy of a font name.
fn safe_name(name: &str) -> String {
    let mut safe = String::with_capacity(name.len());

    for c in name.chars() {
        if c.is_alphanumeric() || c == '-' || c == '+' {
            safe.push(c);
        } else {
            safe.push('_');
        }
    }

    safe
}

/// Write every embedded font program into `<assets>/fonts`, next to `<assets>/pb/<stem>.pb`.
fn write_fonts(src: &str, stem: &str) {
    let out_dir = std::path::Path::new(stem)
        .parent()
        .unwrap_or(std::path::Path::new("."));
    let font_dir = match out_dir.parent() {
        Some(parent) => parent.join("fonts"),
        None => out_dir.join("fonts"),
    };

    if std::fs::create_dir_all(&font_dir).is_err() {
        return;
    }

    let Ok(doc) = PdfDocument::open(src) else {
        return;
    };
    let Ok(count) = doc.count_objects() else {
        return;
    };

    for num in 1..count as i32 {
        let Ok(obj) = doc.new_indirect(num, 0) else {
            continue;
        };
        let Ok(Some(obj)) = obj.resolve() else {
            continue;
        };

        for (key, ext) in [
            ("FontFile", "pfb"),
            ("FontFile2", "ttf"),
            ("FontFile3", "cff"),
        ] {
            let Ok(Some(ff)) = obj.get_dict(key) else {
                continue;
            };
            let Ok(bytes) = ff.read_stream() else {
                continue;
            };

            if bytes.is_empty() {
                continue;
            }

            let name = match font_name(&obj) {
                Some(name) => name,
                None => format!("font_{num}"),
            };
            let path = font_dir.join(format!("{}.{ext}", safe_name(&name)));

            if !path.exists() {
                std::fs::write(&path, &bytes).ok();
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Session output
// ═══════════════════════════════════════════════════════════════════════════

/// Lift 2D points to z = 0.
fn points(pts: &[[f64; 2]]) -> Vec<Point> {
    let mut out: Vec<Point> = Vec::with_capacity(pts.len());

    for q in pts {
        out.push(Point::new(q[0], q[1], 0.0));
    }

    out
}

/// Colour quantized to bytes, the bucket key.
fn ckey(c: &Color) -> [u8; 4] {
    [
        (c.r.clamp(0.0, 1.0) * 255.0).round() as u8,
        (c.g.clamp(0.0, 1.0) * 255.0).round() as u8,
        (c.b.clamp(0.0, 1.0) * 255.0).round() as u8,
        (c.a.clamp(0.0, 1.0) * 255.0).round() as u8,
    ]
}

/// The session group of a layer, created on first use.
fn get_group(
    session: &mut Session,
    groups: &mut BTreeMap<usize, Rc<RefCell<TreeNode>>>,
    layers: &[String],
    layer: usize,
) -> Rc<RefCell<TreeNode>> {
    if let Some(group) = groups.get(&layer) {
        return group.clone();
    }

    let name = match layers.get(layer) {
        Some(name) => name.as_str(),
        None => "0 unlayered",
    };
    let group = session.add_group(name);
    groups.insert(layer, group.clone());
    group
}

/// Add each stroke as a Line when it has two points, else as a Polyline.
fn add_strokes(
    session: &mut Session,
    groups: &mut BTreeMap<usize, Rc<RefCell<TreeNode>>>,
    st: &State,
) {
    for sk in &st.strokes {
        let group = get_group(session, groups, &st.layers, sk.layer);
        let pts = points(&sk.pts);

        if pts.len() == 2 {
            let mut line = Line::from_points(&pts[0], &pts[1]);
            line.linecolor = sk.c.clone();
            line.width = sk.w;
            session.add_line(line, Some(&group));
            continue;
        }

        let mut polyline = Polyline::new(pts);
        polyline.linecolor = sk.c.clone();
        polyline.width = sk.w;
        session.add_polyline(polyline, Some(&group));
    }
}

/// Add each cubic as a degree-3 NurbsCurve.
fn add_curves(
    session: &mut Session,
    groups: &mut BTreeMap<usize, Rc<RefCell<TreeNode>>>,
    st: &State,
) {
    for cv in &st.curves {
        let group = get_group(session, groups, &st.layers, cv.layer);
        let mut curve = NurbsCurve::create(false, 3, &points(&cv.cv));
        curve.linecolors = vec![cv.c.clone()];
        curve.width = cv.w;
        session.add_nurbscurve(curve, Some(&group));
    }
}

/// Append one triangulated part to the bucket of its (layer, kind, colour).
fn add_part(
    buckets: &mut BTreeMap<(usize, u8, [u8; 4]), Bucket>,
    layer: usize,
    kind: u8,
    c: &Color,
    verts: &[[f64; 2]],
    tris: &[usize],
) {
    let bucket = match buckets.entry((layer, kind, ckey(c))) {
        btree_map::Entry::Occupied(entry) => entry.into_mut(),
        btree_map::Entry::Vacant(entry) => entry.insert(Bucket {
            c: c.clone(),
            verts: Vec::new(),
            tris: Vec::new(),
        }),
    };
    let base = bucket.verts.len();
    bucket.verts.extend_from_slice(verts);

    for t in tris {
        bucket.tris.push(t + base);
    }
}

impl Bucket {
    /// Flat-colour mesh named "text" or "fill", with one transparent zero-width edge colour.
    fn to_mesh(&self, kind: u8) -> Mesh {
        let mut mesh = Mesh::new();

        for p in &self.verts {
            mesh.add_vertex(Point::new(p[0], p[1], 0.0), None);
        }

        for t in self.tris.chunks_exact(3) {
            mesh.add_face(vec![t[0], t[1], t[2]], None);
        }

        mesh.name = if kind == KIND_TEXT {
            "text".to_string()
        } else {
            "fill".to_string()
        };
        mesh.set_objectcolor(self.c.clone());
        mesh.clear_pointcolors();
        mesh.clear_facecolors();
        mesh.set_linecolors(vec![Color::new(0.0, 0.0, 0.0, 0.0)], vec![0.0]);
        mesh
    }
}

/// Merge path fills, then glyphs, into one mesh per (layer, kind, colour).
fn add_meshes(
    session: &mut Session,
    groups: &mut BTreeMap<usize, Rc<RefCell<TreeNode>>>,
    st: &State,
    tri_fills: &[(Vec<[f64; 2]>, Vec<usize>)],
) {
    let mut buckets: BTreeMap<(usize, u8, [u8; 4]), Bucket> = BTreeMap::new();

    for (f, (v, t)) in st.fills.iter().zip(tri_fills) {
        if !t.is_empty() {
            add_part(&mut buckets, f.layer, KIND_REGION, &f.c, v, t);
        }
    }

    for gr in &st.glyph_refs {
        for (v, t) in &gr.g.islands {
            let placed = place_glyph(v, gr.m, st.flip);
            add_part(&mut buckets, gr.layer, KIND_TEXT, &gr.c, &placed, t);
        }
    }

    for ((layer, kind, _), bucket) in &buckets {
        let mesh = bucket.to_mesh(*kind);

        if mesh.number_of_faces() == 0 {
            continue;
        }

        let group = get_group(session, groups, &st.layers, *layer);
        session.add_mesh(mesh, Some(&group));
    }
}

/// Add the paper edge as a closed Polyline in a "page" group.
fn add_page_border(session: &mut Session, bounds: &Rect, flip: f64) {
    let group = session.add_group("page");
    let (px0, py0, px1, py1) = (
        bounds.x0 as f64,
        flip - bounds.y1 as f64,
        bounds.x1 as f64,
        flip - bounds.y0 as f64,
    );
    let mut border = Polyline::new(vec![
        Point::new(px0, py0, 0.0),
        Point::new(px1, py0, 0.0),
        Point::new(px1, py1, 0.0),
        Point::new(px0, py1, 0.0),
        Point::new(px0, py0, 0.0),
    ]);
    border.linecolor = Color::black();
    border.width = 0.35;
    session.add_polyline(border, Some(&group));
}

/// Import one page of `src` into a Session, write `<stem>.pb` and the embedded fonts.
pub fn import_pdf(src: &str, stem: &str, page_no: i32) {
    let doc = Document::open(src).expect("cannot open pdf");
    let page = doc.load_page(page_no).expect("no such page");
    let bounds = page.bounds().expect("no page box");
    let state = Rc::new(RefCell::new(State {
        flip: (bounds.y0 + bounds.y1) as f64,
        layers: vec!["0 unlayered".to_string()],
        ..Default::default()
    }));
    let device = Device::from_native(Collector(state.clone())).expect("device");
    page.run(&device, &Matrix::IDENTITY).expect("run page");
    drop(device);

    let st = state.take();
    let tri_fills: Vec<(Vec<[f64; 2]>, Vec<usize>)> =
        st.fills.par_iter().map(triangulate_fill).collect();
    let name = std::path::Path::new(stem)
        .file_name()
        .unwrap()
        .to_string_lossy()
        .to_string();
    let mut session = Session::new(&name);
    let mut groups: BTreeMap<usize, Rc<RefCell<TreeNode>>> = BTreeMap::new();
    add_strokes(&mut session, &mut groups, &st);
    add_curves(&mut session, &mut groups, &st);
    add_meshes(&mut session, &mut groups, &st, &tri_fills);
    add_page_border(&mut session, &bounds, st.flip);
    write_fonts(src, stem);
    session.pb_dump(&format!("{stem}.pb"));
}
