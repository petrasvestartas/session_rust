use crate::brep::BRep;
use crate::brep::BRepFace;
use crate::brep::BRepOrientation;
use crate::brep::BRepRef;
use crate::closest::Closest;
use crate::line::Line;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
use crate::polyline::Polyline;
use crate::tolerance::Tolerance;
use crate::vector::Vector;
use std::collections::BTreeMap;

// ═══════════════════════════════════════════════════════════════════════════
// Types
// ═══════════════════════════════════════════════════════════════════════════
const EPSILON: f64 = Tolerance::ZERO_TOLERANCE; // Relative parameter and determinant epsilon.
const FORWARD: BRepOrientation = BRepOrientation::Forward; // Use along the curve direction.
const REVERSED: BRepOrientation = BRepOrientation::Reversed; // Use against the curve direction.
const WORK_LIMIT: usize = 200000; // Upper bound on subdivision steps.

/// Axis-aligned box around the control points of a curve.
struct Bounds {
    lo: [f64; 3], // Minimum corner.
    hi: [f64; 3], // Maximum corner.
}

impl Bounds {
    /// Box around the control points of a curve.
    fn new(curve: &NurbsCurve) -> Result<Self, String> {
        let mut p = curve.get_cv(0).ok_or("Missing curve control")?;
        let mut result = Self {
            lo: [p[0], p[1], p[2]],
            hi: [p[0], p[1], p[2]],
        };

        for i in 1..curve.cv_count() {
            p = curve.get_cv(i).ok_or("Missing curve control")?;

            for d in 0..3 {
                result.lo[d] = result.lo[d].min(p[d]);
                result.hi[d] = result.hi[d].max(p[d]);
            }
        }

        Ok(result)
    }

    /// Length of the box diagonal.
    fn diagonal(&self) -> f64 {
        (self.hi[0] - self.lo[0])
            .hypot(self.hi[1] - self.lo[1])
            .hypot(self.hi[2] - self.lo[2])
    }

    /// True when the boxes overlap within tolerance.
    fn overlaps(&self, other: &Self, tolerance: f64) -> bool {
        for d in 0..3 {
            if self.hi[d] + tolerance < other.lo[d] || other.hi[d] + tolerance < self.lo[d] {
                return false;
            }
        }

        true
    }
}

/// Two curve pieces tested for intersection.
struct Pair {
    a: NurbsCurve, // Piece of the first curve.
    b: NurbsCurve, // Piece of the second curve.
    depth: usize,  // Subdivision depth.
}

/// Curve piece waiting to be flattened.
struct Part {
    curve: NurbsCurve, // Piece of the curve.
    depth: usize,      // Subdivision depth.
}

/// Trim or cutter curve in world and surface parameter space.
struct Source {
    edge: i32,         // BRep edge, -1 for a cutter.
    world: NurbsCurve, // 3D curve.
    uv: NurbsCurve,    // Curve in surface parameter space.
}

/// Parameter run along one source curve.
#[derive(Clone, Copy)]
struct Run {
    source: usize, // Index into the sources.
    a: f64,        // Start parameter.
    b: f64,        // End parameter.
}

/// Knot span of a source curve with its cut parameters.
struct Span {
    source: usize,     // Index into the sources.
    a: f64,            // Start parameter.
    b: f64,            // End parameter.
    cuts: Vec<f64>,    // Cut parameters, span ends included.
    curve: NurbsCurve, // Span of the source uv curve.
}

/// Directed half-edge of the trim graph.
struct Directed {
    b: usize, // Head vertex.
    run: Run, // Parameter run along the source.
}

/// Planar trim graph, twin half-edges at index ^ 1.
struct Graph {
    edges: Vec<Directed>,      // Half-edges.
    outgoing: Vec<Vec<usize>>, // Half-edges leaving each vertex, sorted by angle.
}

/// Closed loop of runs with its sampled polygon.
struct Cycle {
    area: f64,          // Signed area in parameter space.
    loop_: Vec<Run>,    // Runs around the loop.
    points: Vec<Point>, // Sampled polygon.
}

/// New BRep edge cut from a source.
struct Piece {
    source: usize, // Index into the sources.
    lo: f64,       // Start parameter on the world curve.
    hi: f64,       // End parameter on the world curve.
    edge: i32,     // BRep edge.
}

// ═══════════════════════════════════════════════════════════════════════════
// Validation
// ═══════════════════════════════════════════════════════════════════════════
/// Err with the message when the condition fails.
fn require(condition: bool, message: &str) -> Result<(), String> {
    if !condition {
        return Err(message.into());
    }

    Ok(())
}

/// Reject a tolerance that is not finite and positive.
fn check_tolerance(tolerance: f64) -> Result<(), String> {
    require(
        tolerance.is_finite() && tolerance > 0.0,
        "Split tolerance must be finite and positive",
    )
}

/// Reject an invalid curve or one with non-finite controls or non-positive weights.
fn check_curve(curve: &NurbsCurve) -> Result<(), String> {
    require(curve.is_valid(), "Split requires valid curves")?;

    for i in 0..curve.cv_count() {
        let p = curve.get_cv(i).ok_or("Split requires valid controls")?;
        let w = curve.weight(i);
        require(
            p[0].is_finite() && p[1].is_finite() && p[2].is_finite() && w.is_finite() && w > 0.0,
            "Split requires finite controls and positive rational weights",
        )?;
    }

    Ok(())
}

/// Reject an invalid surface or one with non-finite controls or non-positive weights.
fn check_surface(surface: &NurbsSurface) -> Result<(), String> {
    require(surface.is_valid(), "Split requires a valid NURBS surface")?;

    for i in 0..surface.cv_count(0) {
        for j in 0..surface.cv_count(1) {
            let p = surface.get_cv(i, j).ok_or("Invalid surface control")?;
            let w = surface.weight(i, j);
            require(
                p[0].is_finite()
                    && p[1].is_finite()
                    && p[2].is_finite()
                    && w.is_finite()
                    && w > 0.0,
                "Split requires finite surface controls and positive rational weights",
            )?;
        }
    }

    Ok(())
}

/// Surface point, an error when the surface cannot be evaluated.
fn surface_point(surface: &NurbsSurface, u: f64, v: f64) -> Result<Point, String> {
    surface
        .point_at(u, v)
        .ok_or_else(|| "Cannot evaluate the source surface".into())
}

/// Surface domain in one direction, an error when the surface has none.
fn surface_domain(surface: &NurbsSurface, dir: usize) -> Result<(f64, f64), String> {
    surface
        .domain(dir)
        .ok_or_else(|| "Invalid surface domain".into())
}

// ═══════════════════════════════════════════════════════════════════════════
// Curve parameters
// ═══════════════════════════════════════════════════════════════════════════
/// Copy of a curve trimmed to [a, b], clamped to its domain.
fn interval(curve: &NurbsCurve, a: f64, b: f64) -> Result<NurbsCurve, String> {
    let mut result = curve.duplicate();
    let lo = curve.domain_start();
    let hi = curve.domain_end();
    let a = a.clamp(lo, hi);
    let b = b.clamp(lo, hi);
    require(b > a, "Split produced an empty curve interval")?;

    if a > lo || b < hi {
        require(result.trim(a, b), "Kernel refused a split interval")?;
    }

    Ok(result)
}

/// Closest parameter and distance on a degree-1 curve, exact per segment.
fn closest_segments(curve: &NurbsCurve, point: &Point, mut t: f64) -> Result<(f64, f64), String> {
    let spans = curve.get_span_vector();
    let mut best = f64::INFINITY;

    for i in 1..spans.len() {
        let a = curve.point_at(spans[i - 1]);
        let b = curve.point_at(spans[i]);
        let v = &b - &a;
        let length2 = v.dot(&v);

        if length2 <= EPSILON * EPSILON {
            continue;
        }

        let fraction = ((point - &a).dot(&v) / length2).clamp(0.0, 1.0);
        let segment = interval(curve, spans[i - 1], spans[i])?;
        let w0 = segment.weight(0);
        let w1 = segment.weight(segment.cv_count() - 1);
        let normalized = fraction * w0 / (w1 * (1.0 - fraction) + fraction * w0);
        let candidate = spans[i - 1] + normalized * (spans[i] - spans[i - 1]);
        let gap = curve.point_at(candidate).distance(point, None);

        if gap < best {
            best = gap;
            t = candidate;
        }
    }

    Ok((t, curve.point_at(t).distance(point, None)))
}

/// Closest parameter and distance from a point to a curve, polished by Newton steps.
fn closest(curve: &NurbsCurve, point: &Point) -> Result<(f64, f64), String> {
    let mut t = Closest::curve_point(curve, point, 0.0, 0.0).0;

    if curve.degree() == 1 {
        return closest_segments(curve, point, t);
    }

    let lo = curve.domain_start();
    let hi = curve.domain_end();

    for _ in 0..24 {
        let eval = curve.evaluate(t, 1);
        let d = &eval[1];
        let r = &eval[0] - &Vector::new(point[0], point[1], point[2]);
        let dd = d.dot(d);

        if dd <= EPSILON * EPSILON {
            break;
        }

        let next = (t - d.dot(&r) / dd).clamp(lo, hi);

        if (next - t).abs() <= EPSILON * (hi - lo) {
            t = next;
            break;
        }

        t = next;
    }

    Ok((t, curve.point_at(t).distance(point, None)))
}

/// Sorted parameters clamped to [lo, hi], dropping near duplicates.
fn unique_parameters(mut values: Vec<f64>, lo: f64, hi: f64) -> Vec<f64> {
    values.sort_by(f64::total_cmp);
    let mut result = Vec::<f64>::new();

    for value in values {
        let value = value.clamp(lo, hi);

        if result.is_empty() || value - result[result.len() - 1] > (hi - lo) * EPSILON * 16.0 {
            result.push(value);
        }
    }

    result
}

// ═══════════════════════════════════════════════════════════════════════════
// Curve intersection
// ═══════════════════════════════════════════════════════════════════════════
/// True when every control point lies within tolerance of the chord.
fn flat(curve: &NurbsCurve, tolerance: f64) -> Result<bool, String> {
    let a = curve.point_at_start();
    let b = curve.point_at_end();
    let v = &b - &a;
    let length2 = v.dot(&v);

    if length2 <= tolerance * tolerance {
        return Ok(Bounds::new(curve)?.diagonal() <= tolerance);
    }

    for i in 0..curve.cv_count() {
        let p = curve.get_cv(i).ok_or("Missing curve control")?;
        let t = (&p - &a).dot(&v) / length2;

        if !(-EPSILON..=1.0 + EPSILON).contains(&t) || p.distance(&(&a + &v * t), None) > tolerance
        {
            return Ok(false);
        }
    }

    Ok(true)
}

/// Newton refinement of a curve-curve intersection seed.
fn refine(a: &NurbsCurve, b: &NurbsCurve, mut ta: f64, mut tb: f64) -> (f64, f64) {
    let a0 = a.domain_start();
    let a1 = a.domain_end();
    let b0 = b.domain_start();
    let b1 = b.domain_end();

    for _ in 0..40 {
        let da = a.evaluate(ta, 1);
        let db = b.evaluate(tb, 1);
        let r = &da[0] - &db[0];
        let u = &da[1];
        let v = &db[1];
        let aa = u.dot(u);
        let ab = u.dot(v);
        let bb = v.dot(v);
        let det = aa * bb - ab * ab;

        if det <= EPSILON * EPSILON * aa * bb {
            break;
        }

        let ar = u.dot(&r);
        let br = v.dot(&r);
        let na = (ta + (-bb * ar + ab * br) / det).clamp(a0, a1);
        let nb = (tb + (-ab * ar + aa * br) / det).clamp(b0, b1);
        let converged =
            (na - ta).abs() < EPSILON * (a1 - a0) && (nb - tb).abs() < EPSILON * (b1 - b0);
        ta = na;
        tb = nb;

        if converged {
            break;
        }
    }

    (ta, tb)
}

/// Err when two flat pieces overlap along a shared line.
fn check_overlap(pair: &Pair, tolerance: f64) -> Result<(), String> {
    let ap = pair.a.point_at_start();
    let aq = pair.a.point_at_end();
    let bp = pair.b.point_at_start();
    let bq = pair.b.point_at_end();
    let u = &aq - &ap;
    let v = &bq - &bp;
    let aa = u.dot(&u);
    let ab = u.dot(&v);
    let vv = v.dot(&v);
    let parallel = aa > tolerance * tolerance
        && vv > tolerance * tolerance
        && aa * vv - ab * ab < EPSILON * EPSILON * aa * vv;

    if !parallel {
        return Ok(());
    }

    let t0 = (&bp - &ap).dot(&u) / aa;
    let t1 = (&bq - &ap).dot(&u) / aa;
    let gap = bp.distance(&(&ap + &u * t0), None);
    let shared = 1.0_f64.min(t0.max(t1)) - 0.0_f64.max(t0.min(t1));

    if gap <= tolerance && shared > tolerance / aa.sqrt() {
        return Err("Overlapping curves do not define isolated split points".into());
    }

    Ok(())
}

/// True when a hit is already recorded within tolerance on both curves.
fn duplicate(
    a: &NurbsCurve,
    b: &NurbsCurve,
    hits: &[(f64, f64)],
    ta: f64,
    tb: f64,
    tolerance: f64,
) -> bool {
    for hit in hits {
        let near_a = a.point_at(hit.0).distance(&a.point_at(ta), None) <= tolerance * 2.0
            && a.point_at((hit.0 + ta) * 0.5)
                .distance(&a.point_at(ta), None)
                <= tolerance * 2.0;
        let near_b = b.point_at(hit.1).distance(&b.point_at(tb), None) <= tolerance * 2.0
            && b.point_at((hit.1 + tb) * 0.5)
                .distance(&b.point_at(tb), None)
                <= tolerance * 2.0;

        if near_a && near_b {
            return true;
        }
    }

    false
}

/// Record the refined crossing of two flat pieces unless it is a duplicate.
fn add_hit(
    a: &NurbsCurve,
    b: &NurbsCurve,
    pair: &Pair,
    tolerance: f64,
    hits: &mut Vec<(f64, f64)>,
) -> Result<(), String> {
    check_overlap(pair, tolerance)?;
    let (ta, tb, d) = Closest::curve_curve(&pair.a, &pair.b);

    if d > tolerance * 2.0 {
        return Ok(());
    }

    let (ta, tb) = refine(&pair.a, &pair.b, ta, tb);

    if a.point_at(ta).distance(&b.point_at(tb), None) > tolerance {
        return Ok(());
    }

    if !duplicate(a, b, hits, ta, tb, tolerance) {
        hits.push((ta, tb));
    }

    Ok(())
}

/// Halve the piece with the larger box at its parameter midpoint.
fn subdivide(pair: Pair, ba: &Bounds, bb: &Bounds, work: &mut Vec<Pair>) -> Result<(), String> {
    if ba.diagonal() >= bb.diagonal() {
        let lo = pair.a.domain_start();
        let hi = pair.a.domain_end();
        let mid = (lo + hi) * 0.5;
        work.push(Pair {
            a: interval(&pair.a, lo, mid)?,
            b: pair.b.clone(),
            depth: pair.depth + 1,
        });
        work.push(Pair {
            a: interval(&pair.a, mid, hi)?,
            b: pair.b,
            depth: pair.depth + 1,
        });
        return Ok(());
    }

    let lo = pair.b.domain_start();
    let hi = pair.b.domain_end();
    let mid = (lo + hi) * 0.5;
    work.push(Pair {
        a: pair.a.clone(),
        b: interval(&pair.b, lo, mid)?,
        depth: pair.depth + 1,
    });
    work.push(Pair {
        a: pair.a,
        b: interval(&pair.b, mid, hi)?,
        depth: pair.depth + 1,
    });

    Ok(())
}

/// Sorted parameter pairs where two curves cross, drawing on a shared work budget.
fn intersections(
    a: &NurbsCurve,
    b: &NurbsCurve,
    tolerance: f64,
    budget: &mut usize,
) -> Result<Vec<(f64, f64)>, String> {
    let av = a.get_span_vector();
    let bv = b.get_span_vector();
    require(
        av.len() > 1 && bv.len() > 1,
        "Split requires nonempty curve spans",
    )?;
    require(
        av.len() - 1 <= *budget / (bv.len() - 1),
        "Curve intersection exceeds the bounded split workload",
    )?;
    let mut work = Vec::<Pair>::new();

    for i in 1..av.len() {
        for j in 1..bv.len() {
            work.push(Pair {
                a: interval(a, av[i - 1], av[i])?,
                b: interval(b, bv[j - 1], bv[j])?,
                depth: 0,
            });
        }
    }

    let mut hits = Vec::<(f64, f64)>::new();

    while let Some(pair) = work.pop() {
        require(
            *budget > 0,
            "Curve intersection exceeds the bounded split workload",
        )?;
        *budget -= 1;
        let ba = Bounds::new(&pair.a)?;
        let bb = Bounds::new(&pair.b)?;

        if !ba.overlaps(&bb, tolerance) {
            continue;
        }

        if (flat(&pair.a, tolerance * 0.1)? && flat(&pair.b, tolerance * 0.1)?) || pair.depth >= 48
        {
            add_hit(a, b, &pair, tolerance, &mut hits)?;
        } else {
            subdivide(pair, &ba, &bb, &mut work)?;
        }
    }

    hits.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    Ok(hits)
}

// ═══════════════════════════════════════════════════════════════════════════
// Trim polygons
// ═══════════════════════════════════════════════════════════════════════════
/// Curves in surface parameter space, exact on a bilinear parallelogram patch.
fn pullback(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    tolerance: f64,
) -> Result<Vec<NurbsCurve>, String> {
    let bilinear = surface.m_cv_count[0] == 2
        && surface.m_cv_count[1] == 2
        && surface.m_order[0] == 2
        && surface.m_order[1] == 2
        && !surface.m_is_rat;

    if !bilinear {
        return Ok(Closest::surface_curve(surface, curve, 0.0, 0.0, tolerance));
    }

    let p = surface.get_cv(0, 0).ok_or("Missing surface control")?;
    let u = &surface.get_cv(1, 0).ok_or("Missing surface control")? - &p;
    let v = &surface.get_cv(0, 1).ok_or("Missing surface control")? - &p;
    let last = surface.get_cv(1, 1).ok_or("Missing surface control")?;
    let uu = u.dot(&u);
    let uv = u.dot(&v);
    let vv = v.dot(&v);
    let det = uu * vv - uv * uv;
    let parallelogram =
        det > EPSILON * EPSILON * uu * vv && last.distance(&(&(&p + &u) + &v), None) <= tolerance;

    if !parallelogram {
        return Ok(Closest::surface_curve(surface, curve, 0.0, 0.0, tolerance));
    }

    let mut result = curve.duplicate();
    let (u0, u1) = surface_domain(surface, 0)?;
    let (v0, v1) = surface_domain(surface, 1)?;

    for i in 0..curve.cv_count() {
        let q = curve.get_cv(i).ok_or("Missing curve control")?;
        let d = &q - &p;
        let du = d.dot(&u);
        let dv = d.dot(&v);
        let a = (du * vv - dv * uv) / det;
        let b = (dv * uu - du * uv) / det;

        if q.distance(&(&p + &u * a + &v * b), None) > tolerance {
            return Ok(vec![]);
        }

        let w = curve.weight(i);
        require(
            result.set_cv_4d(
                i,
                (u0 + a * (u1 - u0)) * w,
                (v0 + b * (v1 - v0)) * w,
                0.0,
                w,
            ),
            "Kernel refused a pullback control",
        )?;
    }

    Ok(vec![result])
}

/// Points sampling a curve until each piece is flat within tolerance.
fn polygon(curve: &NurbsCurve, tolerance: f64) -> Result<Vec<Point>, String> {
    let spans = curve.get_span_vector();
    let mut work = Vec::<Part>::new();

    for i in (2..=spans.len()).rev() {
        work.push(Part {
            curve: interval(curve, spans[i - 2], spans[i - 1])?,
            depth: 0,
        });
    }

    let mut result = Vec::<Point>::new();
    let mut visited = 0;

    while let Some(part) = work.pop() {
        visited += 1;
        require(
            visited <= WORK_LIMIT,
            "Trim sampling exceeds the bounded workload",
        )?;

        if flat(&part.curve, tolerance * 0.25)? {
            result.push(part.curve.point_at_start());
            continue;
        }

        require(part.depth < 40, "Trim sampling exceeds parameter precision")?;
        let lo = part.curve.domain_start();
        let hi = part.curve.domain_end();
        let mid = (lo + hi) * 0.5;
        work.push(Part {
            curve: interval(&part.curve, mid, hi)?,
            depth: part.depth + 1,
        });
        work.push(Part {
            curve: interval(&part.curve, lo, mid)?,
            depth: part.depth + 1,
        });
    }

    Ok(result)
}

/// Even-odd point in polygon test in the xy plane.
fn inside(p: &Point, polygon: &[Point]) -> bool {
    let mut result = false;
    let mut j = polygon.len() - 1;

    for i in 0..polygon.len() {
        let a = &polygon[i];
        let b = &polygon[j];

        if (a[1] > p[1]) != (b[1] > p[1])
            && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            result = !result;
        }

        j = i;
    }

    result
}

/// True inside the first loop and outside every hole loop.
fn inside_loops(p: &Point, loops: &[Vec<Point>]) -> bool {
    if loops.is_empty() || !inside(p, &loops[0]) {
        return false;
    }

    for hole in &loops[1..] {
        if inside(p, hole) {
            return false;
        }
    }

    true
}

// ═══════════════════════════════════════════════════════════════════════════
// Trim arrangement
// ═══════════════════════════════════════════════════════════════════════════
/// Knot spans of every source, cut at their mutual intersections.
fn compute_spans(sources: &[Source], tolerance: f64) -> Result<Vec<Span>, String> {
    let mut spans = Vec::<Span>::new();

    for (si, source) in sources.iter().enumerate() {
        let mut knots = source.uv.get_span_vector();

        if knots.len() == 2 && source.uv.is_closed() {
            let lo = knots[0];
            let hi = knots[knots.len() - 1];
            knots = vec![
                lo,
                lo + (hi - lo) * 0.25,
                (lo + hi) * 0.5,
                lo + (hi - lo) * 0.75,
                hi,
            ];
        }

        for i in 1..knots.len() {
            spans.push(Span {
                source: si,
                a: knots[i - 1],
                b: knots[i],
                cuts: vec![knots[i - 1], knots[i]],
                curve: interval(&source.uv, knots[i - 1], knots[i])?,
            });
        }
    }

    require(
        !spans.is_empty() && spans.len() <= WORK_LIMIT / spans.len(),
        "Face split exceeds the bounded workload",
    )?;
    let mut budget = WORK_LIMIT;

    for i in 0..spans.len() {
        for j in i + 1..spans.len() {
            let hits = intersections(&spans[i].curve, &spans[j].curve, tolerance, &mut budget)?;

            for hit in hits {
                spans[i].cuts.push(hit.0);
                spans[j].cuts.push(hit.1);
            }
        }
    }

    Ok(spans)
}

/// Cut parameters of a span, snapped to its ends and deduplicated.
fn span_cuts(span: &Span, tolerance: f64) -> Vec<f64> {
    let mut cuts = span.cuts.clone();

    for t in &mut cuts {
        let p = span.curve.point_at(*t);

        if p.distance(&span.curve.point_at(span.a), None) <= tolerance {
            *t = span.a;
        } else if p.distance(&span.curve.point_at(span.b), None) <= tolerance {
            *t = span.b;
        }
    }

    unique_parameters(cuts, span.a, span.b)
}

/// Index of the graph vertex at a point, added when none lies within tolerance.
fn node(graph: &mut Graph, vertices: &mut Vec<Point>, p: Point, tolerance: f64) -> usize {
    for (i, vertex) in vertices.iter().enumerate() {
        if p.distance(vertex, None) <= tolerance * 4.0 {
            return i;
        }
    }

    vertices.push(p);
    graph.outgoing.push(vec![]);
    vertices.len() - 1
}

/// Direction angle of a run leaving its start vertex.
fn angle(sources: &[Source], run: &Run) -> f64 {
    let d = &sources[run.source].uv.evaluate(run.a, 1)[1];
    let sign = if run.b > run.a { 1.0 } else { -1.0 };
    (sign * d[1]).atan2(sign * d[0])
}

/// Half-edge graph of the cut spans inside the original loops.
fn compute_graph(
    spans: &[Span],
    sources: &[Source],
    original_loops: &[Vec<Point>],
    tolerance: f64,
) -> Graph {
    let mut graph = Graph {
        edges: vec![],
        outgoing: vec![],
    };
    let mut vertices = Vec::<Point>::new();

    for span in spans {
        let cuts = span_cuts(span, tolerance);
        let source = &sources[span.source];

        for i in 1..cuts.len() {
            let lo = cuts[i - 1];
            let hi = cuts[i];

            if source.edge < 0
                && !inside_loops(&source.uv.point_at((lo + hi) * 0.5), original_loops)
            {
                continue;
            }

            let a = node(&mut graph, &mut vertices, source.uv.point_at(lo), tolerance);
            let b = node(&mut graph, &mut vertices, source.uv.point_at(hi), tolerance);

            if a == b {
                continue;
            }

            let index = graph.edges.len();
            graph.edges.push(Directed {
                b,
                run: Run {
                    source: span.source,
                    a: lo,
                    b: hi,
                },
            });
            graph.edges.push(Directed {
                b: a,
                run: Run {
                    source: span.source,
                    a: hi,
                    b: lo,
                },
            });
            graph.outgoing[a].push(index);
            graph.outgoing[b].push(index + 1);
        }
    }

    let mut angles = Vec::<f64>::new();

    for edge in &graph.edges {
        angles.push(angle(sources, &edge.run));
    }

    for choices in &mut graph.outgoing {
        choices.sort_by(|a, b| angles[*a].total_cmp(&angles[*b]));
    }

    graph
}

/// Signed shoelace area of a closed polygon in the xy plane.
fn signed_area(points: &[Point]) -> f64 {
    let mut area = 0.0;

    for i in 0..points.len() {
        let a = &points[i];
        let b = &points[(i + 1) % points.len()];
        area += (a[0] * b[1] - b[0] * a[1]) * 0.5;
    }

    area
}

/// Loop traced from one half-edge by turning to the previous outgoing half-edge at each vertex.
fn trace_cycle(
    graph: &Graph,
    sources: &[Source],
    initial: usize,
    used: &mut [bool],
    tolerance: f64,
) -> Result<Cycle, String> {
    let mut cycle = Cycle {
        area: 0.0,
        loop_: vec![],
        points: vec![],
    };
    let mut edge = initial;

    while !used[edge] {
        used[edge] = true;
        let item = &graph.edges[edge];
        let run = item.run;
        cycle.loop_.push(run);
        let mut part = interval(&sources[run.source].uv, run.a.min(run.b), run.a.max(run.b))?;

        if run.b < run.a {
            require(part.reverse(), "Kernel refused to reverse a trim fragment")?;
        }

        cycle.points.extend(polygon(&part, tolerance)?);
        let options = &graph.outgoing[item.b];
        let mut slot = options.len();

        for (k, option) in options.iter().enumerate() {
            if *option == edge ^ 1 {
                slot = k;
                break;
            }
        }

        require(slot < options.len(), "Invalid trim graph adjacency")?;
        edge = options[(slot + options.len() - 1) % options.len()];
    }

    require(edge == initial, "Invalid trim graph cycle")?;
    cycle.area = signed_area(&cycle.points);
    Ok(cycle)
}

/// Point eight tolerances left of the middle of a run.
fn left_of(sources: &[Source], run: &Run, tolerance: f64) -> Result<Point, String> {
    let curve = &sources[run.source].uv;
    let t = (run.a + run.b) * 0.5;
    let p = curve.point_at(t);
    let d = &curve.evaluate(t, 1)[1];
    let sign = if run.b > run.a { 1.0 } else { -1.0 };
    let length = d[0].hypot(d[1]);
    require(length > EPSILON, "Cannot orient a degenerate trim fragment")?;

    Ok(Point::new(
        p[0] - sign * d[1] / length * tolerance * 8.0,
        p[1] + sign * d[0] / length * tolerance * 8.0,
        0.0,
    ))
}

/// Non-degenerate graph cycles whose interior lies inside the original loops.
fn compute_cycles(
    graph: &Graph,
    sources: &[Source],
    original_loops: &[Vec<Point>],
    tolerance: f64,
) -> Result<Vec<Cycle>, String> {
    let mut cycles = Vec::<Cycle>::new();
    let mut used = vec![false; graph.edges.len()];

    for initial in 0..graph.edges.len() {
        if used[initial] {
            continue;
        }

        let cycle = trace_cycle(graph, sources, initial, &mut used, tolerance)?;

        if cycle.area.abs() <= tolerance * tolerance {
            continue;
        }

        if inside_loops(
            &left_of(sources, &cycle.loop_[0], tolerance)?,
            original_loops,
        ) {
            cycles.push(cycle);
        }
    }

    Ok(cycles)
}

/// Regions as outer loops, each hole nested in the smallest outer loop around it.
fn nest_cycles(cycles: &[Cycle], tolerance: f64) -> Result<Vec<Vec<Vec<Run>>>, String> {
    let mut result = Vec::<Vec<Vec<Run>>>::new();
    let mut positive = Vec::<usize>::new();

    for (i, cycle) in cycles.iter().enumerate() {
        if cycle.area > 0.0 {
            positive.push(i);
            result.push(vec![cycle.loop_.clone()]);
        }
    }

    for cycle in cycles {
        if cycle.area >= 0.0 {
            continue;
        }

        let mut parent = result.len();
        let mut smallest = f64::INFINITY;

        for i in 0..positive.len() {
            let outer = &cycles[positive[i]];

            if outer.area > cycle.area.abs() + tolerance * tolerance
                && outer.area < smallest
                && inside(&cycle.points[0], &outer.points)
            {
                parent = i;
                smallest = outer.area;
            }
        }

        require(parent < result.len(), "Unowned interior trim loop")?;
        result[parent].push(cycle.loop_.clone());
    }

    Ok(result)
}

/// Regions of the planar arrangement of the sources inside the original loops.
fn arrange(
    sources: &[Source],
    original_loops: &[Vec<Point>],
    tolerance: f64,
) -> Result<Vec<Vec<Vec<Run>>>, String> {
    let spans = compute_spans(sources, tolerance)?;
    let graph = compute_graph(&spans, sources, original_loops, tolerance);
    let cycles = compute_cycles(&graph, sources, original_loops, tolerance)?;
    nest_cycles(&cycles, tolerance)
}

// ═══════════════════════════════════════════════════════════════════════════
// BRep assembly
// ═══════════════════════════════════════════════════════════════════════════
/// Index of the BRep vertex at a point, added when none lies within tolerance.
fn vertex(result: &mut BRep, p: &Point, tolerance: f64) -> usize {
    for (i, vertex) in result.m_vertices.iter().enumerate() {
        if vertex.point.distance(p, None) <= tolerance {
            return i;
        }
    }

    result.add_vertex(p, tolerance)
}

/// Distance from a point to the surface under a pcurve parameter.
fn lifted_gap(surface: &NurbsSurface, uv: &NurbsCurve, p: &Point, t: f64) -> Result<f64, String> {
    let q = uv.point_at(t);
    Ok(surface_point(surface, q[0], q[1])?.distance(p, None))
}

/// Pcurve parameter whose surface point meets a world point, sampled then refined by ternary search.
fn lifted_parameter(
    surface: &NurbsSurface,
    uv: &NurbsCurve,
    p: &Point,
    expected: f64,
    tolerance: f64,
) -> Result<f64, String> {
    let lo = uv.domain_start();
    let hi = uv.domain_end();

    if lifted_gap(surface, uv, p, expected)? <= tolerance {
        return Ok(expected);
    }

    let mut best = expected;
    let mut d = lifted_gap(surface, uv, p, best)?;
    let mut index = 0usize;

    for i in 0..=128 {
        let t = lo + (hi - lo) * i as f64 / 128.0;
        let value = lifted_gap(surface, uv, p, t)?;

        if value < d {
            d = value;
            best = t;
            index = i;
        }
    }

    let mut a = lo + (hi - lo) * index.saturating_sub(1) as f64 / 128.0;
    let mut b = lo + (hi - lo) * (index + 1).min(128) as f64 / 128.0;

    for _ in 0..60 {
        let x = a + (b - a) / 3.0;
        let y = b - (b - a) / 3.0;

        if lifted_gap(surface, uv, p, x)? < lifted_gap(surface, uv, p, y)? {
            b = y;
        } else {
            a = x;
        }
    }

    let mid = (a + b) * 0.5;

    if lifted_gap(surface, uv, p, mid)? < d {
        best = mid;
    }

    require(
        lifted_gap(surface, uv, p, best)? <= tolerance * 4.0,
        "Cannot keep an adjacent trim on its original shared edge",
    )?;
    Ok(best)
}

/// World curve parameter and distance for a surface point, proportional guess first.
fn world_parameter(
    source: &Source,
    t: f64,
    p: &Point,
    tolerance: f64,
) -> Result<(f64, f64), String> {
    let lo = source.world.domain_start();
    let hi = source.world.domain_end();
    let a = source.uv.domain_start();
    let b = source.uv.domain_end();
    let expected = lo + (t - a) / (b - a) * (hi - lo);
    let gap = source.world.point_at(expected).distance(p, None);

    if gap <= tolerance {
        return Ok((expected, gap));
    }

    closest(&source.world, p)
}

/// World curve parameters of both run ends, a closed curve's seam end moved to the domain end.
fn world_run(
    surface: &NurbsSurface,
    source: &Source,
    run: &Run,
    tolerance: f64,
) -> Result<(f64, f64), String> {
    let qa = source.uv.point_at(run.a);
    let qb = source.uv.point_at(run.b);
    let (mut wa, da) = world_parameter(
        source,
        run.a,
        &surface_point(surface, qa[0], qa[1])?,
        tolerance,
    )?;
    let (mut wb, db) = world_parameter(
        source,
        run.b,
        &surface_point(surface, qb[0], qb[1])?,
        tolerance,
    )?;
    require(
        da <= tolerance * 4.0 && db <= tolerance * 4.0,
        "Cutter is not on the selected surface",
    )?;

    let w0 = source.world.domain_start();
    let w1 = source.world.domain_end();
    let c0 = source.uv.domain_start();
    let c1 = source.uv.domain_end();

    if source.world.is_closed() {
        if (wa - w0).abs() < (w1 - w0) * EPSILON && run.a > (c0 + c1) * 0.5 {
            wa = w1;
        }

        if (wb - w0).abs() < (w1 - w0) * EPSILON && run.b > (c0 + c1) * 0.5 {
            wb = w1;
        }
    }

    Ok((wa, wb))
}

/// Pcurves of a piece of a shared BRep edge, cut from every adjacent face trim.
fn add_shared_pcurves(
    result: &mut BRep,
    brep: &BRep,
    source: &Source,
    piece: &Piece,
    world: &NurbsCurve,
    tolerance: f64,
) -> Result<(), String> {
    let w0 = source.world.domain_start();
    let w1 = source.world.domain_end();

    for pc in &brep.m_edges[source.edge as usize].pcurves {
        let surface = &brep.m_surfaces[pc.surface_index as usize];
        let sides = [pc.curve_2d_index, pc.curve_2d_index_2];
        let mut ids = [-1, -1];

        for at in 0..2 {
            if sides[at] < 0 {
                continue;
            }

            let c = &brep.m_curves_2d[sides[at] as usize];
            let c0 = c.domain_start();
            let c1 = c.domain_end();
            let ca = lifted_parameter(
                surface,
                c,
                &world.point_at_start(),
                c0 + (piece.lo - w0) / (w1 - w0) * (c1 - c0),
                tolerance,
            )?;
            let cb = lifted_parameter(
                surface,
                c,
                &world.point_at_end(),
                c0 + (piece.hi - w0) / (w1 - w0) * (c1 - c0),
                tolerance,
            )?;
            require(cb > ca, "A split crosses an unsupported periodic trim seam")?;
            ids[at] = result.add_curve_2d(&interval(c, ca, cb)?) as i32;
        }

        result.add_pcurve(
            piece.edge as usize,
            pc.surface_index as usize,
            ids[0],
            ids[1],
        );
    }

    Ok(())
}

/// Oriented BRep edge for a run, reusing the edge already cut for the same stretch of its source.
fn add_run_edge(
    result: &mut BRep,
    pieces: &mut Vec<Piece>,
    brep: &BRep,
    surface_index: usize,
    sources: &[Source],
    run: &Run,
    tolerance: f64,
) -> Result<BRepRef, String> {
    let source = &sources[run.source];
    let (wa, wb) = world_run(&brep.m_surfaces[surface_index], source, run, tolerance)?;
    let lo = wa.min(wb);
    let hi = wa.max(wb);
    let w0 = source.world.domain_start();
    let w1 = source.world.domain_end();
    require(
        hi - lo > (w1 - w0) * EPSILON,
        "Split would create a collapsed edge",
    )?;
    let orientation = if wa < wb { FORWARD } else { REVERSED };

    for piece in pieces.iter() {
        let same = if source.edge >= 0 {
            sources[piece.source].edge == source.edge
        } else {
            piece.source == run.source
        };

        if same
            && source
                .world
                .point_at(lo)
                .distance(&source.world.point_at(piece.lo), None)
                <= tolerance * 4.0
            && source
                .world
                .point_at(hi)
                .distance(&source.world.point_at(piece.hi), None)
                <= tolerance * 4.0
        {
            return Ok(BRepRef::new(piece.edge, orientation));
        }
    }

    let world = interval(&source.world, lo, hi)?;
    let a = vertex(result, &world.point_at_start(), tolerance * 4.0);
    let b = vertex(result, &world.point_at_end(), tolerance * 4.0);
    let ci = result.add_curve_3d(&world);
    let edge = result.add_edge(ci as i32, a as i32, b as i32);
    result.m_edges[edge].tolerance = tolerance;
    let piece = Piece {
        source: run.source,
        lo,
        hi,
        edge: edge as i32,
    };

    if source.edge >= 0 {
        add_shared_pcurves(result, brep, source, &piece, &world, tolerance)?;
    } else {
        let mut pc = interval(&source.uv, run.a.min(run.b), run.a.max(run.b))?;

        if (wb - wa) * (run.b - run.a) < 0.0 {
            require(pc.reverse(), "Kernel refused to reverse a cutter trim")?;
        }

        let pi = result.add_curve_2d(&pc);
        result.add_pcurve(edge, surface_index, pi as i32, -1);
    }

    pieces.push(piece);
    Ok(BRepRef::new(edge as i32, orientation))
}

/// Sampled trim loops of a face, each boundary edge added to the sources.
fn boundary_loops(
    brep: &BRep,
    face_index: usize,
    uv_tolerance: f64,
    sources: &mut Vec<Source>,
) -> Result<Vec<Vec<Point>>, String> {
    let mut loops = Vec::<Vec<Point>>::new();

    for wr in &brep.m_faces[face_index].wires {
        let mut points = Vec::<Point>::new();

        for er in brep.wire_edges(wr) {
            let edge = &brep.m_edges[er.index as usize];
            require(!edge.degenerated, "Pole-edge splitting is not supported")?;
            let ci = brep.pcurve_index(er.index as usize, face_index, er.orientation);
            require(ci >= 0, "Face has no source UV boundary")?;
            let mut uv = brep.m_curves_2d[ci as usize].clone();
            check_curve(&uv)?;
            check_curve(&brep.m_curves_3d[edge.curve_3d_index as usize])?;
            sources.push(Source {
                edge: er.index,
                world: brep.m_curves_3d[edge.curve_3d_index as usize].clone(),
                uv: uv.clone(),
            });

            if er.orientation == REVERSED {
                require(uv.reverse(), "Kernel refused to reverse a face trim")?;
            }

            points.extend(polygon(&uv, uv_tolerance)?);
        }

        require(points.len() >= 3, "Face has an invalid boundary")?;
        loops.push(points);
    }

    Ok(loops)
}

/// Replace every use of a cut boundary edge in the wires by its pieces in order.
fn replace_wires(result: &mut BRep, brep: &BRep, sources: &[Source], pieces: &[Piece]) {
    let mut replacements = BTreeMap::<i32, Vec<(f64, i32)>>::new();

    for piece in pieces {
        if sources[piece.source].edge >= 0 {
            replacements
                .entry(sources[piece.source].edge)
                .or_default()
                .push((piece.lo, piece.edge));
        }
    }

    for wi in 0..brep.m_wires.len() {
        let mut refs = Vec::<BRepRef>::new();

        for er in &brep.m_wires[wi].edges {
            if !replacements.contains_key(&er.index) {
                refs.push(*er);
                continue;
            }

            let mut items = replacements[&er.index].clone();
            items.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
            items.dedup();

            if er.orientation == REVERSED {
                items.reverse();
            }

            for item in &items {
                refs.push(BRepRef::new(item.1, er.orientation));
            }
        }

        result.m_wires[wi].edges = refs;
    }
}

/// Put the first region on the split face and the others on new faces beside it in every shell.
fn add_faces(result: &mut BRep, face: &BRepFace, face_index: usize, new_wires: &[Vec<BRepRef>]) {
    result.m_faces[face_index].wires = new_wires[0].clone();
    let mut added = Vec::<usize>::new();

    for wires in &new_wires[1..] {
        let mut next = face.clone();
        next.wires = wires.clone();
        added.push(result.face_count());
        result.m_faces.push(next);
    }

    for shell in &mut result.m_shells {
        let mut refs = Vec::<BRepRef>::new();

        for fr in &shell.faces {
            refs.push(*fr);

            if fr.index == face_index as i32 {
                for index in &added {
                    refs.push(BRepRef::new(*index as i32, fr.orientation));
                }
            }
        }

        shell.faces = refs;
    }
}

/// Reject a face boundary whose consecutive edges do not share a vertex.
fn check_wires(result: &BRep) -> Result<(), String> {
    for face in &result.m_faces {
        for wr in &face.wires {
            let edges = result.wire_edges(wr);

            for i in 0..edges.len() {
                let a = &result.m_edges[edges[i].index as usize];
                let next = edges[(i + 1) % edges.len()];
                let b = &result.m_edges[next.index as usize];
                let tail = if edges[i].orientation == REVERSED {
                    a.start_vertex
                } else {
                    a.end_vertex
                };
                let head = if next.orientation == REVERSED {
                    b.end_vertex
                } else {
                    b.start_vertex
                };
                require(tail == head, "Split produced an open face boundary")?;
            }
        }
    }

    Ok(())
}

/// Reject a split that opens a shell, leaves a wire open or moves an edge off its vertices or trims.
fn validate(result: &BRep, original: &BRep, tolerance: f64) -> Result<(), String> {
    require(result.is_valid(), "Split produced invalid BRep references")?;

    for s in 0..original.m_shells.len() {
        if original.is_closed(s) {
            require(result.is_closed(s), "Split would open a joined shell")?;
        }
    }

    check_wires(result)?;

    for edge in &result.m_edges {
        if edge.degenerated {
            continue;
        }

        let world = &result.m_curves_3d[edge.curve_3d_index as usize];
        let start = world
            .point_at_start()
            .distance(&result.m_vertices[edge.start_vertex as usize].point, None);
        let end = world
            .point_at_end()
            .distance(&result.m_vertices[edge.end_vertex as usize].point, None);
        require(
            start <= tolerance * 4.0 && end <= tolerance * 4.0,
            "Split edge does not meet its vertices",
        )?;

        for pc in &edge.pcurves {
            for ci in [pc.curve_2d_index, pc.curve_2d_index_2] {
                if ci < 0 {
                    continue;
                }

                let uv = &result.m_curves_2d[ci as usize];
                let lo = uv.domain_start();
                let hi = uv.domain_end();

                for k in 0..=32 {
                    let q = uv.point_at(lo + (hi - lo) * k as f64 / 32.0);
                    let p =
                        surface_point(&result.m_surfaces[pc.surface_index as usize], q[0], q[1])?;
                    require(
                        closest(world, &p)?.1 <= tolerance.max(edge.tolerance) * 8.0,
                        "Split edge and surface trim do not coincide",
                    )?;
                }
            }
        }
    }

    Ok(())
}

/// Parameter-space tolerance of a surface: tolerance over its larger world length per unit parameter.
fn surface_uv_tolerance(surface: &NurbsSurface, tolerance: f64) -> Result<f64, String> {
    let (u0, u1) = surface_domain(surface, 0)?;
    let (v0, v1) = surface_domain(surface, 1)?;
    let origin = surface_point(surface, u0, v0)?;
    let scale = (origin.distance(&surface_point(surface, u1, v0)?, None) / (u1 - u0))
        .max(origin.distance(&surface_point(surface, u0, v1)?, None) / (v1 - v0));
    require(scale > EPSILON, "Cannot split a degenerate surface domain")?;

    Ok(tolerance / scale)
}

/// Wires of every region, one new edge per run added to result.
fn region_wires(
    result: &mut BRep,
    pieces: &mut Vec<Piece>,
    brep: &BRep,
    surface_index: usize,
    sources: &[Source],
    regions: &[Vec<Vec<Run>>],
    tolerance: f64,
) -> Result<Vec<Vec<BRepRef>>, String> {
    let mut new_wires = Vec::<Vec<BRepRef>>::new();

    for region in regions {
        let mut wires = Vec::<BRepRef>::new();

        for loop_ in region {
            let mut refs = Vec::<BRepRef>::new();

            for run in loop_ {
                refs.push(add_run_edge(
                    result,
                    pieces,
                    brep,
                    surface_index,
                    sources,
                    run,
                    tolerance,
                )?);
            }

            let wi = result.add_wire(&refs);
            wires.push(BRepRef::new(wi as i32, FORWARD));
        }

        new_wires.push(wires);
    }

    Ok(new_wires)
}

// ═══════════════════════════════════════════════════════════════════════════
// Split
// ═══════════════════════════════════════════════════════════════════════════
/// Split a curve at isolated 3D intersections, retaining every piece and rejecting overlapping cutters.
pub fn split_curve_by_curves(
    curve: &NurbsCurve,
    cutters: &[NurbsCurve],
    tolerance: f64,
) -> Result<Vec<NurbsCurve>, String> {
    check_tolerance(tolerance)?;
    check_curve(curve)?;
    require(!cutters.is_empty(), "Select at least one cutter")?;
    let lo = curve.domain_start();
    let hi = curve.domain_end();
    let mut cuts = vec![lo, hi];
    let mut cut_at_seam = false;
    let mut budget = WORK_LIMIT;

    for cutter in cutters {
        check_curve(cutter)?;
        let hits = intersections(curve, cutter, tolerance, &mut budget)?;

        for hit in hits {
            let a = hit.0;

            if (a - lo).abs() <= (hi - lo) * EPSILON * 16.0
                || (a - hi).abs() <= (hi - lo) * EPSILON * 16.0
            {
                cut_at_seam = true;
            }

            cuts.push(a);
        }
    }

    let cuts = unique_parameters(cuts, lo, hi);

    if cuts.len() == 2 {
        return Ok(vec![curve.clone()]);
    }

    let mut result = Vec::<NurbsCurve>::new();

    for i in 1..cuts.len() {
        result.push(interval(curve, cuts[i - 1], cuts[i])?);
    }

    if curve.is_closed() && result.len() > 1 && !cut_at_seam {
        let joined = NurbsCurve::join(
            &[result[result.len() - 1].clone(), result[0].clone()],
            Some(tolerance),
        );
        require(
            joined.len() == 1,
            "Cannot join the uncut seam of a closed curve",
        )?;
        result[0] = joined[0].clone();
        result.pop();
    }

    Ok(result)
}

/// Partition one face inside its owning BRep, retaining all regions and shared shell topology.
pub fn split_brep_face_by_curves(
    brep: &BRep,
    face_index: usize,
    cutters: &[NurbsCurve],
    tolerance: f64,
) -> Result<BRep, String> {
    check_tolerance(tolerance)?;
    require(brep.is_valid(), "Split requires a valid BRep")?;
    require(
        face_index < brep.face_count(),
        "Select one BRep face to split",
    )?;
    require(!cutters.is_empty(), "Select at least one cutter")?;
    let face = &brep.m_faces[face_index];
    let surface = &brep.m_surfaces[face.surface_index as usize];
    check_surface(surface)?;

    let uv_tolerance = surface_uv_tolerance(surface, tolerance)?;

    let mut sources = Vec::<Source>::new();
    let original_loops = boundary_loops(brep, face_index, uv_tolerance, &mut sources)?;

    for cutter in cutters {
        check_curve(cutter)?;

        for uv in pullback(surface, cutter, tolerance)? {
            sources.push(Source {
                edge: -1,
                world: cutter.clone(),
                uv,
            });
        }
    }

    let regions = arrange(&sources, &original_loops, uv_tolerance)?;

    if regions.len() < 2 {
        return Ok(brep.clone());
    }

    let mut result = brep.clone();
    let mut pieces = Vec::<Piece>::new();
    let new_wires = region_wires(
        &mut result,
        &mut pieces,
        brep,
        face.surface_index as usize,
        &sources,
        &regions,
        tolerance,
    )?;
    replace_wires(&mut result, brep, &sources, &pieces);
    add_faces(&mut result, face, face_index, &new_wires);
    validate(&result, brep, tolerance)?;

    Ok(result)
}

/// Wrap a surface's natural boundary in a BRep and partition it with on-surface curves.
pub fn split_surface_by_curves(
    surface: &NurbsSurface,
    cutters: &[NurbsCurve],
    tolerance: f64,
) -> Result<BRep, String> {
    check_tolerance(tolerance)?;
    check_surface(surface)?;
    let mut result = BRep::new();
    let si = result.add_surface(surface);
    let (u0, u1) = surface_domain(surface, 0)?;
    let (v0, v1) = surface_domain(surface, 1)?;
    let uv = [
        Point::new(u0, v0, 0.0),
        Point::new(u1, v0, 0.0),
        Point::new(u1, v1, 0.0),
        Point::new(u0, v1, 0.0),
    ];
    let at = [v0, u1, v1, u0];
    let mut edges = Vec::<BRepRef>::new();

    for i in 0..4 {
        let mut curve = surface
            .iso_curve(i % 2, at[i])
            .ok_or("Cannot extract a natural boundary")?;

        if i >= 2 {
            require(
                curve.reverse(),
                "Kernel refused to reverse a natural boundary",
            )?;
        }

        let a = vertex(&mut result, &curve.point_at_start(), tolerance);
        let b = vertex(&mut result, &curve.point_at_end(), tolerance);
        require(
            a != b,
            "Closed or pole boundaries need a BRep with explicit seam topology",
        )?;
        let ci = result.add_curve_3d(&curve);
        let edge = result.add_edge(ci as i32, a as i32, b as i32);
        result.m_edges[edge].tolerance = tolerance;
        let pc = NurbsCurve::create(false, 1, &[uv[i].clone(), uv[(i + 1) % 4].clone()]);
        let pi = result.add_curve_2d(&pc);
        result.add_pcurve(edge, si, pi as i32, -1);
        edges.push(BRepRef::new(edge as i32, FORWARD));
    }

    let wi = result.add_wire(&edges);
    result.add_face(si as i32, &[BRepRef::new(wi as i32, FORWARD)], 0.0);
    split_brep_face_by_curves(&result, 0, cutters, tolerance)
}

/// Split a line at isolated 3D intersections, retaining line types and display attributes.
pub fn split_line_by_curves(
    line: &Line,
    cutters: &[NurbsCurve],
    tolerance: f64,
) -> Result<Vec<Line>, String> {
    let curve = NurbsCurve::create(false, 1, &[line.point_at(0.0), line.point_at(1.0)]);
    let mut result = Vec::<Line>::new();

    for piece in split_curve_by_curves(&curve, cutters, tolerance)? {
        let mut next = Line::from_points(&piece.point_at_start(), &piece.point_at_end());
        next.name = line.name.clone();
        next.width = line.width;
        next.dash = line.dash.clone();
        next.linecolor = line.linecolor.clone();
        result.push(next);
    }

    Ok(result)
}

/// Split a polyline, retaining each original corner, piece order and display attributes.
pub fn split_polyline_by_curves(
    polyline: &Polyline,
    cutters: &[NurbsCurve],
    tolerance: f64,
) -> Result<Vec<Polyline>, String> {
    let curve = NurbsCurve::create(false, 1, &polyline.get_points());
    let mut result = Vec::<Polyline>::new();

    for piece in split_curve_by_curves(&curve, cutters, tolerance)? {
        let mut points = Vec::<Point>::new();

        for t in piece.get_span_vector() {
            points.push(piece.point_at(t));
        }

        let mut next = Polyline::new(points);
        next.name = polyline.name.clone();
        next.width = polyline.width;
        next.dash = polyline.dash.clone();
        next.linecolor = polyline.linecolor.clone();
        result.push(next);
    }

    Ok(result)
}
