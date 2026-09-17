use crate::brep::BRep;
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

const EPSILON: f64 = Tolerance::ZERO_TOLERANCE;
const FORWARD: BRepOrientation = BRepOrientation::Forward;
const REVERSED: BRepOrientation = BRepOrientation::Reversed;
const WORK_LIMIT: usize = 200000;

fn require(condition: bool, message: &str) -> Result<(), String> {
    if !condition {
        return Err(message.into());
    }
    Ok(())
}
fn check_tolerance(tolerance: f64) -> Result<(), String> {
    require(
        tolerance.is_finite() && tolerance > 0.0,
        "Split tolerance must be finite and positive",
    )
}
fn check_curve(curve: &NurbsCurve) -> Result<(), String> {
    require(curve.is_valid(), "Split requires valid curves")?;
    for i in 0..curve.cv_count() {
        let p = curve.get_cv(i).ok_or("Split requires valid controls")?;
        require(
            p[0].is_finite()
                && p[1].is_finite()
                && p[2].is_finite()
                && curve.weight(i).is_finite()
                && curve.weight(i) > 0.0,
            "Split requires finite controls and positive rational weights",
        )?;
    }
    Ok(())
}
fn check_surface(surface: &NurbsSurface) -> Result<(), String> {
    require(surface.is_valid(), "Split requires a valid NURBS surface")?;
    for i in 0..surface.m_cv_count[0] {
        for j in 0..surface.m_cv_count[1] {
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
fn interval(curve: &NurbsCurve, a: f64, b: f64) -> Result<NurbsCurve, String> {
    let mut result = curve.duplicate();
    let (lo, hi) = curve.domain();
    let a = a.clamp(lo, hi);
    let b = b.clamp(lo, hi);
    require(b > a, "Split produced an empty curve interval")?;
    if a > lo || b < hi {
        require(result.trim(a, b), "Kernel refused a split interval")?;
    }
    Ok(result)
}
fn surface_point(surface: &NurbsSurface, u: f64, v: f64) -> Result<Point, String> {
    surface
        .point_at(u, v)
        .ok_or_else(|| "Cannot evaluate the source surface".into())
}
fn closest(curve: &NurbsCurve, point: &Point) -> Result<(f64, f64), String> {
    let (mut t, _) = Closest::curve_point(curve, point, 0.0, 0.0);
    let (lo, hi) = curve.domain();
    if curve.degree() == 1 {
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
        return Ok((t, curve.point_at(t).distance(point, None)));
    }
    for _ in 0..24 {
        let eval = curve.evaluate(t, 1);
        let d = eval[1].clone();
        let r = &eval[0] - &Vector::new(point[0], point[1], point[2]);
        let dd = d.dot(&d);
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

struct Box3 {
    lo: [f64; 3],
    hi: [f64; 3],
}
impl Box3 {
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
    fn diagonal(&self) -> f64 {
        (self.hi[0] - self.lo[0])
            .hypot(self.hi[1] - self.lo[1])
            .hypot(self.hi[2] - self.lo[2])
    }
    fn overlaps(&self, other: &Self, tolerance: f64) -> bool {
        for d in 0..3 {
            if self.hi[d] + tolerance < other.lo[d] || other.hi[d] + tolerance < self.lo[d] {
                return false;
            }
        }
        true
    }
}
fn flat(curve: &NurbsCurve, tolerance: f64) -> Result<bool, String> {
    let a = curve.point_at_start();
    let b = curve.point_at_end();
    let v = &b - &a;
    let length2 = v.dot(&v);
    if length2 <= tolerance * tolerance {
        return Ok(Box3::new(curve)?.diagonal() <= tolerance);
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
fn refine(a: &NurbsCurve, b: &NurbsCurve, mut ta: f64, mut tb: f64) -> (f64, f64) {
    let (a0, a1) = a.domain();
    let (b0, b1) = b.domain();
    for _ in 0..40 {
        let da = a.evaluate(ta, 1);
        let db = b.evaluate(tb, 1);
        let r = &da[0] - &db[0];
        let u = da[1].clone();
        let v = db[1].clone();
        let aa = u.dot(&u);
        let ab = u.dot(&v);
        let bb = v.dot(&v);
        let det = aa * bb - ab * ab;
        if det <= EPSILON * EPSILON * aa * bb {
            break;
        }
        let ar = u.dot(&r);
        let br = v.dot(&r);
        let na = (ta + (-bb * ar + ab * br) / det).clamp(a0, a1);
        let nb = (tb + (-ab * ar + aa * br) / det).clamp(b0, b1);
        if (na - ta).abs() < EPSILON * (a1 - a0) && (nb - tb).abs() < EPSILON * (b1 - b0) {
            ta = na;
            tb = nb;
            break;
        }
        ta = na;
        tb = nb;
    }
    (ta, tb)
}
fn intersections(
    a: &NurbsCurve,
    b: &NurbsCurve,
    tolerance: f64,
    budget: &mut usize,
) -> Result<Vec<(f64, f64)>, String> {
    struct Pair {
        a: NurbsCurve,
        b: NurbsCurve,
        depth: usize,
    }
    let mut work = Vec::<Pair>::new();
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
        let ba = Box3::new(&pair.a)?;
        let bb = Box3::new(&pair.b)?;
        if !ba.overlaps(&bb, tolerance) {
            continue;
        }
        if (flat(&pair.a, tolerance * 0.1)? && flat(&pair.b, tolerance * 0.1)?) || pair.depth >= 48
        {
            let ap = pair.a.point_at_start();
            let aq = pair.a.point_at_end();
            let bp = pair.b.point_at_start();
            let bq = pair.b.point_at_end();
            let u = &aq - &ap;
            let v = &bq - &bp;
            let aa = u.dot(&u);
            let ab = u.dot(&v);
            let vv = v.dot(&v);
            if aa > tolerance * tolerance
                && vv > tolerance * tolerance
                && aa * vv - ab * ab < EPSILON * EPSILON * aa * vv
            {
                let t0 = (&bp - &ap).dot(&u) / aa;
                let t1 = (&bq - &ap).dot(&u) / aa;
                let gap = bp.distance(&(&ap + &u * t0), None);
                if gap <= tolerance
                    && 1.0_f64.min(t0.max(t1)) - 0.0_f64.max(t0.min(t1)) > tolerance / aa.sqrt()
                {
                    return Err("Overlapping curves do not define isolated split points".into());
                }
            }
            let (ta, tb, d) = Closest::curve_curve(&pair.a, &pair.b);
            if d > tolerance * 2.0 {
                continue;
            }
            let (ta, tb) = refine(&pair.a, &pair.b, ta, tb);
            if a.point_at(ta).distance(&b.point_at(tb), None) > tolerance {
                continue;
            }
            let mut duplicate = false;
            for hit in &hits {
                if a.point_at(hit.0).distance(&a.point_at(ta), None) <= tolerance * 2.0
                    && a.point_at((hit.0 + ta) * 0.5)
                        .distance(&a.point_at(ta), None)
                        <= tolerance * 2.0
                    && b.point_at(hit.1).distance(&b.point_at(tb), None) <= tolerance * 2.0
                    && b.point_at((hit.1 + tb) * 0.5)
                        .distance(&b.point_at(tb), None)
                        <= tolerance * 2.0
                {
                    duplicate = true;
                    break;
                }
            }
            if !duplicate {
                hits.push((ta, tb));
            }
            continue;
        }
        if ba.diagonal() >= bb.diagonal() {
            let (lo, hi) = pair.a.domain();
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
        } else {
            let (lo, hi) = pair.b.domain();
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
        }
    }
    hits.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    Ok(hits)
}

fn pullback(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    tolerance: f64,
) -> Result<Vec<NurbsCurve>, String> {
    if surface.m_cv_count == [2, 2] && surface.m_order == [2, 2] && !surface.m_is_rat {
        let p = surface.get_cv(0, 0).ok_or("Missing surface control")?;
        let u = &surface.get_cv(1, 0).ok_or("Missing surface control")? - &p;
        let v = &surface.get_cv(0, 1).ok_or("Missing surface control")? - &p;
        let last = surface.get_cv(1, 1).ok_or("Missing surface control")?;
        let uu = u.dot(&u);
        let uv = u.dot(&v);
        let vv = v.dot(&v);
        let det = uu * vv - uv * uv;
        if det > EPSILON * EPSILON * uu * vv && last.distance(&(&p + (&u + &v)), None) <= tolerance
        {
            let mut result = curve.duplicate();
            let (u0, u1) = surface.domain(0).ok_or("Invalid surface domain")?;
            let (v0, v1) = surface.domain(1).ok_or("Invalid surface domain")?;
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
                result.set_cv_4d(
                    i,
                    (u0 + a * (u1 - u0)) * w,
                    (v0 + b * (v1 - v0)) * w,
                    0.0,
                    w,
                );
            }
            return Ok(vec![result]);
        }
    }
    Ok(Closest::surface_curve(surface, curve, 0.0, 0.0, tolerance))
}

fn polygon(curve: &NurbsCurve, tolerance: f64) -> Result<Vec<Point>, String> {
    struct Part {
        curve: NurbsCurve,
        depth: usize,
    }
    let mut work = Vec::<Part>::new();
    let spans = curve.get_span_vector();
    for i in (2..=spans.len()).rev() {
        work.push(Part {
            curve: interval(curve, spans[i - 2], spans[i - 1])?,
            depth: 0,
        });
    }
    let mut result = vec![];
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
        let (lo, hi) = part.curve.domain();
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
fn inside_loops(p: &Point, loops: &[Vec<Point>]) -> bool {
    if loops.is_empty() || !inside(p, &loops[0]) {
        return false;
    }
    for hole in loops.iter().skip(1) {
        if inside(p, hole) {
            return false;
        }
    }
    true
}
struct Source {
    edge: i32,
    world: NurbsCurve,
    uv: NurbsCurve,
}
#[derive(Clone, Copy)]
struct Run {
    source: usize,
    a: f64,
    b: f64,
}
type Loop = Vec<Run>;
type Region = Vec<Loop>;

fn node(
    vertices: &mut Vec<Point>,
    outgoing: &mut Vec<Vec<usize>>,
    p: Point,
    tolerance: f64,
) -> usize {
    for (i, q) in vertices.iter().enumerate() {
        if p.distance(q, None) <= tolerance * 4.0 {
            return i;
        }
    }
    vertices.push(p);
    outgoing.push(vec![]);
    vertices.len() - 1
}
fn arrange(
    sources: &[Source],
    original_loops: &[Vec<Point>],
    tolerance: f64,
) -> Result<Vec<Region>, String> {
    struct Span {
        source: usize,
        a: f64,
        b: f64,
        cuts: Vec<f64>,
        curve: NurbsCurve,
    }
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
            for (a, b) in intersections(&spans[i].curve, &spans[j].curve, tolerance, &mut budget)? {
                spans[i].cuts.push(a);
                spans[j].cuts.push(b);
            }
        }
    }
    struct Directed {
        b: usize,
        run: Run,
    }
    let mut vertices = Vec::<Point>::new();
    let mut edges = Vec::<Directed>::new();
    let mut outgoing = Vec::<Vec<usize>>::new();
    for span in &mut spans {
        for k in 0..span.cuts.len() {
            let p = span.curve.point_at(span.cuts[k]);
            if p.distance(&span.curve.point_at(span.a), None) <= tolerance {
                span.cuts[k] = span.a;
            } else if p.distance(&span.curve.point_at(span.b), None) <= tolerance {
                span.cuts[k] = span.b;
            }
        }
        let cuts = unique_parameters(span.cuts.clone(), span.a, span.b);
        for i in 1..cuts.len() {
            let lo = cuts[i - 1];
            let hi = cuts[i];
            let source = &sources[span.source];
            if source.edge < 0
                && !inside_loops(&source.uv.point_at((lo + hi) * 0.5), original_loops)
            {
                continue;
            }
            let a = node(
                &mut vertices,
                &mut outgoing,
                source.uv.point_at(lo),
                tolerance,
            );
            let b = node(
                &mut vertices,
                &mut outgoing,
                source.uv.point_at(hi),
                tolerance,
            );
            if a == b {
                continue;
            }
            let index = edges.len();
            edges.push(Directed {
                b,
                run: Run {
                    source: span.source,
                    a: lo,
                    b: hi,
                },
            });
            edges.push(Directed {
                b: a,
                run: Run {
                    source: span.source,
                    a: hi,
                    b: lo,
                },
            });
            outgoing[a].push(index);
            outgoing[b].push(index + 1);
        }
    }
    let angle = |edge: usize| {
        let run = edges[edge].run;
        let d = sources[run.source].uv.evaluate(run.a, 1)[1].clone();
        let sign = if run.b > run.a { 1.0 } else { -1.0 };
        (sign * d[1]).atan2(sign * d[0])
    };
    for choices in &mut outgoing {
        choices.sort_by(|&a, &b| angle(a).total_cmp(&angle(b)));
    }
    struct Cycle {
        area: f64,
        loop_: Loop,
        points: Vec<Point>,
    }
    let mut cycles = Vec::<Cycle>::new();
    let mut used = vec![false; edges.len()];
    for initial in 0..edges.len() {
        if used[initial] {
            continue;
        }
        let mut loop_ = Loop::new();
        let mut points = Vec::<Point>::new();
        let mut edge = initial;
        while !used[edge] {
            used[edge] = true;
            let item = &edges[edge];
            let run = item.run;
            loop_.push(run);
            let mut part = interval(&sources[run.source].uv, run.a.min(run.b), run.a.max(run.b))?;
            if run.b < run.a {
                part.reverse();
            }
            points.extend(polygon(&part, tolerance)?);
            let options = &outgoing[item.b];
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
        let mut area = 0.0;
        for i in 0..points.len() {
            let a = &points[i];
            let b = &points[(i + 1) % points.len()];
            area += (a[0] * b[1] - b[0] * a[1]) * 0.5;
        }
        if area.abs() <= tolerance * tolerance {
            continue;
        }
        let run = loop_[0];
        let curve = &sources[run.source].uv;
        let t = (run.a + run.b) * 0.5;
        let p = curve.point_at(t);
        let d = curve.evaluate(t, 1)[1].clone();
        let sign = if run.b > run.a { 1.0 } else { -1.0 };
        let length = d[0].hypot(d[1]);
        require(length > EPSILON, "Cannot orient a degenerate trim fragment")?;
        let left = Point::new(
            p[0] - sign * d[1] / length * tolerance * 8.0,
            p[1] + sign * d[0] / length * tolerance * 8.0,
            0.0,
        );
        if !inside_loops(&left, original_loops) {
            continue;
        }
        cycles.push(Cycle {
            area,
            loop_,
            points,
        });
    }
    let mut result = Vec::<Region>::new();
    let mut positive = Vec::<usize>::new();
    for (i, cycle) in cycles.iter().enumerate() {
        if cycle.area > 0.0 {
            positive.push(i);
            result.push(vec![cycle.loop_.clone()]);
        }
    }
    for cycle in &cycles {
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

fn vertex(result: &mut BRep, p: &Point, tolerance: f64) -> usize {
    for (i, v) in result.m_vertices.iter().enumerate() {
        if v.point.distance(p, None) <= tolerance {
            return i;
        }
    }
    result.add_vertex(p, tolerance)
}

fn lifted_parameter(
    surface: &NurbsSurface,
    uv: &NurbsCurve,
    p: &Point,
    expected: f64,
    tolerance: f64,
) -> Result<f64, String> {
    let (lo, hi) = uv.domain();
    let gap = |t: f64| -> Result<f64, String> {
        let q = uv.point_at(t);
        Ok(surface_point(surface, q[0], q[1])?.distance(p, None))
    };
    if gap(expected)? <= tolerance {
        return Ok(expected);
    }
    let mut best = expected;
    let mut d = gap(best)?;
    let mut index = 0usize;
    for i in 0..=128 {
        let t = lo + (hi - lo) * i as f64 / 128.0;
        let value = gap(t)?;
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
        if gap(x)? < gap(y)? {
            b = y;
        } else {
            a = x;
        }
    }
    let mid = (a + b) * 0.5;
    if gap(mid)? < d {
        best = mid;
    }
    require(
        gap(best)? <= tolerance * 4.0,
        "Cannot keep an adjacent trim on its original shared edge",
    )?;
    Ok(best)
}

fn validate(result: &BRep, original: &BRep, tolerance: f64) -> Result<(), String> {
    require(result.is_valid(), "Split produced invalid BRep references")?;
    for s in 0..original.m_shells.len() {
        if original.is_closed(s) {
            require(result.is_closed(s), "Split would open a joined shell")?;
        }
    }
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
    for edge in &result.m_edges {
        if edge.degenerated {
            continue;
        }
        let world = &result.m_curves_3d[edge.curve_3d_index as usize];
        require(
            world
                .point_at_start()
                .distance(&result.m_vertices[edge.start_vertex as usize].point, None)
                <= tolerance * 4.0
                && world
                    .point_at_end()
                    .distance(&result.m_vertices[edge.end_vertex as usize].point, None)
                    <= tolerance * 4.0,
            "Split edge does not meet its vertices",
        )?;
        for pc in &edge.pcurves {
            for ci in [pc.curve_2d_index, pc.curve_2d_index_2] {
                if ci < 0 {
                    continue;
                }
                let uv = &result.m_curves_2d[ci as usize];
                let (lo, hi) = uv.domain();
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

/// Split a curve at isolated 3D intersections, retaining every piece and rejecting overlapping cutters.
pub fn split_curve_by_curves(
    curve: &NurbsCurve,
    cutters: &[NurbsCurve],
    tolerance: f64,
) -> Result<Vec<NurbsCurve>, String> {
    check_tolerance(tolerance)?;
    check_curve(curve)?;
    require(!cutters.is_empty(), "Select at least one cutter")?;
    let (lo, hi) = curve.domain();
    let mut cuts = vec![lo, hi];
    let mut cut_at_seam = false;
    let mut budget = WORK_LIMIT;
    for cutter in cutters {
        check_curve(cutter)?;
        for hit in intersections(curve, cutter, tolerance, &mut budget)? {
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
    let mut result = Vec::<NurbsCurve>::new();
    if cuts.len() == 2 {
        return Ok(vec![curve.clone()]);
    }
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
    let (u0, u1) = surface.domain(0).ok_or("Invalid surface domain")?;
    let (v0, v1) = surface.domain(1).ok_or("Invalid surface domain")?;
    let origin = surface_point(surface, u0, v0)?;
    let scale = (origin.distance(&surface_point(surface, u1, v0)?, None) / (u1 - u0))
        .max(origin.distance(&surface_point(surface, u0, v1)?, None) / (v1 - v0));
    require(scale > EPSILON, "Cannot split a degenerate surface domain")?;
    let uv_tolerance = tolerance / scale;
    let mut sources = Vec::<Source>::new();
    let mut original_loops = Vec::<Vec<Point>>::new();
    for wr in &face.wires {
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
                uv.reverse();
            }
            points.extend(polygon(&uv, uv_tolerance)?);
        }
        require(points.len() >= 3, "Face has an invalid boundary")?;
        original_loops.push(points);
    }
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
    struct Piece {
        source: usize,
        lo: f64,
        hi: f64,
        edge: i32,
    }
    let mut pieces = Vec::<Piece>::new();
    let mut replacements = BTreeMap::<i32, Vec<(f64, i32)>>::new();
    let mut make_edge = |result: &mut BRep, run: &Run| -> Result<BRepRef, String> {
        let source = &sources[run.source];
        let uv = &source.uv;
        let qa = uv.point_at(run.a);
        let qb = uv.point_at(run.b);
        let pa = surface_point(surface, qa[0], qa[1])?;
        let pb = surface_point(surface, qb[0], qb[1])?;
        let parameter = |t: f64, p: &Point| -> Result<(f64, f64), String> {
            let (lo, hi) = source.world.domain();
            let (a, b) = uv.domain();
            let expected = lo + (t - a) / (b - a) * (hi - lo);
            let gap = source.world.point_at(expected).distance(p, None);
            if gap <= tolerance {
                return Ok((expected, gap));
            }
            closest(&source.world, p)
        };
        let (mut wa, da) = parameter(run.a, &pa)?;
        let (mut wb, db) = parameter(run.b, &pb)?;
        require(
            da <= tolerance * 4.0 && db <= tolerance * 4.0,
            "Cutter is not on the selected surface",
        )?;
        let (w0, w1) = source.world.domain();
        let (c0, c1) = uv.domain();
        if source.world.is_closed() {
            if (wa - w0).abs() < (w1 - w0) * EPSILON && run.a > (c0 + c1) * 0.5 {
                wa = w1;
            }
            if (wb - w0).abs() < (w1 - w0) * EPSILON && run.b > (c0 + c1) * 0.5 {
                wb = w1;
            }
        }
        let lo = wa.min(wb);
        let hi = wa.max(wb);
        require(
            hi - lo > (w1 - w0) * EPSILON,
            "Split would create a collapsed edge",
        )?;
        let orientation = if wa < wb { FORWARD } else { REVERSED };
        for piece in &pieces {
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
        let ei = result.add_edge(ci as i32, a as i32, b as i32);
        result.m_edges[ei].tolerance = tolerance;
        if source.edge >= 0 {
            let old = &brep.m_edges[source.edge as usize];
            for pc in &old.pcurves {
                let mut ids = [-1, -1];
                for (at, ci) in [pc.curve_2d_index, pc.curve_2d_index_2]
                    .into_iter()
                    .enumerate()
                {
                    if ci >= 0 {
                        let c = &brep.m_curves_2d[ci as usize];
                        let (c0, c1) = c.domain();
                        let ca = lifted_parameter(
                            &brep.m_surfaces[pc.surface_index as usize],
                            c,
                            &world.point_at_start(),
                            c0 + (lo - w0) / (w1 - w0) * (c1 - c0),
                            tolerance,
                        )?;
                        let cb = lifted_parameter(
                            &brep.m_surfaces[pc.surface_index as usize],
                            c,
                            &world.point_at_end(),
                            c0 + (hi - w0) / (w1 - w0) * (c1 - c0),
                            tolerance,
                        )?;
                        require(cb > ca, "A split crosses an unsupported periodic trim seam")?;
                        ids[at] = result.add_curve_2d(&interval(c, ca, cb)?) as i32;
                    }
                }
                result.add_pcurve(ei, pc.surface_index as usize, ids[0], ids[1]);
            }
            replacements
                .entry(source.edge)
                .or_default()
                .push((lo, ei as i32));
        } else {
            let mut pc = interval(uv, run.a.min(run.b), run.a.max(run.b))?;
            if (wb - wa) * (run.b - run.a) < 0.0 {
                pc.reverse();
            }
            let ci = result.add_curve_2d(&pc);
            result.add_pcurve(ei, face.surface_index as usize, ci as i32, -1);
        }
        pieces.push(Piece {
            source: run.source,
            lo,
            hi,
            edge: ei as i32,
        });
        Ok(BRepRef::new(ei as i32, orientation))
    };
    let mut new_wires = Vec::<Vec<BRepRef>>::new();
    for region in &regions {
        let mut wires = Vec::<BRepRef>::new();
        for loop_ in region {
            let mut refs = Vec::<BRepRef>::new();
            for run in loop_ {
                refs.push(make_edge(&mut result, run)?);
            }
            let wi = result.add_wire(&refs);
            wires.push(BRepRef::new(wi as i32, FORWARD));
        }
        new_wires.push(wires);
    }
    for items in replacements.values_mut() {
        items.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        items.dedup();
    }
    for wi in 0..brep.m_wires.len() {
        let mut refs = Vec::<BRepRef>::new();
        for er in &brep.m_wires[wi].edges {
            let Some(found) = replacements.get(&er.index) else {
                refs.push(*er);
                continue;
            };
            let mut items = found.clone();
            if er.orientation == REVERSED {
                items.reverse();
            }
            for item in &items {
                refs.push(BRepRef::new(item.1, er.orientation));
            }
        }
        result.m_wires[wi].edges = refs;
    }
    result.m_faces[face_index].wires = new_wires[0].clone();
    let mut added = Vec::<usize>::new();
    for wires in new_wires.iter().skip(1) {
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
    let (u0, u1) = surface.domain(0).ok_or("Invalid surface domain")?;
    let (v0, v1) = surface.domain(1).ok_or("Invalid surface domain")?;
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
            curve.reverse();
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
        let ci = result.add_curve_2d(&pc);
        result.add_pcurve(edge, si, ci as i32, -1);
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
