use std::collections::BTreeMap;

use crate::mesh::Mesh;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
use crate::tolerance::Tolerance;
use crate::vector::Vector;

/// Adaptive mesh of a NURBS surface: a quadtree in UV split where normals turn or chords deviate, T-junctions fanned, poles and seams shared.
pub struct RemeshNurbsSurfaceAdaptive {
    surface: NurbsSurface, // Surface to mesh.
    max_angle: f64,        // Largest normal turn across a cell in degrees.
    max_edge_length: f64,  // Longest cell edge, 0 for no limit.
    min_edge_length: f64,  // Shortest cell edge still split, 0 for no limit.
    max_chord_height: f64, // Largest chord height, 0 for 0.5 percent of the bbox diagonal.
}

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════
const MAX_DEPTH: usize = 8;
const STACK_SIZE: usize = 64;
const KEY_SCALE: f64 = 1e10;

/// Surface sample: position and unit normal, zero where the surface has none.
#[derive(Clone)]
struct Corner {
    p: Point,  // Surface point.
    n: Vector, // Unit normal, zero at a pole.
}

/// Quadtree cell: UV bounds, corners SW SE NE NW and the centre.
#[derive(Clone)]
struct Node {
    u0: f64,        // Lower u bound.
    v0: f64,        // Lower v bound.
    u1: f64,        // Upper u bound.
    v1: f64,        // Upper v bound.
    c: [Corner; 5], // Corners SW SE NE NW and the centre.
    depth: usize,   // Subdivision depth from the root.
    leaf: bool,     // True until the cell is split.
}

/// Quadtree over the UV domain: the surface it samples, its tolerances, the cells, the leaf corners by key and the mesh it emits.
struct Quadtree<'a> {
    s: &'a NurbsSurface,                   // Surface sampled.
    usp: Vec<f64>,                         // Span vector in u.
    vsp: Vec<f64>,                         // Span vector in v.
    closed: [bool; 2],                     // True per direction when the surface closes on itself.
    norm_tol: f64,                         // Normal turn tolerance in squared length.
    chord_tol: f64,                        // Chord height tolerance.
    max_edge: f64,                         // Longest cell edge, 0 for no limit.
    min_edge: f64,                         // Shortest cell edge still split, 0 for no limit.
    nodes: Vec<Node>,                      // Cell pool, root cells first.
    corners: BTreeMap<(i64, i64), Corner>, // Leaf corners by key.
    rows: BTreeMap<i64, Vec<i64>>,         // U keys on each row.
    cols: BTreeMap<i64, Vec<i64>>,         // V keys on each column.
    mesh: Mesh,                            // Mesh emitted.
    keys: BTreeMap<(i64, i64), usize>,     // Mesh vertex per key.
    south: Option<usize>,                  // Pole vertex on the v0 side.
    north: Option<usize>,                  // Pole vertex on the v1 side.
}

impl<'a> Quadtree<'a> {
    /// Construct over a surface with its span vectors, seam flags and tolerances.
    fn new(
        s: &'a NurbsSurface,
        norm_tol: f64,
        chord_tol: f64,
        max_edge: f64,
        min_edge: f64,
    ) -> Self {
        Quadtree {
            s,
            usp: s.get_span_vector(0),
            vsp: s.get_span_vector(1),
            closed: [s.is_closed(0), s.is_closed(1)],
            norm_tol,
            chord_tol,
            max_edge,
            min_edge,
            nodes: Vec::new(),
            corners: BTreeMap::new(),
            rows: BTreeMap::new(),
            cols: BTreeMap::new(),
            mesh: Mesh::new(),
            keys: BTreeMap::new(),
            south: None,
            north: None,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Sampling
// ═══════════════════════════════════════════════════════════════════════════
/// Euclidean length without the zero gate of magnitude().
fn norm(v: &Vector) -> f64 {
    v.magnitude_squared().sqrt()
}

/// Squared distance between two points.
fn dist2(a: &Point, b: &Point) -> f64 {
    (a - b).magnitude_squared()
}

/// Midpoint of two points.
fn midpoint(a: &Point, b: &Point) -> Point {
    Point::sum(a, b) * 0.5
}

/// Diagonal of the control point bounding box.
fn bbox_diagonal(s: &NurbsSurface) -> f64 {
    let mut lo = Point::new(1e30, 1e30, 1e30);
    let mut hi = Point::new(-1e30, -1e30, -1e30);

    for i in 0..s.cv_count(0) {
        for j in 0..s.cv_count(1) {
            let p = s.get_cv(i, j).unwrap_or_default();

            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
    }

    norm(&(hi - lo))
}

/// Point and unit normal Su x Sv at (u, v); zero normal at a pole, where normal_at would give +Z.
fn sample(s: &NurbsSurface, u: f64, v: f64) -> Corner {
    let mut c = Corner {
        p: s.point_at(u, v).unwrap_or_default(),
        n: Vector::new(0.0, 0.0, 0.0),
    };
    let derivatives = s.evaluate(u, v, 1);

    if derivatives.len() < 3 {
        return c;
    }

    let n = derivatives[2].cross(&derivatives[1]);
    let length = norm(&n);

    if length > 1e-10 {
        c.n = n / length;
    }

    c
}

/// Leaf cell over [u0, u1] x [v0, v1] from its sampled corners SW SE NE NW, centre sampled here.
fn make_node(
    s: &NurbsSurface,
    u0: f64,
    v0: f64,
    u1: f64,
    v1: f64,
    corners: [Corner; 4],
    depth: usize,
) -> Node {
    let centre = sample(s, (u0 + u1) * 0.5, (v0 + v1) * 0.5);
    let [sw, se, ne, nw] = corners;

    Node {
        u0,
        v0,
        u1,
        v1,
        c: [sw, se, ne, nw, centre],
        depth,
        leaf: true,
    }
}

/// Edge midpoints S, E, N, W of the cell.
fn sample_edges(s: &NurbsSurface, p: &Node) -> [Corner; 4] {
    let um = (p.u0 + p.u1) * 0.5;
    let vm = (p.v0 + p.v1) * 0.5;

    [
        sample(s, um, p.v0),
        sample(s, p.u1, vm),
        sample(s, um, p.v1),
        sample(s, p.u0, vm),
    ]
}

// ═══════════════════════════════════════════════════════════════════════════
// Splitting
// ═══════════════════════════════════════════════════════════════════════════
/// True when both normals exist and differ by more than norm_tol in squared length.
fn normals_turn(a: &Corner, b: &Corner, norm_tol: f64) -> bool {
    if a.n.magnitude_squared() <= 1e-20 || b.n.magnitude_squared() <= 1e-20 {
        return false;
    }

    (&a.n - &b.n).magnitude_squared() > norm_tol
}

/// True when the edge midpoint sits more than chord_tol off the chord from a to b.
fn chord_off(mid: &Corner, a: &Corner, b: &Corner, chord_tol: f64) -> bool {
    dist2(&mid.p, &midpoint(&a.p, &b.p)) > chord_tol * chord_tol
}

/// True when every edge is shorter than min_edge, so the cell is not split further.
fn too_short(p: &Node, min_edge: f64) -> bool {
    if min_edge <= 0.0 {
        return false;
    }

    let mut longest = 0.0_f64;

    for i in 0..4 {
        longest = longest.max(dist2(&p.c[i].p, &p.c[(i + 1) % 4].p));
    }

    longest < min_edge * min_edge
}

/// True when the centre sits more than twice chord_tol off a diagonal midpoint; never on a cell with a collapsed edge.
fn twisted(p: &Node, chord_tol: f64) -> bool {
    for i in 0..4 {
        if dist2(&p.c[i].p, &p.c[(i + 1) % 4].p) < chord_tol * chord_tol {
            return false;
        }
    }

    let twist_tol2 = 4.0 * chord_tol * chord_tol;

    dist2(&p.c[4].p, &midpoint(&p.c[0].p, &p.c[2].p)) > twist_tol2
        || dist2(&p.c[4].p, &midpoint(&p.c[1].p, &p.c[3].p)) > twist_tol2
}

/// Per direction, true when the arc at the centre, taken as a circle of its normal curvature, rises more than chord_tol over its chord.
fn curved(s: &NurbsSurface, p: &Node, chord_tol: f64) -> (bool, bool) {
    let d = s.evaluate((p.u0 + p.u1) * 0.5, (p.v0 + p.v1) * 0.5, 2);

    if d.len() < 6 {
        return (false, false);
    }

    let su = &d[3];
    let sv = &d[1];
    let suu = &d[5];
    let svv = &d[2];
    let n = su.cross(sv);
    let length = norm(&n);
    let su2 = su.magnitude_squared();
    let sv2 = sv.magnitude_squared();

    if length <= 1e-10 || su2 <= 1e-20 || sv2 <= 1e-20 {
        return (false, false);
    }

    let unit = n * (1.0 / length);
    let kappa_u = suu.dot(&unit).abs() / su2;
    let kappa_v = svv.dot(&unit).abs() / sv2;
    let span_u = su2.sqrt() * (p.u1 - p.u0);
    let span_v = sv2.sqrt() * (p.v1 - p.v0);
    let curved_u = kappa_u > 1e-20 && span_u * (kappa_u / (8.0 * chord_tol)).sqrt() > 1.0;
    let curved_v = kappa_v > 1e-20 && span_v * (kappa_v / (8.0 * chord_tol)).sqrt() > 1.0;

    (curved_u, curved_v)
}

/// Directions to split: normals turning along an edge or from a corner to its midpoint, a midpoint off its chord, a twisted centre, an edge past max_edge, or the curvature at the centre.
fn split_flags(q: &Quadtree, p: &Node, mids: &[Corner; 4]) -> (bool, bool) {
    let sw = &p.c[0];
    let se = &p.c[1];
    let ne = &p.c[2];
    let nw = &p.c[3];

    let mut split_u = normals_turn(sw, se, q.norm_tol) || normals_turn(ne, nw, q.norm_tol);
    let mut split_v = normals_turn(se, ne, q.norm_tol) || normals_turn(nw, sw, q.norm_tol);

    split_u = split_u
        || chord_off(&mids[0], sw, se, q.chord_tol)
        || chord_off(&mids[2], nw, ne, q.chord_tol);
    split_u =
        split_u || normals_turn(&mids[0], sw, q.norm_tol) || normals_turn(&mids[2], nw, q.norm_tol);
    split_v = split_v
        || chord_off(&mids[3], sw, nw, q.chord_tol)
        || chord_off(&mids[1], se, ne, q.chord_tol);
    split_v =
        split_v || normals_turn(&mids[3], sw, q.norm_tol) || normals_turn(&mids[1], se, q.norm_tol);

    if !split_u && !split_v && twisted(p, q.chord_tol) {
        split_u = true;
        split_v = true;
    }

    if q.max_edge > 0.0 {
        let limit = q.max_edge * q.max_edge;

        split_u = split_u || dist2(&sw.p, &se.p) > limit || dist2(&ne.p, &nw.p) > limit;
        split_v = split_v || dist2(&se.p, &ne.p) > limit || dist2(&nw.p, &sw.p) > limit;
    }

    if !split_u || !split_v {
        let curvature = curved(q.s, p, q.chord_tol);

        split_u = split_u || curvature.0;
        split_v = split_v || curvature.1;
    }

    (split_u, split_v)
}

/// Children of cell idx appended to the pool: four quadrants, or two halves along the split direction.
fn split_node(q: &mut Quadtree, idx: usize, mids: &[Corner; 4], split_u: bool, split_v: bool) {
    let s = q.s;
    let p = q.nodes[idx].clone();
    let um = (p.u0 + p.u1) * 0.5;
    let vm = (p.v0 + p.v1) * 0.5;
    let depth = p.depth + 1;

    q.nodes[idx].leaf = false;

    if split_u && split_v {
        q.nodes.push(make_node(
            s,
            p.u0,
            p.v0,
            um,
            vm,
            [
                p.c[0].clone(),
                mids[0].clone(),
                p.c[4].clone(),
                mids[3].clone(),
            ],
            depth,
        ));
        q.nodes.push(make_node(
            s,
            um,
            p.v0,
            p.u1,
            vm,
            [
                mids[0].clone(),
                p.c[1].clone(),
                mids[1].clone(),
                p.c[4].clone(),
            ],
            depth,
        ));
        q.nodes.push(make_node(
            s,
            um,
            vm,
            p.u1,
            p.v1,
            [
                p.c[4].clone(),
                mids[1].clone(),
                p.c[2].clone(),
                mids[2].clone(),
            ],
            depth,
        ));
        q.nodes.push(make_node(
            s,
            p.u0,
            vm,
            um,
            p.v1,
            [
                mids[3].clone(),
                p.c[4].clone(),
                mids[2].clone(),
                p.c[3].clone(),
            ],
            depth,
        ));
    } else if split_u {
        q.nodes.push(make_node(
            s,
            p.u0,
            p.v0,
            um,
            p.v1,
            [
                p.c[0].clone(),
                mids[0].clone(),
                mids[2].clone(),
                p.c[3].clone(),
            ],
            depth,
        ));
        q.nodes.push(make_node(
            s,
            um,
            p.v0,
            p.u1,
            p.v1,
            [
                mids[0].clone(),
                p.c[1].clone(),
                p.c[2].clone(),
                mids[2].clone(),
            ],
            depth,
        ));
    } else {
        q.nodes.push(make_node(
            s,
            p.u0,
            p.v0,
            p.u1,
            vm,
            [
                p.c[0].clone(),
                p.c[1].clone(),
                mids[1].clone(),
                mids[3].clone(),
            ],
            depth,
        ));
        q.nodes.push(make_node(
            s,
            p.u0,
            vm,
            p.u1,
            p.v1,
            [
                mids[3].clone(),
                mids[1].clone(),
                p.c[2].clone(),
                p.c[3].clone(),
            ],
            depth,
        ));
    }
}

/// Cells split from root down to MAX_DEPTH over an explicit stack, first child popped first so the pool fills depth first.
fn subdivide(q: &mut Quadtree, root: usize) {
    let mut stack = [0usize; STACK_SIZE];
    let mut top = 0;

    stack[top] = root;
    top += 1;

    while top > 0 {
        top -= 1;

        let idx = stack[top];
        let p = q.nodes[idx].clone();

        if p.depth >= MAX_DEPTH || too_short(&p, q.min_edge) {
            continue;
        }

        let mids = sample_edges(q.s, &p);
        let split = split_flags(q, &p, &mids);

        if !split.0 && !split.1 {
            continue;
        }

        let first = q.nodes.len();

        split_node(q, idx, &mids, split.0, split.1);

        let count = q.nodes.len() - first;

        assert!(top + count <= STACK_SIZE);

        for i in (0..count).rev() {
            stack[top] = first + i;
            top += 1;
        }
    }
}

/// One root cell per span pair, corners from the grid of span intersections, each subdivided before the next.
fn build(q: &mut Quadtree) {
    let nu = q.usp.len();
    let nv = q.vsp.len();

    let mut grid: Vec<Corner> = Vec::with_capacity(nu * nv);

    for i in 0..nu {
        for j in 0..nv {
            grid.push(sample(q.s, q.usp[i], q.vsp[j]));
        }
    }

    for i in 0..nu - 1 {
        for j in 0..nv - 1 {
            let root = q.nodes.len();
            let corners = [
                grid[i * nv + j].clone(),
                grid[(i + 1) * nv + j].clone(),
                grid[(i + 1) * nv + j + 1].clone(),
                grid[i * nv + j + 1].clone(),
            ];

            q.nodes.push(make_node(
                q.s,
                q.usp[i],
                q.vsp[j],
                q.usp[i + 1],
                q.vsp[j + 1],
                corners,
                0,
            ));
            subdivide(q, root);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Vertices and faces
// ═══════════════════════════════════════════════════════════════════════════
/// t rounded at KEY_SCALE, so a parameter reached from two cells keys alike.
fn quantize(t: f64) -> i64 {
    (t * KEY_SCALE).round() as i64
}

/// t at the seam of a closed direction maps to the start of sp.
fn wrap(closed: bool, sp: &[f64], t: f64) -> f64 {
    if closed && (t - sp[sp.len() - 1]).abs() < 1e-10 {
        sp[0]
    } else {
        t
    }
}

/// Sorted without repeats.
fn sort_unique(line: &mut Vec<i64>) {
    line.sort_unstable();
    line.dedup();
}

/// Every leaf corner by key, and the u keys on each row and v keys on each column for the T-junction search.
fn index_leaves(q: &mut Quadtree) {
    for nd in &q.nodes {
        if !nd.leaf {
            continue;
        }

        let us = [nd.u0, nd.u1, nd.u1, nd.u0];
        let vs = [nd.v0, nd.v0, nd.v1, nd.v1];

        for ci in 0..4 {
            let key = (
                quantize(wrap(q.closed[0], &q.usp, us[ci])),
                quantize(wrap(q.closed[1], &q.vsp, vs[ci])),
            );

            q.corners.entry(key).or_insert_with(|| nd.c[ci].clone());
            q.rows.entry(key.1).or_default().push(quantize(us[ci]));
            q.cols.entry(key.0).or_default().push(quantize(vs[ci]));
        }
    }

    for line in q.rows.values_mut() {
        sort_unique(line);
    }

    for line in q.cols.values_mut() {
        sort_unique(line);
    }
}

/// Parameters on one line strictly between t0 and t1, in walk order from t0 to t1.
fn between(lines: &BTreeMap<i64, Vec<i64>>, line: i64, t0: f64, t1: f64) -> Vec<f64> {
    let mut result = Vec::new();

    let Some(found) = lines.get(&line) else {
        return result;
    };

    let lo = quantize(t0).min(quantize(t1));
    let hi = quantize(t0).max(quantize(t1));
    let first = found.partition_point(|&k| k <= lo);
    let last = found.partition_point(|&k| k < hi);

    for &k in &found[first..last] {
        result.push(k as f64 / KEY_SCALE);
    }

    if t0 > t1 {
        result.reverse();
    }

    result
}

/// T-junction parameters between u0 and u1 on the row at v.
fn row_mids(q: &Quadtree, u0: f64, u1: f64, v: f64) -> Vec<f64> {
    between(&q.rows, quantize(wrap(q.closed[1], &q.vsp, v)), u0, u1)
}

/// T-junction parameters between v0 and v1 on the column at u.
fn col_mids(q: &Quadtree, u: f64, v0: f64, v1: f64) -> Vec<f64> {
    between(&q.cols, quantize(wrap(q.closed[0], &q.usp, u)), v0, v1)
}

/// Mesh vertex at (u, v): the pole on a singular side, else one per key, sampled when no leaf corner holds it.
fn vertex_at(q: &mut Quadtree, u: f64, v: f64) -> usize {
    if let Some(south) = q.south {
        if (v - q.vsp[0]).abs() < 1e-10 {
            return south;
        }
    }

    if let Some(north) = q.north {
        if (v - q.vsp[q.vsp.len() - 1]).abs() < 1e-10 {
            return north;
        }
    }

    let uw = wrap(q.closed[0], &q.usp, u);
    let vw = wrap(q.closed[1], &q.vsp, v);
    let key = (quantize(uw), quantize(vw));

    if let Some(&vertex) = q.keys.get(&key) {
        return vertex;
    }

    let s = q.s;
    let corner = q.corners.entry(key).or_insert_with(|| sample(s, uw, vw));
    let vertex = q.mesh.add_vertex(corner.p.clone(), None);
    let vd = q.mesh.vertex.get_mut(&vertex).unwrap();

    vd.attributes.insert("u".to_string(), uw);
    vd.attributes.insert("v".to_string(), vw);
    q.keys.insert(key, vertex);

    vertex
}

/// Vertices counter-clockwise around the leaf with the T-junction vertices on each edge, repeats at poles and seams dropped.
fn leaf_polygon(q: &mut Quadtree, nd: &Node) -> Vec<usize> {
    let mut poly = vec![vertex_at(q, nd.u0, nd.v0)];

    for u in row_mids(q, nd.u0, nd.u1, nd.v0) {
        poly.push(vertex_at(q, u, nd.v0));
    }

    poly.push(vertex_at(q, nd.u1, nd.v0));

    for v in col_mids(q, nd.u1, nd.v0, nd.v1) {
        poly.push(vertex_at(q, nd.u1, v));
    }

    poly.push(vertex_at(q, nd.u1, nd.v1));

    for u in row_mids(q, nd.u1, nd.u0, nd.v1) {
        poly.push(vertex_at(q, u, nd.v1));
    }

    poly.push(vertex_at(q, nd.u0, nd.v1));

    for v in col_mids(q, nd.u0, nd.v1, nd.v0) {
        poly.push(vertex_at(q, nd.u0, v));
    }

    poly.dedup();

    while poly.len() > 1 && poly[0] == poly[poly.len() - 1] {
        poly.pop();
    }

    poly
}

/// Faces of one leaf: a triangle, a quad cut along its shorter diagonal, or a fan around the centre once T-junctions add vertices.
fn add_leaf_faces(q: &mut Quadtree, nd: &Node) {
    let poly = leaf_polygon(q, nd);
    let n = poly.len();

    if n < 3 {
        return;
    }

    if n == 3 {
        q.mesh.add_face(vec![poly[0], poly[1], poly[2]], None);

        return;
    }

    if n == 4 {
        let p0 = q.mesh.vertex[&poly[0]].position();
        let p1 = q.mesh.vertex[&poly[1]].position();
        let p2 = q.mesh.vertex[&poly[2]].position();
        let p3 = q.mesh.vertex[&poly[3]].position();

        if dist2(&p0, &p2) <= dist2(&p1, &p3) {
            q.mesh.add_face(vec![poly[0], poly[1], poly[2]], None);
            q.mesh.add_face(vec![poly[0], poly[2], poly[3]], None);
        } else {
            q.mesh.add_face(vec![poly[0], poly[1], poly[3]], None);
            q.mesh.add_face(vec![poly[1], poly[2], poly[3]], None);
        }

        return;
    }

    let cu = wrap(q.closed[0], &q.usp, (nd.u0 + nd.u1) * 0.5);
    let cv = wrap(q.closed[1], &q.vsp, (nd.v0 + nd.v1) * 0.5);
    let key = (quantize(cu), quantize(cv));

    q.corners.entry(key).or_insert_with(|| nd.c[4].clone());

    let centre = vertex_at(q, cu, cv);

    for i in 0..n {
        let j = (i + 1) % n;

        if poly[i] != poly[j] && poly[i] != centre && poly[j] != centre {
            q.mesh.add_face(vec![poly[i], poly[j], centre], None);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Normals
// ═══════════════════════════════════════════════════════════════════════════
/// Sum of the unnormalized face normals around each vertex key, faces taken in key order.
fn fan_normals(mesh: &Mesh) -> Vec<Vector> {
    let mut sums = vec![Vector::new(0.0, 0.0, 0.0); mesh.vertex.len()];
    let mut face_keys: Vec<usize> = mesh.face.keys().copied().collect();

    face_keys.sort_unstable();

    for key in face_keys {
        let vertices = &mesh.face[&key];
        let p0 = mesh.vertex[&vertices[0]].position();
        let p1 = mesh.vertex[&vertices[1]].position();
        let p2 = mesh.vertex[&vertices[2]].position();
        let n = (&p1 - &p0).cross(&(&p2 - &p0));

        for &vertex in vertices {
            sums[vertex] += &n;
        }
    }

    sums
}

/// Unit fan normal on every vertex, zero where the fan cancels.
fn set_normals(mesh: &mut Mesh) {
    let sums = fan_normals(mesh);

    for (&key, vd) in mesh.vertex.iter_mut() {
        let length = norm(&sums[key]);
        let n = if length > 1e-15 {
            &sums[key] / length
        } else {
            sums[key].clone()
        };

        vd.set_normal(n[0], n[1], n[2]);
    }
}

impl RemeshNurbsSurfaceAdaptive {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct over a surface with the default tolerances.
    pub fn new(surface: NurbsSurface) -> Self {
        RemeshNurbsSurfaceAdaptive {
            surface,
            max_angle: 20.0,
            max_edge_length: 0.0,
            min_edge_length: 0.0,
            max_chord_height: 0.0,
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the largest normal turn in degrees.
    pub fn get_max_angle(&self) -> f64 {
        self.max_angle
    }

    /// Return the longest cell edge.
    pub fn get_max_edge_length(&self) -> f64 {
        self.max_edge_length
    }

    /// Return the shortest cell edge still split.
    pub fn get_min_edge_length(&self) -> f64 {
        self.min_edge_length
    }

    /// Return the largest chord height.
    pub fn get_max_chord_height(&self) -> f64 {
        self.max_chord_height
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Largest normal turn across a cell in degrees, 20 by default.
    pub fn set_max_angle(&mut self, degrees: f64) -> &mut Self {
        self.max_angle = degrees;

        self
    }

    /// Longest cell edge; 0 for no limit.
    pub fn set_max_edge_length(&mut self, length: f64) -> &mut Self {
        self.max_edge_length = length;

        self
    }

    /// Shortest cell edge still split; 0 for no limit.
    pub fn set_min_edge_length(&mut self, length: f64) -> &mut Self {
        self.min_edge_length = length;

        self
    }

    /// Largest chord height; 0 for 0.5 percent of the bbox diagonal.
    pub fn set_max_chord_height(&mut self, height: f64) -> &mut Self {
        self.max_chord_height = height;

        self
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Meshing
    // ═══════════════════════════════════════════════════════════════════════════
    /// Triangle mesh with u, v vertex attributes and fan normals.
    pub fn mesh(&self) -> Mesh {
        let norm_tol = 2.0 - 2.0 * (self.max_angle * Tolerance::PI / 180.0).cos();
        let chord_tol = if self.max_chord_height > 0.0 {
            self.max_chord_height
        } else {
            bbox_diagonal(&self.surface) * 0.005
        };

        let mut q = Quadtree::new(
            &self.surface,
            norm_tol,
            chord_tol,
            self.max_edge_length,
            self.min_edge_length,
        );

        build(&mut q);
        index_leaves(&mut q);

        if self.surface.is_singular(0) {
            let pole = self
                .surface
                .point_at(q.usp[0], q.vsp[0])
                .unwrap_or_default();

            q.south = Some(q.mesh.add_vertex(pole, None));
        }

        if self.surface.is_singular(2) {
            let pole = self
                .surface
                .point_at(q.usp[0], q.vsp[q.vsp.len() - 1])
                .unwrap_or_default();

            q.north = Some(q.mesh.add_vertex(pole, None));
        }

        for i in 0..q.nodes.len() {
            let nd = q.nodes[i].clone();

            if nd.leaf {
                add_leaf_faces(&mut q, &nd);
            }
        }

        set_normals(&mut q.mesh);

        q.mesh
    }
}
