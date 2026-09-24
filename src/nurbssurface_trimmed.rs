#![allow(
    clippy::manual_range_contains,
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]
use crate::closest::Closest;
use crate::color::Color;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
use crate::primitives::Primitives;
use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
use crate::tolerance::PI;
use crate::vector::Vector;
use crate::xform::Xform;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::collections::HashMap;
use std::collections::HashSet;

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Normal on the side of a C0 knot line that belongs to the triangle around center.
fn crease_side_normal(
    surface: &NurbsSurface,
    knots: &[Vec<f64>; 2],
    center: [f64; 2],
    mut uv: [f64; 2],
) -> Vector {
    for dir in 0..2 {
        if knots[dir].contains(&uv[dir]) {
            if center[dir] < uv[dir] {
                uv[dir] = uv[dir].next_down();
            }

            if center[dir] > uv[dir] {
                uv[dir] = uv[dir].next_up();
            }
        }
    }

    surface.normal_at(uv[0], uv[1])
}

/// Winding-number test of (u, v) against a closed UV polygon.
fn point_in_polygon_2d(u: f64, v: f64, poly: &[Point]) -> bool {
    let mut winding = 0;
    let n = poly.len();

    for i in 0..n {
        let j = (i + 1) % n;
        let x0 = poly[i][0];
        let y0 = poly[i][1];
        let x1 = poly[j][0];
        let y1 = poly[j][1];
        let cross = (x1 - x0) * (v - y0) - (y1 - y0) * (u - x0);

        if y0 <= v && y1 > v && cross > 0.0 {
            winding += 1;
        }

        if y0 > v && y1 <= v && cross < 0.0 {
            winding -= 1;
        }
    }

    winding != 0
}

/// True when (u, v) lies inside the outer loop and outside every hole.
fn inside_loops(u: f64, v: f64, loops_uv: &[Vec<Point>]) -> bool {
    if !point_in_polygon_2d(u, v, &loops_uv[0]) {
        return false;
    }

    for li in 1..loops_uv.len() {
        if point_in_polygon_2d(u, v, &loops_uv[li]) {
            return false;
        }
    }

    true
}

/// Surface point at (u, v) as a plain array.
fn eval3(srf: &NurbsSurface, u: f64, v: f64) -> [f64; 3] {
    let p = srf.point_at(u, v).unwrap_or_default();

    [p[0], p[1], p[2]]
}

/// Signed distance of the surface point at (u, v) to the plane (q, n).
fn plane_field(srf: &NurbsSurface, q: &[f64; 3], n: &[f64; 3], u: f64, v: f64) -> f64 {
    let p = eval3(srf, u, v);

    (p[0] - q[0]) * n[0] + (p[1] - q[1]) * n[1] + (p[2] - q[2]) * n[2]
}

/// Newton steps of (u, v) onto the plane (q, n) along the field gradient.
fn refine_crossing(
    srf: &NurbsSurface,
    q: &[f64; 3],
    n: &[f64; 3],
    mut u: f64,
    mut v: f64,
) -> (f64, f64) {
    for _it in 0..12 {
        let fv = plane_field(srf, q, n, u, v);

        if fv.abs() < 1e-9 {
            break;
        }

        let h = 1e-4;
        let a = eval3(srf, u + h, v);
        let b = eval3(srf, u - h, v);
        let c = eval3(srf, u, v + h);
        let d = eval3(srf, u, v - h);
        let gu = ((a[0] - b[0]) * n[0] + (a[1] - b[1]) * n[1] + (a[2] - b[2]) * n[2]) / (2.0 * h);
        let gv = ((c[0] - d[0]) * n[0] + (c[1] - d[1]) * n[1] + (c[2] - d[2]) * n[2]) / (2.0 * h);
        let g2 = gu * gu + gv * gv;

        if g2 < 1e-20 {
            break;
        }

        u -= fv * gu / g2;
        v -= fv * gv / g2;
    }

    (u, v)
}

/// Normal turn in degrees along dir over [t0, t1] on the line smid of the other direction, summed over four steps.
fn span_turn(srf: &NurbsSurface, dir: usize, t0: f64, t1: f64, smid: f64) -> f64 {
    let mut ma = 0.0_f64;
    let mut pn = Vector::new(0.0, 0.0, 0.0);

    for k in 0..=4 {
        let t = t0 + k as f64 * (t1 - t0) / 4.0;
        let nm = if dir == 0 {
            srf.normal_at(t, smid)
        } else {
            srf.normal_at(smid, t)
        };

        if k > 0 {
            let d = pn.dot(&nm).clamp(-1.0, 1.0);
            ma += d.acos() * 180.0 / PI;
        }

        pn = nm;
    }

    ma
}

/// Largest distance of the quarter points along dir over [t0, t1] on the line smid from their chord.
fn span_deviation(srf: &NurbsSurface, dir: usize, t0: f64, t1: f64, smid: f64) -> f64 {
    let p0 = if dir == 0 {
        eval3(srf, t0, smid)
    } else {
        eval3(srf, smid, t0)
    };
    let p1 = if dir == 0 {
        eval3(srf, t1, smid)
    } else {
        eval3(srf, smid, t1)
    };
    let mut dev = 0.0_f64;

    for k in 1..=3 {
        let fr = k as f64 / 4.0;
        let tm = t0 + fr * (t1 - t0);
        let pm = if dir == 0 {
            eval3(srf, tm, smid)
        } else {
            eval3(srf, smid, tm)
        };
        let lx = p0[0] + fr * (p1[0] - p0[0]);
        let ly = p0[1] + fr * (p1[1] - p0[1]);
        let lz = p0[2] + fr * (p1[2] - p0[2]);
        let dd = ((pm[0] - lx) * (pm[0] - lx)
            + (pm[1] - ly) * (pm[1] - ly)
            + (pm[2] - lz) * (pm[2] - lz))
            .sqrt();

        if dd > dev {
            dev = dd;
        }
    }

    dev
}

/// Subdivisions per span along dir from the normal turn (max_angle_deg) and the chord deviation (chord_tol) at the mid line of the other direction.
fn span_subdivisions(
    srf: &NurbsSurface,
    dir: usize,
    sp: &[f64],
    osp: &[f64],
    deg: usize,
    max_angle_deg: f64,
    chord_tol: f64,
) -> Vec<usize> {
    let n = sp.len() - 1;
    let mut subs = vec![if deg > 1 { 2usize } else { 1 }; n];
    let smid = (osp[0] + osp[osp.len() - 1]) * 0.5;

    for i in 0..n {
        let t0 = sp[i];
        let t1 = sp[i + 1];

        if deg > 1 {
            let ma = span_turn(srf, dir, t0, t1, smid);
            subs[i] = subs[i].max(1.max(((ma / max_angle_deg).ceil() as usize).min(64)));
        }

        let dev = span_deviation(srf, dir, t0, t1, smid);

        if dev > chord_tol {
            subs[i] = subs[i].max(((dev / chord_tol).sqrt().ceil() as usize).min(64));
        }
    }

    subs
}

/// Grid parameters: each span of sp cut into subs[i] equal steps, ending on the last knot.
fn span_parameters(sp: &[f64], subs: &[usize]) -> Vec<f64> {
    let mut out = Vec::new();

    for i in 0..sp.len() - 1 {
        for st in 0..subs[i] {
            out.push(sp[i] + st as f64 * (sp[i + 1] - sp[i]) / subs[i] as f64);
        }
    }

    out.push(sp[sp.len() - 1]);

    out
}

/// Span-adaptive grid parameters in u and v; None when the surface has no span in a direction.
fn span_grid(
    srf: &NurbsSurface,
    max_angle_deg: f64,
    chord_tol: f64,
) -> Option<(Vec<f64>, Vec<f64>)> {
    let usp = srf.get_span_vector(0);
    let vsp = srf.get_span_vector(1);

    if usp.len() < 2 || vsp.len() < 2 {
        return None;
    }

    let us = span_parameters(
        &usp,
        &span_subdivisions(srf, 0, &usp, &vsp, srf.degree(0), max_angle_deg, chord_tol),
    );
    let vs = span_parameters(
        &vsp,
        &span_subdivisions(srf, 1, &vsp, &usp, srf.degree(1), max_angle_deg, chord_tol),
    );

    if us.len() < 2 || vs.len() < 2 {
        return None;
    }

    Some((us, vs))
}

/// Unit normal as a plain array, or none when degenerate.
fn unit3(n: &Vector) -> Option<[f64; 3]> {
    let nl = n.magnitude_squared().sqrt();

    if nl < 1e-12 {
        return None;
    }

    Some([n[0] / nl, n[1] / nl, n[2] / nl])
}

/// Parameters of the 2D segment crossing p1p2 x p3p4, or false when parallel or outside.
fn segment_intersection(
    p1: &[f64; 2],
    p2: &[f64; 2],
    p3: &[f64; 2],
    p4: &[f64; 2],
) -> Option<(f64, f64)> {
    let d1u = p2[0] - p1[0];
    let d1v = p2[1] - p1[1];
    let d2u = p4[0] - p3[0];
    let d2v = p4[1] - p3[1];
    let den = d1u * d2v - d1v * d2u;

    if den.abs() < 1e-20 {
        return None;
    }

    let s = ((p3[0] - p1[0]) * d2v - (p3[1] - p1[1]) * d2u) / den;
    let t = ((p3[0] - p1[0]) * d1v - (p3[1] - p1[1]) * d1u) / den;

    if s < -1e-12 || s > 1.0 + 1e-12 || t < -1e-12 || t > 1.0 + 1e-12 {
        return None;
    }

    Some((s, t))
}

/// Newton refinement of a UV curve-curve crossing (ta, tb), clamped to the domains.
fn newton_curve_curve(
    ca: &NurbsCurve,
    mut ta: f64,
    cb: &NurbsCurve,
    mut tb: f64,
    tol: f64,
) -> (f64, f64) {
    for _it in 0..8 {
        let da = ca.evaluate(ta, 1);
        let db = cb.evaluate(tb, 1);
        let fu = da[0][0] - db[0][0];
        let fv = da[0][1] - db[0][1];

        if fu.hypot(fv) < tol {
            break;
        }

        let j00 = da[1][0];
        let j01 = -db[1][0];
        let j10 = da[1][1];
        let j11 = -db[1][1];
        let den = j00 * j11 - j01 * j10;

        if den.abs() < 1e-20 {
            break;
        }

        ta -= (fu * j11 - j01 * fv) / den;
        tb -= (j00 * fv - fu * j10) / den;
        let adom = ca.domain();
        let bdom = cb.domain();
        ta = ta.max(adom.0).min(adom.1);
        tb = tb.max(bdom.0).min(bdom.1);
    }

    (ta, tb)
}

/// Signed area of a closed UV loop sampled at 64 parameters.
fn loop_signed_area(loop_crv: &NurbsCurve) -> f64 {
    let n = 64;
    let (l0, l1) = loop_crv.domain();
    let mut s = 0.0;
    let mut prev = loop_crv.point_at(l0);

    for i in 1..=n {
        let p = loop_crv.point_at(l0 + (l1 - l0) * i as f64 / n as f64);
        s += prev[0] * p[1] - p[0] * prev[1];
        prev = p;
    }

    s * 0.5
}

/// Plane coordinates of pt in the affine frame (p00, u_axis, v_axis).
fn project_to_uv(
    pt: &Point,
    p00: &Point,
    u_axis: &Vector,
    v_axis: &Vector,
    u_len2: f64,
    v_len2: f64,
) -> Point {
    let d = pt - p00;

    Point::new(d.dot(u_axis) / u_len2, d.dot(v_axis) / v_len2, 0.0)
}

// ═══════════════════════════════════════════════════════════════════════════
// VertexWelder
// ═══════════════════════════════════════════════════════════════════════════
/// Adds 3D points to a mesh, returning the existing vertex when one lies within tol.
struct VertexWelder {
    tol: f64,                                             // Weld tolerance.
    cell: f64,                                            // Hash cell size.
    cells: HashMap<(i64, i64, i64), Vec<(Point, usize)>>, // Vertices per cell.
}

impl VertexWelder {
    /// Construct with a weld tolerance and a hash cell size.
    fn new(tol: f64, cell: f64) -> Self {
        Self {
            tol,
            cell,
            cells: HashMap::new(),
        }
    }

    /// Weld a 3D point, returning the existing vertex within tol or a new one.
    fn weld(&mut self, mesh: &mut Mesh, p: Point) -> usize {
        let ci = (p[0] / self.cell).floor() as i64;
        let cj = (p[1] / self.cell).floor() as i64;
        let ck = (p[2] / self.cell).floor() as i64;

        for di in -1..=1 {
            for dj in -1..=1 {
                for dk in -1..=1 {
                    let Some(bucket) = self.cells.get(&(ci + di, cj + dj, ck + dk)) else {
                        continue;
                    };

                    for (q, vk) in bucket {
                        if (q - &p).magnitude_squared() <= self.tol * self.tol {
                            return *vk;
                        }
                    }
                }
            }
        }

        let vk = mesh.add_vertex(p.clone(), None);
        self.cells.entry((ci, cj, ck)).or_default().push((p, vk));

        vk
    }

    /// Weld the surface point at (u, v); a new vertex gets the surface normal.
    fn weld_surface(&mut self, mesh: &mut Mesh, srf: &NurbsSurface, u: f64, v: f64) -> usize {
        let before = mesh.number_of_vertices();
        let vk = self.weld(mesh, srf.point_at(u, v).unwrap_or_default());

        if mesh.number_of_vertices() > before {
            let nm = srf.normal_at(u, v);

            if let Some(vd) = mesh.vertex.get_mut(&vk) {
                vd.set_normal(nm[0], nm[1], nm[2]);
            }
        }

        vk
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Plane clipping
// ═══════════════════════════════════════════════════════════════════════════
/// Welded polygon of the part of a grid cell where the field is <= 0: kept corners and Newton-refined edge crossings in order.
fn clip_cell(
    welder: &mut VertexWelder,
    mesh: &mut Mesh,
    srf: &NurbsSurface,
    q: &[f64; 3],
    n: &[f64; 3],
    cu: &[f64; 4],
    cv: &[f64; 4],
    fc: &[f64; 4],
) -> Vec<usize> {
    let inn = [fc[0] <= 0.0, fc[1] <= 0.0, fc[2] <= 0.0, fc[3] <= 0.0];
    let mut poly: Vec<usize> = Vec::new();

    for k in 0..4 {
        let kn = (k + 1) % 4;

        if inn[k] {
            poly.push(welder.weld_surface(mesh, srf, cu[k], cv[k]));
        }

        if inn[k] != inn[kn] {
            let t = if (fc[k] - fc[kn]).abs() > 1e-30 {
                fc[k] / (fc[k] - fc[kn])
            } else {
                0.5
            };
            let u = cu[k] + (cu[kn] - cu[k]) * t;
            let v = cv[k] + (cv[kn] - cv[k]) * t;
            let (u, v) = refine_crossing(srf, q, n, u, v);
            poly.push(welder.weld_surface(mesh, srf, u, v));
        }
    }

    poly
}

/// Fan a welded polygon into the mesh from its first vertex, skipping triangles with a repeated vertex.
fn add_fan(mesh: &mut Mesh, poly: &[usize]) {
    for t in 1..poly.len().saturating_sub(1) {
        let a = poly[0];
        let b = poly[t];
        let c = poly[t + 1];

        if a == b || b == c || c == a {
            continue;
        }

        mesh.add_face(vec![a, b, c], None);
    }
}

/// Two UV triangles per cell of the grid us x vs.
fn grid_triangles(us: &[f64], vs: &[f64]) -> Vec<[[f64; 2]; 3]> {
    let mut tris: Vec<[[f64; 2]; 3]> = Vec::with_capacity((us.len() - 1) * (vs.len() - 1) * 2);

    for i in 0..us.len() - 1 {
        for j in 0..vs.len() - 1 {
            let a = [us[i], vs[j]];
            let b = [us[i + 1], vs[j]];
            let c = [us[i + 1], vs[j + 1]];
            let d = [us[i], vs[j + 1]];
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        }
    }

    tris
}

/// UV triangles clipped to the half (S-q).n <= 1e-9, each kept part fanned from its first corner.
fn clip_triangles(
    srf: &NurbsSurface,
    q: &[f64; 3],
    n: &[f64; 3],
    tris: &[[[f64; 2]; 3]],
) -> Vec<[[f64; 2]; 3]> {
    let eps = 1e-9;
    let mut next: Vec<[[f64; 2]; 3]> = Vec::new();

    for t in tris {
        let mut poly: Vec<[f64; 2]> = Vec::new();

        for e in 0..3 {
            let p = t[e];
            let r = t[(e + 1) % 3];
            let fp = plane_field(srf, q, n, p[0], p[1]);
            let fr = plane_field(srf, q, n, r[0], r[1]);
            let pin = fp <= eps;
            let rin = fr <= eps;

            if pin {
                poly.push(p);
            }

            if pin != rin {
                let tt = if (fp - fr).abs() > 1e-30 {
                    fp / (fp - fr)
                } else {
                    0.5
                };
                let cu = p[0] + (r[0] - p[0]) * tt;
                let cv = p[1] + (r[1] - p[1]) * tt;
                let (cu, cv) = refine_crossing(srf, q, n, cu, cv);
                poly.push([cu, cv]);
            }
        }

        for w in 1..poly.len().saturating_sub(1) {
            next.push([poly[0], poly[w], poly[w + 1]]);
        }
    }

    next
}

/// Mesh of UV triangles lifted onto the surface, seams welded within weld_tol, degenerate faces skipped.
fn weld_triangles(srf: &NurbsSurface, tris: &[[[f64; 2]; 3]], weld_tol: f64) -> Mesh {
    let mut result = Mesh::new();
    let mut welder = VertexWelder::new(weld_tol, weld_tol);

    for t in tris {
        let a = welder.weld_surface(&mut result, srf, t[0][0], t[0][1]);
        let b = welder.weld_surface(&mut result, srf, t[1][0], t[1][1]);
        let c = welder.weld_surface(&mut result, srf, t[2][0], t[2][1]);

        if a == b || b == c || c == a {
            continue;
        }

        result.add_face(vec![a, b, c], None);
    }

    result
}

// ═══════════════════════════════════════════════════════════════════════════
// UVGraph
// ═══════════════════════════════════════════════════════════════════════════
/// Snapped UV vertices of the split graph: points within snap of each other share one id.
struct UVVertexPool {
    snap: f64,                              // Snap distance.
    cells: HashMap<(i64, i64), Vec<usize>>, // Ids per cell.
    verts: Vec<[f64; 2]>,                   // UV position per id.
}

impl UVVertexPool {
    /// Construct with the snap distance.
    fn new(snap: f64) -> Self {
        Self {
            snap,
            cells: HashMap::new(),
            verts: Vec::new(),
        }
    }

    /// Id of the vertex within snap of p, a new one when none.
    fn id(&mut self, p: [f64; 2]) -> usize {
        let ci = (p[0] / self.snap).floor() as i64;
        let cj = (p[1] / self.snap).floor() as i64;

        for di in -1i64..=1 {
            for dj in -1i64..=1 {
                let Some(bucket) = self.cells.get(&(ci + di, cj + dj)) else {
                    continue;
                };

                for &vk in bucket {
                    let q = self.verts[vk];

                    if (q[0] - p[0]).hypot(q[1] - p[1]) <= self.snap {
                        return vk;
                    }
                }
            }
        }

        let vk = self.verts.len();
        self.verts.push([p[0], p[1]]);
        self.cells.entry((ci, cj)).or_default().push(vk);

        vk
    }
}

/// Graph edge between two pool vertices on pcurve cidx (negative: a domain border) over [ta, tb].
#[derive(Clone, Copy)]
struct SplitEdge {
    a: usize,  // First pool vertex.
    b: usize,  // Second pool vertex.
    cidx: i32, // Pcurve index, negative for a domain border.
    ta: f64,   // Parameter at a.
    tb: f64,   // Parameter at b.
}

/// Directed copy of a split edge: fwd when it runs a -> b.
struct HalfEdge {
    tail: usize, // Start vertex.
    head: usize, // End vertex.
    eidx: usize, // Split edge index.
    fwd: bool,   // True when it runs a -> b.
}

/// UV domain of the split surface and the distance under which UV points snap together.
#[derive(Clone, Copy)]
struct SplitDomain {
    u0: f64,   // Start of the u domain.
    u1: f64,   // End of the u domain.
    v0: f64,   // Start of the v domain.
    v1: f64,   // End of the v domain.
    snap: f64, // Snap distance in UV.
}

/// Sampled pcurve in UV with the parameter of each sample.
struct UVPoly {
    cidx: i32,          // Pcurve index, negative for a domain border.
    pts: Vec<[f64; 2]>, // UV samples.
    ts: Vec<f64>,       // Parameter per sample.
}

/// Consecutive half-edges of a cycle on one pcurve.
struct Run {
    cidx: i32, // Pcurve index, negative for a domain border.
    va: usize, // First vertex.
    vb: usize, // Last vertex.
    ta: f64,   // Parameter at va.
    tb: f64,   // Parameter at vb.
}

/// Domain of the surface with the snap distance: tolerance carried from 3D into UV, else 1e-7 of the shorter side.
fn split_domain(srf: &NurbsSurface, tolerance: f64) -> SplitDomain {
    let (u0, u1) = srf.domain(0).unwrap_or((0.0, 1.0));
    let (v0, v1) = srf.domain(1).unwrap_or((0.0, 1.0));
    let range_u = u1 - u0;
    let range_v = v1 - v0;

    let spans_u = srf.get_span_vector(0);
    let spans_v = srf.get_span_vector(1);
    let nu = spans_u.len().saturating_sub(1).max(1) * 4;
    let nv = spans_v.len().saturating_sub(1).max(1) * 4;
    let du = range_u / nu as f64;
    let dv = range_v / nv as f64;
    let mu = (u0 + u1) * 0.5;
    let mv = (v0 + v1) * 0.5;
    let pmid = srf.point_at(mu, mv).unwrap_or_default();
    let uv_to_3d_u = pmid.distance(
        &srf.point_at((mu + du).min(u1), mv).unwrap_or_default(),
        None,
    ) / du;
    let uv_to_3d_v = pmid.distance(
        &srf.point_at(mu, (mv + dv).min(v1)).unwrap_or_default(),
        None,
    ) / dv;
    let mut uv_to_3d = uv_to_3d_u.max(uv_to_3d_v);

    if uv_to_3d < 1e-10 {
        uv_to_3d = 1.0;
    }

    let snap = if tolerance > 0.0 {
        (tolerance / uv_to_3d).max(1e-9)
    } else {
        range_u.min(range_v) * 1e-7
    };

    SplitDomain {
        u0,
        u1,
        v0,
        v1,
        snap,
    }
}

/// Snap a UV point onto the domain border when within the snap distance of it.
fn snap_to_border(p: &mut [f64; 2], dom: &SplitDomain) {
    if (p[0] - dom.u0).abs() < dom.snap {
        p[0] = dom.u0;
    }

    if (p[0] - dom.u1).abs() < dom.snap {
        p[0] = dom.u1;
    }

    if (p[1] - dom.v0).abs() < dom.snap {
        p[1] = dom.v0;
    }

    if (p[1] - dom.v1).abs() < dom.snap {
        p[1] = dom.v1;
    }
}

/// One pass inserting the parameter midpoint of every chord farther than samp_tol from the curve; the count inserted.
fn refine_samples(crv: &NurbsCurve, entries: &mut Vec<[f64; 3]>, samp_tol: f64) -> usize {
    let mut inserted = 0;
    let mut i = 0;

    while i + 1 < entries.len() {
        let a = entries[i];
        let b = entries[i + 1];
        let tm = (a[0] + b[0]) * 0.5;
        let pm = crv.point_at(tm);
        let exu = b[1] - a[1];
        let exv = b[2] - a[2];
        let l2 = exu * exu + exv * exv;
        let mut dev = 0.0;

        if l2 > 1e-30 {
            let s = ((pm[0] - a[1]) * exu + (pm[1] - a[2]) * exv) / l2;
            let cx = a[1] + s * exu;
            let cy = a[2] + s * exv;
            dev = (pm[0] - cx).hypot(pm[1] - cy);
        }

        if dev > samp_tol && entries.len() < 4096 {
            entries.insert(i + 1, [tm, pm[0], pm[1]]);
            inserted += 1;
            i += 2;
        } else {
            i += 1;
        }
    }

    inserted
}

/// Samples (t, u, v) of a pcurve: uniform in t, then up to six passes of chord refinement.
fn sample_pcurve(crv: &NurbsCurve, samp_tol: f64) -> Vec<[f64; 3]> {
    let (ct0, ct1) = crv.domain();
    let n = (crv.cv_count() * 4).clamp(16, 2048);
    let mut entries: Vec<[f64; 3]> = Vec::new();

    for i in 0..=n {
        let t = ct0 + (ct1 - ct0) * i as f64 / n as f64;
        let p = crv.point_at(t);
        entries.push([t, p[0], p[1]]);
    }

    for _depth in 0..6 {
        if refine_samples(crv, &mut entries, samp_tol) == 0 {
            break;
        }
    }

    entries
}

/// Polyline of pcurve cidx from its samples: clamped into the domain, snapped to the border, repeats dropped.
fn clamp_samples(entries: &[[f64; 3]], cidx: i32, dom: &SplitDomain) -> UVPoly {
    let mut poly = UVPoly {
        cidx,
        pts: Vec::new(),
        ts: Vec::new(),
    };

    for e in entries {
        let mut p = [e[1].max(dom.u0).min(dom.u1), e[2].max(dom.v0).min(dom.v1)];
        snap_to_border(&mut p, dom);

        if let Some(last) = poly.pts.last() {
            if (p[0] - last[0]).abs() < 1e-15 && (p[1] - last[1]).abs() < 1e-15 {
                continue;
            }
        }

        poly.pts.push(p);
        poly.ts.push(e[0]);
    }

    poly
}

/// True when every point lies within the snap distance of one domain side.
fn on_border(pts: &[[f64; 2]], dom: &SplitDomain) -> bool {
    let mut on_u0 = true;
    let mut on_u1 = true;
    let mut on_v0 = true;
    let mut on_v1 = true;

    for p in pts {
        if (p[0] - dom.u0).abs() >= dom.snap {
            on_u0 = false;
        }

        if (p[0] - dom.u1).abs() >= dom.snap {
            on_u1 = false;
        }

        if (p[1] - dom.v0).abs() >= dom.snap {
            on_v0 = false;
        }

        if (p[1] - dom.v1).abs() >= dom.snap {
            on_v1 = false;
        }
    }

    on_u0 || on_u1 || on_v0 || on_v1
}

/// Length of a UV polyline.
fn polyline_length(pts: &[[f64; 2]]) -> f64 {
    let mut ext = 0.0;

    for k in 1..pts.len() {
        ext += (pts[k][0] - pts[k - 1][0]).hypot(pts[k][1] - pts[k - 1][1]);
    }

    ext
}

/// Polylines of the valid pcurves that neither hug the border nor fall short of min_ext, then the four domain sides.
fn uv_polylines(pcurves: &[NurbsCurve], dom: &SplitDomain) -> Vec<UVPoly> {
    let range_u = dom.u1 - dom.u0;
    let range_v = dom.v1 - dom.v0;
    let samp_tol = range_u.max(range_v) * 2e-5;
    let min_ext = (dom.snap * 8.0).max(range_u.min(range_v) * 1e-5);
    let mut polylines: Vec<UVPoly> = Vec::new();

    for (cidx, crv) in pcurves.iter().enumerate() {
        if !crv.is_valid() {
            continue;
        }

        let poly = clamp_samples(&sample_pcurve(crv, samp_tol), cidx as i32, dom);

        if poly.pts.len() >= 2
            && !on_border(&poly.pts, dom)
            && polyline_length(&poly.pts) >= min_ext
        {
            polylines.push(poly);
        }
    }

    polylines.push(UVPoly {
        cidx: -1,
        pts: vec![[dom.u0, dom.v0], [dom.u1, dom.v0]],
        ts: vec![dom.u0, dom.u1],
    });
    polylines.push(UVPoly {
        cidx: -2,
        pts: vec![[dom.u1, dom.v0], [dom.u1, dom.v1]],
        ts: vec![dom.v0, dom.v1],
    });
    polylines.push(UVPoly {
        cidx: -3,
        pts: vec![[dom.u1, dom.v1], [dom.u0, dom.v1]],
        ts: vec![dom.u1, dom.u0],
    });
    polylines.push(UVPoly {
        cidx: -4,
        pts: vec![[dom.u0, dom.v1], [dom.u0, dom.v0]],
        ts: vec![dom.v1, dom.v0],
    });

    polylines
}

/// UV bounds (umin, umax, vmin, vmax) of a polyline.
fn uv_bounds(pts: &[[f64; 2]]) -> [f64; 4] {
    let mut bounds = [pts[0][0], pts[0][0], pts[0][1], pts[0][1]];

    for p in pts {
        bounds[0] = bounds[0].min(p[0]);
        bounds[1] = bounds[1].max(p[0]);
        bounds[2] = bounds[2].min(p[1]);
        bounds[3] = bounds[3].max(p[1]);
    }

    bounds
}

/// True when the bounds of B meet the bounds of A grown by snap.
fn boxes_overlap(a_poly: &UVPoly, b_poly: &UVPoly, snap: f64) -> bool {
    let a = uv_bounds(&a_poly.pts);
    let b = uv_bounds(&b_poly.pts);

    !(b[0] > a[1] + snap || b[1] < a[0] - snap || b[2] > a[3] + snap || b[3] < a[2] - snap)
}

/// Parameter of a point along domain side cidx: u on the bottom and top sides, v on the left and right.
fn border_parameter(cidx: i32, hp: &[f64; 2]) -> f64 {
    if cidx == -1 || cidx == -3 {
        hp[0]
    } else {
        hp[1]
    }
}

/// UV point of a crossing moved onto its pcurves, Newton-refined when both are pcurves, snapped to the border; ta and tb follow it.
fn crossing_point(
    acidx: i32,
    mut ta: f64,
    bcidx: i32,
    mut tb: f64,
    hit: [f64; 2],
    pcurves: &[NurbsCurve],
    dom: &SplitDomain,
) -> ([f64; 2], f64, f64) {
    let mut hp = hit;

    if acidx >= 0 && bcidx >= 0 {
        (ta, tb) = newton_curve_curve(
            &pcurves[acidx as usize],
            ta,
            &pcurves[bcidx as usize],
            tb,
            dom.snap * 0.01,
        );
    }

    if acidx >= 0 {
        let pa = pcurves[acidx as usize].point_at(ta);
        hp = [pa[0], pa[1]];
    } else if bcidx >= 0 {
        let pb = pcurves[bcidx as usize].point_at(tb);
        hp = [pb[0], pb[1]];
    }

    snap_to_border(&mut hp, dom);

    if bcidx < 0 {
        tb = border_parameter(bcidx, &hp);
    }

    if acidx < 0 {
        ta = border_parameter(acidx, &hp);
    }

    (hp, ta, tb)
}

/// Crossings of polylines pi and pj as events (fraction, u, v, parameter) on each crossed segment.
fn add_crossings(
    polylines: &[UVPoly],
    pi: usize,
    pj: usize,
    pcurves: &[NurbsCurve],
    dom: &SplitDomain,
    splits: &mut HashMap<(usize, usize), Vec<(f64, f64, f64, f64)>>,
) {
    let a_poly = &polylines[pi];
    let b_poly = &polylines[pj];

    for ia in 0..a_poly.pts.len() - 1 {
        for ib in 0..b_poly.pts.len() - 1 {
            let Some((s, t)) = segment_intersection(
                &a_poly.pts[ia],
                &a_poly.pts[ia + 1],
                &b_poly.pts[ib],
                &b_poly.pts[ib + 1],
            ) else {
                continue;
            };
            let ta = a_poly.ts[ia] + (a_poly.ts[ia + 1] - a_poly.ts[ia]) * s;
            let tb = b_poly.ts[ib] + (b_poly.ts[ib + 1] - b_poly.ts[ib]) * t;
            let hit = [
                a_poly.pts[ia][0] + (a_poly.pts[ia + 1][0] - a_poly.pts[ia][0]) * s,
                a_poly.pts[ia][1] + (a_poly.pts[ia + 1][1] - a_poly.pts[ia][1]) * s,
            ];
            let (hp, ta, tb) = crossing_point(a_poly.cidx, ta, b_poly.cidx, tb, hit, pcurves, dom);
            splits
                .entry((pi, ia))
                .or_default()
                .push((s, hp[0], hp[1], ta));
            splits
                .entry((pj, ib))
                .or_default()
                .push((t, hp[0], hp[1], tb));
        }
    }
}

/// Crossing events of every pair of overlapping polylines with at least one pcurve, keyed by (polyline, segment).
fn polyline_crossings(
    polylines: &[UVPoly],
    pcurves: &[NurbsCurve],
    dom: &SplitDomain,
) -> HashMap<(usize, usize), Vec<(f64, f64, f64, f64)>> {
    let mut splits: HashMap<(usize, usize), Vec<(f64, f64, f64, f64)>> = HashMap::new();

    for pi in 0..polylines.len() {
        for pj in (pi + 1)..polylines.len() {
            if (polylines[pi].cidx >= 0 || polylines[pj].cidx >= 0)
                && boxes_overlap(&polylines[pi], &polylines[pj], dom.snap)
            {
                add_crossings(polylines, pi, pj, pcurves, dom, &mut splits);
            }
        }
    }

    splits
}

/// Graph edges along every polyline between consecutive pool vertices, its crossings inserted in order.
fn split_edges(
    polylines: &[UVPoly],
    splits: &HashMap<(usize, usize), Vec<(f64, f64, f64, f64)>>,
    pool: &mut UVVertexPool,
) -> Vec<SplitEdge> {
    let mut edges: Vec<SplitEdge> = Vec::new();

    for (pi, poly) in polylines.iter().enumerate() {
        let mut chain: Vec<(usize, f64)> = Vec::new();

        for i in 0..poly.pts.len() {
            chain.push((pool.id(poly.pts[i]), poly.ts[i]));

            if i + 1 < poly.pts.len() {
                if let Some(sp) = splits.get(&(pi, i)) {
                    let mut evs = sp.clone();
                    evs.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));

                    for ev in evs {
                        chain.push((pool.id([ev.1, ev.2]), ev.3));
                    }
                }
            }
        }

        for i in 0..chain.len().saturating_sub(1) {
            let (a, ta) = chain[i];
            let (b, tb) = chain[i + 1];

            if a == b {
                continue;
            }

            edges.push(SplitEdge {
                a,
                b,
                cidx: poly.cidx,
                ta,
                tb,
            });
        }
    }

    edges
}

/// Edges left after repeatedly dropping every edge with an end of degree one.
fn prune_dangling(edges: &[SplitEdge]) -> Vec<SplitEdge> {
    let mut alive = vec![true; edges.len()];
    let mut changed = true;

    for _ in 0..=edges.len() {
        if !changed {
            break;
        }

        changed = false;
        let mut degree: HashMap<usize, usize> = HashMap::new();

        for (ei, e) in edges.iter().enumerate() {
            if !alive[ei] {
                continue;
            }

            *degree.entry(e.a).or_insert(0) += 1;
            *degree.entry(e.b).or_insert(0) += 1;
        }

        for (ei, e) in edges.iter().enumerate() {
            if !alive[ei] {
                continue;
            }

            if degree.get(&e.a).copied().unwrap_or(0) == 1
                || degree.get(&e.b).copied().unwrap_or(0) == 1
            {
                alive[ei] = false;
                changed = true;
            }
        }
    }

    let mut live_edges: Vec<SplitEdge> = Vec::new();

    for (ei, e) in edges.iter().enumerate() {
        if alive[ei] {
            live_edges.push(*e);
        }
    }

    live_edges
}

/// Two opposite half-edges per edge, the forward one at the even index.
fn half_edges(edges: &[SplitEdge]) -> Vec<HalfEdge> {
    let mut hes: Vec<HalfEdge> = Vec::new();

    for (ei, e) in edges.iter().enumerate() {
        hes.push(HalfEdge {
            tail: e.a,
            head: e.b,
            eidx: ei,
            fwd: true,
        });
        hes.push(HalfEdge {
            tail: e.b,
            head: e.a,
            eidx: ei,
            fwd: false,
        });
    }

    hes
}

/// Successor of every half-edge around its face: the twin of an outgoing half-edge continues with its predecessor in the angle-sorted fan.
fn next_half_edges(hes: &[HalfEdge], verts: &[[f64; 2]]) -> Vec<usize> {
    let mut out_map: Vec<Vec<usize>> = vec![Vec::new(); verts.len()];

    for (hi, he) in hes.iter().enumerate() {
        out_map[he.tail].push(hi);
    }

    for vid in 0..out_map.len() {
        let mut fan: Vec<(f64, usize)> = Vec::new();

        for &hi in &out_map[vid] {
            let angle = (verts[hes[hi].head][1] - verts[vid][1])
                .atan2(verts[hes[hi].head][0] - verts[vid][0]);
            fan.push((angle, hi));
        }

        fan.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));

        for k in 0..fan.len() {
            out_map[vid][k] = fan[k].1;
        }
    }

    let mut next_he = vec![usize::MAX; hes.len()];

    for outs in &out_map {
        for pos in 0..outs.len() {
            next_he[outs[pos] ^ 1] = outs[(pos + outs.len() - 1) % outs.len()];
        }
    }

    next_he
}

/// Cycles of at least two half-edges traced through next_he, each half-edge in one cycle.
fn face_cycles(next_he: &[usize]) -> Vec<Vec<usize>> {
    let mut visited = vec![false; next_he.len()];
    let mut faces: Vec<Vec<usize>> = Vec::new();

    for hi in 0..next_he.len() {
        if visited[hi] {
            continue;
        }

        let mut cycle = Vec::new();
        let mut cur = hi;

        while cur != usize::MAX && !visited[cur] {
            visited[cur] = true;
            cycle.push(cur);
            cur = next_he[cur];
        }

        if cycle.len() >= 2 {
            faces.push(cycle);
        }
    }

    faces
}

/// Signed area of a half-edge cycle.
fn cycle_area(cycle: &[usize], hes: &[HalfEdge], verts: &[[f64; 2]]) -> f64 {
    let mut s = 0.0;

    for &hi in cycle {
        let a = verts[hes[hi].tail];
        let b = verts[hes[hi].head];
        s += a[0] * b[1] - b[0] * a[1];
    }

    s * 0.5
}

/// True when a cycle passes through a vertex of the domain border.
fn touches_border(cycle: &[usize], hes: &[HalfEdge], border_vids: &HashSet<usize>) -> bool {
    for &hi in cycle {
        if border_vids.contains(&hes[hi].tail) {
            return true;
        }
    }

    false
}

/// Face cycles by orientation: counter-clockwise faces with their area, clockwise holes clear of the domain border.
fn classify_faces(
    faces: Vec<Vec<usize>>,
    hes: &[HalfEdge],
    verts: &[[f64; 2]],
    edges: &[SplitEdge],
    snap: f64,
) -> (Vec<(Vec<usize>, f64)>, Vec<Vec<usize>>) {
    let mut border_vids: HashSet<usize> = HashSet::new();

    for e in edges {
        if e.cidx < 0 {
            border_vids.insert(e.a);
            border_vids.insert(e.b);
        }
    }

    let mut pos_faces: Vec<(Vec<usize>, f64)> = Vec::new();
    let mut neg_faces: Vec<Vec<usize>> = Vec::new();

    for cycle in faces {
        let area = cycle_area(&cycle, hes, verts);

        if area > snap * snap {
            pos_faces.push((cycle, area));
        } else if area < -snap * snap && !touches_border(&cycle, hes, &border_vids) {
            neg_faces.push(cycle);
        }
    }

    (pos_faces, neg_faces)
}

/// Even-odd test of p against a half-edge cycle.
fn point_in_cycle(p: [f64; 2], cycle: &[usize], hes: &[HalfEdge], verts: &[[f64; 2]]) -> bool {
    let mut inside = false;

    for &hi in cycle {
        let a = verts[hes[hi].tail];
        let b = verts[hes[hi].head];

        if (a[1] > p[1]) != (b[1] > p[1])
            && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            inside = !inside;
        }
    }

    inside
}

/// True when two cycles pass through the same set of vertices.
fn same_vertices(a: &[usize], b: &[usize], hes: &[HalfEdge]) -> bool {
    let mut a_vids: HashSet<usize> = HashSet::new();
    let mut b_vids: HashSet<usize> = HashSet::new();

    for &hi in a {
        a_vids.insert(hes[hi].tail);
    }

    for &hi in b {
        b_vids.insert(hes[hi].tail);
    }

    a_vids == b_vids
}

/// Holes per positive face: each hole goes to the smallest face that contains it and is not its own vertex ring.
fn assign_holes(
    neg_faces: &[Vec<usize>],
    pos_faces: &[(Vec<usize>, f64)],
    hes: &[HalfEdge],
    verts: &[[f64; 2]],
) -> Vec<Vec<Vec<usize>>> {
    let mut holes_of: Vec<Vec<Vec<usize>>> = vec![Vec::new(); pos_faces.len()];

    for cycle in neg_faces {
        let sample = verts[hes[cycle[0]].tail];
        let mut best: i32 = -1;
        let mut best_area = f64::INFINITY;

        for (fi, (fc, area)) in pos_faces.iter().enumerate() {
            if *area < best_area
                && point_in_cycle(sample, fc, hes, verts)
                && !same_vertices(cycle, fc, hes)
            {
                best = fi as i32;
                best_area = *area;
            }
        }

        if best >= 0 {
            holes_of[best as usize].push(cycle.clone());
        }
    }

    holes_of
}

/// Runs of a cycle: consecutive half-edges on one pcurve merged.
fn cycle_runs(cycle: &[usize], hes: &[HalfEdge], edges: &[SplitEdge]) -> Vec<Run> {
    let mut runs: Vec<Run> = Vec::new();

    for &hi in cycle {
        let he = &hes[hi];
        let e = edges[he.eidx];
        let ta = if he.fwd { e.ta } else { e.tb };
        let tb = if he.fwd { e.tb } else { e.ta };

        if let Some(last) = runs.last_mut() {
            if last.cidx == e.cidx && last.vb == he.tail {
                last.vb = he.head;
                last.tb = tb;
                continue;
            }
        }

        runs.push(Run {
            cidx: e.cidx,
            va: he.tail,
            vb: he.head,
            ta,
            tb,
        });
    }

    runs
}

/// Pcurve piece of a run, trimmed to its parameters and oriented along it; None when the run cannot be cut.
fn run_piece(run: &Run, pcurves: &[NurbsCurve]) -> Option<NurbsCurve> {
    if run.cidx < 0 {
        return None;
    }

    let crv = &pcurves[run.cidx as usize];
    let (c0, c1) = crv.domain();
    let lo = c0.max(run.ta.min(run.tb));
    let hi_ = c1.min(run.ta.max(run.tb));
    let mut piece = crv.duplicate();

    if hi_ - lo < (c1 - c0) - 1e-12 && hi_ - lo > 1e-14 {
        if !piece.trim(lo, hi_) {
            return None;
        }
    } else if hi_ - lo <= 1e-14 && !(run.va == run.vb && piece.is_closed()) {
        return None;
    }

    if !piece.is_valid() {
        return None;
    }

    if run.ta > run.tb && !piece.reverse() {
        return None;
    }

    Some(piece)
}

/// Pieces of a cycle: trimmed pcurve runs, straight UV segments where a run cannot be cut.
fn cycle_to_segments(
    cycle: &[usize],
    hes: &[HalfEdge],
    edges: &[SplitEdge],
    verts: &[[f64; 2]],
    pcurves: &[NurbsCurve],
) -> Vec<NurbsCurve> {
    let mut pieces: Vec<NurbsCurve> = Vec::new();

    for run in &cycle_runs(cycle, hes, edges) {
        if let Some(piece) = run_piece(run, pcurves) {
            pieces.push(piece);
            continue;
        }

        let pa = verts[run.va];
        let pb = verts[run.vb];

        if (pb[0] - pa[0]).hypot(pb[1] - pa[1]) > 1e-14 {
            let seg_pts = vec![Point::new(pa[0], pa[1], 0.0), Point::new(pb[0], pb[1], 0.0)];
            pieces.push(NurbsCurve::create(false, 1, &seg_pts));
        }
    }

    pieces
}

/// Close a curve whose ends lie within tol by moving its last control point onto the first; true when it ends closed.
fn close_curve(curve: &mut NurbsCurve, tol: f64) -> bool {
    if curve.is_closed() {
        return true;
    }

    if curve.point_at_start().distance(&curve.point_at_end(), None) > tol {
        return false;
    }

    let last = curve.cv_count() - 1;
    let (Some((x, y, z, _w)), Some((_xe, _ye, _ze, we))) =
        (curve.get_cv_4d(0), curve.get_cv_4d(last))
    else {
        return false;
    };

    curve.set_cv_4d(last, x, y, z, we) && curve.is_closed()
}

/// Closed loop of a cycle: the joined pieces when they close, else the polygon through its vertices.
fn cycle_to_loop(
    cycle: &[usize],
    hes: &[HalfEdge],
    edges: &[SplitEdge],
    verts: &[[f64; 2]],
    pcurves: &[NurbsCurve],
    snap_uv: f64,
) -> NurbsCurve {
    let pieces = cycle_to_segments(cycle, hes, edges, verts, pcurves);

    if pieces.is_empty() {
        return NurbsCurve::default();
    }

    let join_tol = snap_uv * 4.0;
    let mut joined = NurbsCurve::join(&pieces, Some(join_tol));

    if joined.len() == 1 && joined[0].is_valid() && close_curve(&mut joined[0], join_tol) {
        return joined.remove(0);
    }

    let mut loop_pts: Vec<Point> = Vec::new();

    for &hi in cycle {
        let a = verts[hes[hi].tail];
        loop_pts.push(Point::new(a[0], a[1], 0.0));
    }

    loop_pts.push(Point::new(loop_pts[0][0], loop_pts[0][1], 0.0));

    NurbsCurve::create(false, 1, &loop_pts)
}

// ═══════════════════════════════════════════════════════════════════════════
// Delaunay2D
// ═══════════════════════════════════════════════════════════════════════════
/// UV vertex of the triangulation.
struct Vertex2D {
    x: f64, // U coordinate.
    y: f64, // V coordinate.
}

/// Triangle with per-edge neighbours; edge k is opposite vertex k.
struct Triangle {
    v: [i32; 3],            // Vertex indices.
    adj: [i32; 3],          // Neighbour across each edge, -1 on the hull.
    constrained: [bool; 3], // True where an edge is a constraint.
    alive: bool,            // False once removed.
}

/// Incremental constrained Delaunay triangulation in UV with Bowyer-Watson insertion.
struct Delaunay2D {
    vertices: Vec<Vertex2D>,  // Vertices, the super triangle first.
    triangles: Vec<Triangle>, // Triangle pool, dead ones flagged.
    super_v: [i32; 3],        // Super triangle vertices.
    edge_map: HashMap<(i32, i32), (i32, i32)>, // Hull edge -> (triangle, edge index).
    last_found: i32,          // Triangle the last locate ended in.
    visit_epoch: i32,         // Stamp of the current search.
    visit_stamp: Vec<i32>,    // Last search stamp per triangle.
}

impl Delaunay2D {
    /// Order-independent key of an edge.
    fn edge_key(a: i32, b: i32) -> (i32, i32) {
        (a.min(b), a.max(b))
    }

    /// Positive when d lies inside the circumcircle of a, b, c.
    fn in_circumcircle(
        ax: f64,
        ay: f64,
        bx: f64,
        by: f64,
        cx: f64,
        cy: f64,
        dx: f64,
        dy: f64,
    ) -> f64 {
        let adx = ax - dx;
        let ady = ay - dy;
        let bdx = bx - dx;
        let bdy = by - dy;
        let cdx = cx - dx;
        let cdy = cy - dy;

        (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy)
            + (bdx * bdx + bdy * bdy) * (cdx * ady - adx * cdy)
            + (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady)
    }

    /// Twice the signed area of a, b, c.
    fn orient2d(ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> f64 {
        (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    }

    /// Construct with a super triangle around the box.
    fn new(xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Self {
        let dx = xmax - xmin;
        let dy = ymax - ymin;
        let d = dx.max(dy);
        let cx = (xmin + xmax) * 0.5;
        let cy = (ymin + ymax) * 0.5;
        let scale = 20.0;
        let mut dt = Delaunay2D {
            vertices: Vec::new(),
            triangles: Vec::new(),
            super_v: [-1; 3],
            edge_map: HashMap::new(),
            last_found: 0,
            visit_epoch: 0,
            visit_stamp: Vec::new(),
        };
        dt.vertices.push(Vertex2D {
            x: cx - scale * d,
            y: cy - scale * d,
        });
        dt.vertices.push(Vertex2D {
            x: cx + scale * d,
            y: cy - scale * d,
        });
        dt.vertices.push(Vertex2D {
            x: cx,
            y: cy + scale * d,
        });
        dt.super_v = [0, 1, 2];
        dt.triangles.push(Triangle {
            v: [0, 1, 2],
            adj: [-1, -1, -1],
            constrained: [false; 3],
            alive: true,
        });
        dt.register_edges(0);

        dt
    }

    /// Record the hull edges of triangle ti in the edge map.
    fn register_edges(&mut self, ti: i32) {
        for k in 0..3 {
            let a = self.triangles[ti as usize].v[(k + 1) % 3];
            let b = self.triangles[ti as usize].v[(k + 2) % 3];
            let key = Self::edge_key(a, b);

            if let Some(&(oti, ok)) = self.edge_map.get(&key) {
                self.triangles[ti as usize].adj[k] = oti;
                self.triangles[oti as usize].adj[ok as usize] = ti;
                self.edge_map.remove(&key);
            } else {
                self.edge_map.insert(key, (ti, k as i32));
            }
        }
    }

    /// Drop the hull edges of triangle ti from the edge map and its neighbours.
    fn unregister_edges(&mut self, ti: i32) {
        for k in 0..3 {
            let a = self.triangles[ti as usize].v[(k + 1) % 3];
            let b = self.triangles[ti as usize].v[(k + 2) % 3];
            let key = Self::edge_key(a, b);
            let adj_ti = self.triangles[ti as usize].adj[k];

            if adj_ti >= 0
                && adj_ti < self.triangles.len() as i32
                && self.triangles[adj_ti as usize].alive
            {
                let mut adj_kk = -1i32;

                for kk in 0..3 {
                    if self.triangles[adj_ti as usize].adj[kk] == ti {
                        adj_kk = kk as i32;
                        break;
                    }
                }

                if adj_kk >= 0 {
                    self.triangles[adj_ti as usize].adj[adj_kk as usize] = -1;
                    let adj_a = self.triangles[adj_ti as usize].v[(adj_kk as usize + 1) % 3];
                    let adj_b = self.triangles[adj_ti as usize].v[(adj_kk as usize + 2) % 3];
                    self.edge_map
                        .insert(Self::edge_key(adj_a, adj_b), (adj_ti, adj_kk));
                }
            } else if let Some(&(eti, _)) = self.edge_map.get(&key) {
                if eti == ti {
                    self.edge_map.remove(&key);
                }
            }
        }
    }

    /// Triangle containing (x, y) by walking from start_tri, -1 when none.
    fn locate(&self, x: f64, y: f64, mut start_tri: i32) -> i32 {
        if start_tri < 0
            || start_tri >= self.triangles.len() as i32
            || !self.triangles[start_tri as usize].alive
        {
            start_tri = self.triangles.len() as i32 - 1;

            while start_tri >= 0 && !self.triangles[start_tri as usize].alive {
                start_tri -= 1;
            }

            if start_tri < 0 {
                return -1;
            }
        }

        let mut cur = start_tri;
        let max_iter = self.triangles.len() as i32;

        for _ in 0..max_iter {
            let t = &self.triangles[cur as usize];
            let mut moved = false;

            for k in 0..3 {
                let a = t.v[k];
                let b = t.v[(k + 1) % 3];
                let ax = self.vertices[a as usize].x;
                let ay = self.vertices[a as usize].y;
                let bx = self.vertices[b as usize].x;
                let by = self.vertices[b as usize].y;

                if Self::orient2d(ax, ay, bx, by, x, y) < 0.0 {
                    let opp = t.adj[(k + 2) % 3];

                    if opp >= 0
                        && (opp as usize) < self.triangles.len()
                        && self.triangles[opp as usize].alive
                    {
                        cur = opp;
                        moved = true;
                        break;
                    }
                }
            }

            if !moved {
                return cur;
            }
        }

        cur
    }

    /// Insert a point and return its vertex index, the existing one when coincident.
    fn insert(&mut self, x: f64, y: f64) -> i32 {
        let start = self.locate(x, y, self.last_found);
        let existing = self.find_coincident(start, x, y);

        if existing >= 0 {
            return existing;
        }

        let vi = self.vertices.len() as i32;
        self.vertices.push(Vertex2D { x, y });
        let bad = self.collect_cavity(start, x, y);

        if bad.is_empty() {
            self.vertices.pop();

            return -1;
        }

        let polygon = self.cavity_polygon(&bad);
        self.fill_cavity(vi, &bad, &polygon);
        self.last_found = self.triangles.len() as i32 - 1;

        vi
    }

    /// Corner of triangle ti holding vertex v, -1 when none does.
    fn vertex_index(&self, ti: i32, v: i32) -> i32 {
        for k in 0..3 {
            if self.triangles[ti as usize].v[k] == v {
                return k as i32;
            }
        }

        -1
    }

    /// Vertex of triangle ti across its edge shared with triangle nb, -1 when they are not neighbours.
    fn opposite_vertex(&self, ti: i32, nb: i32) -> i32 {
        for k in 0..3 {
            if self.triangles[ti as usize].adj[k] == nb {
                return self.triangles[ti as usize].v[k];
            }
        }

        -1
    }

    /// Vertex of triangle start within 1e-6 of (x, y), -1 when none.
    fn find_coincident(&self, start: i32, x: f64, y: f64) -> i32 {
        if start < 0 || !self.triangles[start as usize].alive {
            return -1;
        }

        for &vi in &self.triangles[start as usize].v {
            let ddx = self.vertices[vi as usize].x - x;
            let ddy = self.vertices[vi as usize].y - y;

            if ddx * ddx + ddy * ddy < 1e-12 {
                return vi;
            }
        }

        -1
    }

    /// True when (x, y) lies inside the circumcircle of triangle ti.
    fn circumcircle_contains(&self, ti: i32, x: f64, y: f64) -> bool {
        let [v0, v1, v2] = self.triangles[ti as usize].v;
        let ax = self.vertices[v0 as usize].x;
        let ay = self.vertices[v0 as usize].y;
        let bx = self.vertices[v1 as usize].x;
        let by = self.vertices[v1 as usize].y;
        let cx = self.vertices[v2 as usize].x;
        let cy = self.vertices[v2 as usize].y;
        let o = Self::orient2d(ax, ay, bx, by, cx, cy);
        let ic = if o > 0.0 {
            Self::in_circumcircle(ax, ay, bx, by, cx, cy, x, y)
        } else {
            Self::in_circumcircle(ax, ay, cx, cy, bx, by, x, y)
        };

        ic > 0.0
    }

    /// Triangles whose circumcircle holds (x, y), grown from start across unconstrained edges.
    fn collect_cavity(&mut self, start: i32, x: f64, y: f64) -> Vec<i32> {
        self.visit_epoch += 1;

        if self.visit_stamp.len() < self.triangles.len() + 64 {
            self.visit_stamp.resize(self.triangles.len() + 64, 0);
        }

        let mut bad: Vec<i32> = Vec::new();

        if start >= 0 {
            bad.push(start);
            self.visit_stamp[start as usize] = self.visit_epoch;
        }

        let mut front = 0;

        while front < bad.len() {
            let ti = bad[front];
            front += 1;

            if !self.triangles[ti as usize].alive || !self.circumcircle_contains(ti, x, y) {
                bad[front - 1] = -1;
                continue;
            }

            for k in 0..3 {
                let nb = self.triangles[ti as usize].adj[k];

                if self.triangles[ti as usize].constrained[k]
                    || nb < 0
                    || self.visit_stamp[nb as usize] == self.visit_epoch
                {
                    continue;
                }

                self.visit_stamp[nb as usize] = self.visit_epoch;
                bad.push(nb);
            }
        }

        bad.retain(|&ti| ti >= 0);

        bad
    }

    /// Edges of the bad triangles that face a good neighbour or the hull.
    fn cavity_polygon(&self, bad: &[i32]) -> Vec<(i32, i32, bool)> {
        let mut bad_set: HashSet<i32> = HashSet::new();

        for &ti in bad {
            bad_set.insert(ti);
        }

        let mut polygon: Vec<(i32, i32, bool)> = Vec::new();

        for &ti in bad {
            let t = &self.triangles[ti as usize];

            for k in 0..3 {
                let nb = t.adj[k];

                if nb < 0 || !bad_set.contains(&nb) {
                    polygon.push((t.v[(k + 1) % 3], t.v[(k + 2) % 3], t.constrained[k]));
                }
            }
        }

        polygon
    }

    /// Replace the bad triangles by a fan from vertex vi to the polygon edges.
    fn fill_cavity(&mut self, vi: i32, bad: &[i32], polygon: &[(i32, i32, bool)]) {
        for &ti in bad {
            self.unregister_edges(ti);
            self.triangles[ti as usize].alive = false;
        }

        for &(e0, e1, constr) in polygon {
            let o = Self::orient2d(
                self.vertices[vi as usize].x,
                self.vertices[vi as usize].y,
                self.vertices[e0 as usize].x,
                self.vertices[e0 as usize].y,
                self.vertices[e1 as usize].x,
                self.vertices[e1 as usize].y,
            );

            if o.abs() < 1e-20 {
                continue;
            }

            let new_ti = self.triangles.len() as i32;
            let (va, vb) = if o > 0.0 { (e0, e1) } else { (e1, e0) };
            self.triangles.push(Triangle {
                v: [vi, va, vb],
                adj: [-1, -1, -1],
                constrained: [constr, false, false],
                alive: true,
            });
            self.register_edges(new_ti);
        }
    }

    /// Force the edge v0-v1 into the triangulation by flipping the edges it crosses.
    fn insert_constraint(&mut self, v0: i32, v1: i32) {
        if v0 == v1 || self.constrain_existing(v0, v1) {
            return;
        }

        let start_ti = self.first_triangle_at(v0);

        if start_ti < 0 {
            return;
        }

        let Some((it, ivl, ivr)) = self.first_crossed(start_ti, v0, v1) else {
            return;
        };
        let mut poly_l: Vec<i32> = vec![v0, ivl];
        let mut poly_r: Vec<i32> = vec![v0, ivr];
        let mut intersected: Vec<i32> = vec![it];
        self.walk_crossed(v0, v1, ivl, ivr, &mut poly_l, &mut poly_r, &mut intersected);
        poly_l.push(v1);
        poly_r.push(v1);
        self.retriangulate(v0, v1, &poly_l, &poly_r, &intersected);
    }

    /// Mark v0-v1 constrained when it already is a triangle edge; false when it is not.
    fn constrain_existing(&mut self, v0: i32, v1: i32) -> bool {
        for ti in 0..self.triangles.len() {
            if !self.triangles[ti].alive {
                continue;
            }

            for k in 0..3 {
                let e0 = self.triangles[ti].v[(k + 1) % 3];
                let e1 = self.triangles[ti].v[(k + 2) % 3];

                if !((e0 == v0 && e1 == v1) || (e0 == v1 && e1 == v0)) {
                    continue;
                }

                self.triangles[ti].constrained[k] = true;
                let nb = self.triangles[ti].adj[k];

                if nb >= 0
                    && (nb as usize) < self.triangles.len()
                    && self.triangles[nb as usize].alive
                {
                    for kk in 0..3 {
                        if self.triangles[nb as usize].adj[kk] == ti as i32 {
                            self.triangles[nb as usize].constrained[kk] = true;
                            break;
                        }
                    }
                }

                return true;
            }
        }

        false
    }

    /// Lowest live triangle with vertex v, -1 when none.
    fn first_triangle_at(&self, v: i32) -> i32 {
        for ti in 0..self.triangles.len() {
            if self.triangles[ti].alive && self.has_vertex(ti as i32, v) {
                return ti as i32;
            }
        }

        -1
    }

    /// Triangle around v0 whose opposite edge the segment v0-v1 crosses, with that edge's left and right ends; None when none.
    fn first_crossed(&self, start_ti: i32, v0: i32, v1: i32) -> Option<(i32, i32, i32)> {
        let ax = self.vertices[v0 as usize].x;
        let ay = self.vertices[v0 as usize].y;
        let bx = self.vertices[v1 as usize].x;
        let by = self.vertices[v1 as usize].y;
        let mut ti = start_ti;

        for _ in 0..self.triangles.len() + 4 {
            if !self.triangles[ti as usize].alive {
                return None;
            }

            let k_v0 = self.vertex_index(ti, v0);

            if k_v0 < 0 {
                return None;
            }

            let k = k_v0 as usize;
            let ip2 = self.triangles[ti as usize].v[(k + 1) % 3];
            let ip1 = self.triangles[ti as usize].v[(k + 2) % 3];
            let p2 = &self.vertices[ip2 as usize];
            let p1 = &self.vertices[ip1 as usize];
            let op2 = Self::orient2d(ax, ay, bx, by, p2.x, p2.y);
            let op1 = Self::orient2d(ax, ay, bx, by, p1.x, p1.y);

            if op2 < 0.0 && op1 >= 0.0 {
                return Some((ti, ip1, ip2));
            }

            let next = self.triangles[ti as usize].adj[(k + 1) % 3];

            if next < 0 || !self.triangles[next as usize].alive || next == start_ti {
                return None;
            }

            ti = next;
        }

        None
    }

    /// Walk the triangles crossed by v0-v1 from intersected[0], collecting the vertices left and right of it.
    fn walk_crossed(
        &self,
        v0: i32,
        v1: i32,
        mut ivl: i32,
        mut ivr: i32,
        poly_l: &mut Vec<i32>,
        poly_r: &mut Vec<i32>,
        intersected: &mut Vec<i32>,
    ) {
        let ax = self.vertices[v0 as usize].x;
        let ay = self.vertices[v0 as usize].y;
        let bx = self.vertices[v1 as usize].x;
        let by = self.vertices[v1 as usize].y;
        let mut iv = v0;
        let mut cur_it = intersected[0];

        for _ in 0..self.triangles.len() * 2 + 8 {
            if self.has_vertex(cur_it, v1) {
                break;
            }

            let k_iv = self.vertex_index(cur_it, iv);

            if k_iv < 0 {
                break;
            }

            let i_topo = self.triangles[cur_it as usize].adj[k_iv as usize];

            if i_topo < 0 || !self.triangles[i_topo as usize].alive {
                break;
            }

            let i_vopo = self.opposite_vertex(i_topo, cur_it);

            if i_vopo < 0 {
                break;
            }

            let p = &self.vertices[i_vopo as usize];
            let o = Self::orient2d(ax, ay, bx, by, p.x, p.y);

            if o < 0.0 {
                if i_vopo != v1 {
                    poly_r.push(i_vopo);
                }

                iv = ivr;
                ivr = i_vopo;
            } else {
                if i_vopo != v1 {
                    poly_l.push(i_vopo);
                }

                iv = ivl;
                ivl = i_vopo;
            }

            intersected.push(i_topo);
            cur_it = i_topo;
        }
    }

    /// Replace the crossed triangles by the two fans on either side of v0-v1 and constrain it.
    fn retriangulate(
        &mut self,
        v0: i32,
        v1: i32,
        poly_l: &[i32],
        poly_r: &[i32],
        intersected: &[i32],
    ) {
        for &ti in intersected {
            self.unregister_edges(ti);
            self.triangles[ti as usize].alive = false;
        }

        let first_new = self.triangles.len();

        for i in 0..poly_l.len().saturating_sub(2) {
            self.add_triangle(v1, poly_l[i + 1], poly_l[i]);
        }

        for i in 1..poly_r.len().saturating_sub(1) {
            self.add_triangle(v0, poly_r[i], poly_r[i + 1]);
        }

        self.inherit_constraints(first_new);
        self.mark_edge(v0, v1);
    }

    /// Constrain every edge of a triangle from first_new on that its older neighbour holds constrained.
    fn inherit_constraints(&mut self, first_new: usize) {
        for new_ti in first_new..self.triangles.len() {
            if !self.triangles[new_ti].alive {
                continue;
            }

            for k in 0..3 {
                let nb = self.triangles[new_ti].adj[k];

                if nb < 0 || nb as usize >= first_new || !self.triangles[nb as usize].alive {
                    continue;
                }

                for kk in 0..3 {
                    if self.triangles[nb as usize].adj[kk] == new_ti as i32
                        && self.triangles[nb as usize].constrained[kk]
                    {
                        self.triangles[new_ti].constrained[k] = true;
                        break;
                    }
                }
            }
        }
    }

    /// Mark the edge v0-v1 constrained in every live triangle that has it.
    fn mark_edge(&mut self, v0: i32, v1: i32) {
        for tri in &mut self.triangles {
            if !tri.alive {
                continue;
            }

            for k in 0..3 {
                let e0 = tri.v[(k + 1) % 3];
                let e1 = tri.v[(k + 2) % 3];

                if (e0 == v0 && e1 == v1) || (e0 == v1 && e1 == v0) {
                    tri.constrained[k] = true;
                }
            }
        }
    }

    /// True when triangle ti has vertex v.
    fn has_vertex(&self, ti: i32, v: i32) -> bool {
        self.triangles[ti as usize].v[0] == v
            || self.triangles[ti as usize].v[1] == v
            || self.triangles[ti as usize].v[2] == v
    }

    /// New counter-clockwise triangle over three vertices; skipped when degenerate.
    fn add_triangle(&mut self, pa: i32, pb: i32, pc: i32) {
        let o = Self::orient2d(
            self.vertices[pa as usize].x,
            self.vertices[pa as usize].y,
            self.vertices[pb as usize].x,
            self.vertices[pb as usize].y,
            self.vertices[pc as usize].x,
            self.vertices[pc as usize].y,
        );

        if o.abs() < 1e-20 {
            return;
        }

        let new_ti = self.triangles.len() as i32;
        let (vb, vc) = if o > 0.0 { (pb, pc) } else { (pc, pb) };
        self.triangles.push(Triangle {
            v: [pa, vb, vc],
            adj: [-1, -1, -1],
            constrained: [false; 3],
            alive: true,
        });
        self.register_edges(new_ti);
    }

    /// Drop the triangles touching the super triangle.
    fn cleanup(&mut self) {
        let sv = self.super_v;

        for ti in 0..self.triangles.len() {
            if !self.triangles[ti].alive {
                continue;
            }

            for k in 0..3 {
                if self.triangles[ti].v[k] == sv[0]
                    || self.triangles[ti].v[k] == sv[1]
                    || self.triangles[ti].v[k] == sv[2]
                {
                    self.unregister_edges(ti as i32);
                    self.triangles[ti].alive = false;
                    break;
                }
            }
        }

        self.last_found = 0;

        for i in 0..self.triangles.len() {
            if self.triangles[i].alive {
                self.last_found = i as i32;
                break;
            }
        }
    }

    /// Vertex index triples of the live triangles.
    fn get_triangles(&self) -> Vec<[i32; 3]> {
        let mut result = Vec::new();

        for t in &self.triangles {
            if !t.alive {
                continue;
            }

            let [a, b, c] = [t.v[0], t.v[1], t.v[2]];
            let o = Self::orient2d(
                self.vertices[a as usize].x,
                self.vertices[a as usize].y,
                self.vertices[b as usize].x,
                self.vertices[b as usize].y,
                self.vertices[c as usize].x,
                self.vertices[c as usize].y,
            );
            result.push(if o > 0.0 { [a, b, c] } else { [a, c, b] });
        }

        result
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Triangulation
// ═══════════════════════════════════════════════════════════════════════════
/// Loop polygon in UV before refinement: the control points of a polyline, else samples, the closing repeat dropped.
fn loop_points(crv: &NurbsCurve) -> Vec<Point> {
    let mut raw: Vec<Point> = Vec::new();

    if crv.degree() <= 1 && !crv.is_rational() {
        for i in 0..crv.cv_count() {
            raw.push(crv.get_cv(i).unwrap_or_default());
        }
    } else {
        let n = (crv.cv_count() * 4).clamp(16, 2048);
        raw = crv.divide_by_count(n, true).0;
    }

    while raw.len() > 1 {
        let dx = raw[0][0] - raw[raw.len() - 1][0];
        let dy = raw[0][1] - raw[raw.len() - 1][1];

        if dx * dx + dy * dy < 1e-20 {
            raw.pop();
        } else {
            break;
        }
    }

    raw
}

/// Append the UV points of the edge start-end without end, halved up to six times until each 3D chord is within deflection.
fn subdivide_edge(
    srf: &NurbsSurface,
    start: &Point,
    end: &Point,
    deflection: f64,
    out: &mut Vec<Point>,
) {
    let mut stack: Vec<(Point, Point, i32)> = vec![(start.clone(), end.clone(), 0)];

    while let Some((a, b, depth)) = stack.pop() {
        let mu = (a[0] + b[0]) * 0.5;
        let mv = (a[1] + b[1]) * 0.5;
        let pa = srf.point_at(a[0], a[1]).unwrap_or_default();
        let pm = srf.point_at(mu, mv).unwrap_or_default();
        let edge = &srf.point_at(b[0], b[1]).unwrap_or_default() - &pa;
        let l2 = edge.magnitude_squared();
        let dev = if l2 > 1e-30 {
            let t = (&pm - &pa).dot(&edge) / l2;
            (&pm - &(&pa + &(&edge * t))).magnitude_squared().sqrt()
        } else {
            (&pm - &pa).magnitude_squared().sqrt()
        };

        if dev > deflection && depth < 6 {
            stack.push((Point::new(mu, mv, 0.0), b, depth + 1));
            stack.push((a, Point::new(mu, mv, 0.0), depth + 1));
        } else {
            out.push(a);
        }
    }
}

/// Interior knots per direction whose multiplicity reaches the degree: the C0 lines of the surface.
fn find_crease_knots(surface: &NurbsSurface) -> [Vec<f64>; 2] {
    let mut crease_knots = [Vec::new(), Vec::new()];

    for dir in 0..2 {
        let Some((start, end)) = surface.domain(dir) else {
            continue;
        };
        let knots = &surface.m_nurbsknot[dir];

        for &knot in knots {
            if knot <= start || knot >= end || crease_knots[dir].contains(&knot) {
                continue;
            }

            let mut multiplicity = 0;

            for &value in knots {
                if value == knot {
                    multiplicity += 1;
                }
            }

            if multiplicity >= surface.degree(dir) {
                crease_knots[dir].push(knot);
            }
        }
    }

    crease_knots
}

/// UV bounds (umin, vmin, umax, vmax) of a loop polygon.
fn loop_bounds(pts: &[Point]) -> [f64; 4] {
    let mut bounds = [1e30_f64, 1e30_f64, -1e30_f64, -1e30_f64];

    for p in pts {
        if p[0] < bounds[0] {
            bounds[0] = p[0];
        }

        if p[1] < bounds[1] {
            bounds[1] = p[1];
        }

        if p[0] > bounds[2] {
            bounds[2] = p[0];
        }

        if p[1] > bounds[3] {
            bounds[3] = p[1];
        }
    }

    bounds
}

/// Constrain loop edge i in pieces cut where it crosses a crease knot line, each crossing inserted and recorded.
fn insert_loop_edge(
    dt: &mut Delaunay2D,
    pts: &[Point],
    vis: &[i32],
    li: usize,
    i: usize,
    crease_knots: &[Vec<f64>; 2],
    boundary_intervals: &mut BTreeMap<usize, (usize, usize, f64)>,
) {
    let j = (i + 1) % vis.len();
    let mut events = vec![(0.0, vis[i]), (1.0, vis[j])];

    for dir in 0..2 {
        let delta = pts[j][dir] - pts[i][dir];

        if delta == 0.0 {
            continue;
        }

        for &knot in &crease_knots[dir] {
            let t = (knot - pts[i][dir]) / delta;

            if t <= 0.0 || t >= 1.0 {
                continue;
            }

            let mut uv = [
                pts[i][0] + t * (pts[j][0] - pts[i][0]),
                pts[i][1] + t * (pts[j][1] - pts[i][1]),
            ];
            uv[dir] = knot;
            let vi = dt.insert(uv[0], uv[1]);

            if vi >= 0 {
                boundary_intervals.insert(vi as usize, (li, i, t));
            }

            events.push((t, vi));
        }
    }

    events.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));

    for k in 1..events.len() {
        if events[k - 1].1 >= 0 && events[k].1 >= 0 && events[k - 1].1 != events[k].1 {
            dt.insert_constraint(events[k - 1].1, events[k].1);
        }
    }
}

/// Insert each loop's vertices, then constrain its edges; the vertex index of every loop sample.
fn insert_loops(
    dt: &mut Delaunay2D,
    loops_uv: &[Vec<Point>],
    crease_knots: &[Vec<f64>; 2],
    boundary_intervals: &mut BTreeMap<usize, (usize, usize, f64)>,
) -> Vec<Vec<i32>> {
    let mut loop_vids: Vec<Vec<i32>> = Vec::new();

    for (li, pts) in loops_uv.iter().enumerate() {
        let mut vis: Vec<i32> = Vec::new();

        for p in pts {
            vis.push(dt.insert(p[0], p[1]));
        }

        for i in 0..vis.len() {
            insert_loop_edge(dt, pts, &vis, li, i, crease_knots, boundary_intervals);
        }

        loop_vids.push(vis);
    }

    loop_vids
}

/// Insert the crease knot crossings inside the loops and constrain each knot line between consecutive vertices on it.
fn insert_crease_lines(dt: &mut Delaunay2D, loops_uv: &[Vec<Point>], crease_knots: &[Vec<f64>; 2]) {
    for &u in &crease_knots[0] {
        for &v in &crease_knots[1] {
            if inside_loops(u, v, loops_uv) {
                dt.insert(u, v);
            }
        }
    }

    for dir in 0..2 {
        for &knot in &crease_knots[dir] {
            let mut nodes = Vec::new();

            for (vi, vertex) in dt.vertices.iter().enumerate() {
                let uv = [vertex.x, vertex.y];

                if uv[dir] == knot {
                    nodes.push((uv[1 - dir], vi as i32));
                }
            }

            nodes.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));

            for k in 1..nodes.len() {
                let mut uv = [knot, knot];
                uv[1 - dir] = (nodes[k - 1].0 + nodes[k].0) * 0.5;

                if inside_loops(uv[0], uv[1], loops_uv) {
                    dt.insert_constraint(nodes[k - 1].1, nodes[k].1);
                }
            }
        }
    }
}

/// Smallest dot product between the crease-side normals at the corners of triangle abc around its centroid.
fn min_normal_dot(
    surface: &NurbsSurface,
    crease_knots: &[Vec<f64>; 2],
    center: [f64; 2],
    a: &Vertex2D,
    b: &Vertex2D,
    c: &Vertex2D,
) -> f64 {
    let na = crease_side_normal(surface, crease_knots, center, [a.x, a.y]);
    let nb = crease_side_normal(surface, crease_knots, center, [b.x, b.y]);
    let nc2 = crease_side_normal(surface, crease_knots, center, [c.x, c.y]);
    let d1 = na.dot(&nb);
    let d2 = nb.dot(&nc2);
    let d3 = na.dot(&nc2);

    d1.min(d2.min(d3))
}

/// Centroids of the live triangles inside the loops whose chord leaves deflection or whose corner normals turn past the angle bound.
fn refinement_points(
    dt: &Delaunay2D,
    surface: &NurbsSurface,
    loops_uv: &[Vec<Point>],
    crease_knots: &[Vec<f64>; 2],
    deflection: f64,
    cos_max_angle: f64,
) -> Vec<[f64; 2]> {
    let mut to_insert: Vec<[f64; 2]> = Vec::new();

    for tri in &dt.triangles {
        if !tri.alive {
            continue;
        }

        let a = &dt.vertices[tri.v[0] as usize];
        let b = &dt.vertices[tri.v[1] as usize];
        let c = &dt.vertices[tri.v[2] as usize];
        let cu = (a.x + b.x + c.x) / 3.0;
        let cv = (a.y + b.y + c.y) / 3.0;

        if !inside_loops(cu, cv, loops_uv) {
            continue;
        }

        let pa = surface.point_at(a.x, a.y).unwrap_or_default();
        let pb = surface.point_at(b.x, b.y).unwrap_or_default();
        let pc = surface.point_at(c.x, c.y).unwrap_or_default();
        let pm = surface.point_at(cu, cv).unwrap_or_default();
        let n = (&pb - &pa).cross(&(&pc - &pa));
        let nl = n.magnitude_squared().sqrt();

        if nl < 1e-30 {
            continue;
        }

        let dev = ((&pm - &pa).dot(&n) / nl).abs();

        if dev > deflection
            || min_normal_dot(surface, crease_knots, [cu, cv], a, b, c) < cos_max_angle
        {
            to_insert.push([cu, cv]);
        }
    }

    to_insert
}

/// Insert refinement centroids for up to eight rounds, until none is needed or the vertex cap is hit.
fn refine(
    dt: &mut Delaunay2D,
    surface: &NurbsSurface,
    loops_uv: &[Vec<Point>],
    crease_knots: &[Vec<f64>; 2],
    deflection: f64,
    cos_max_angle: f64,
) {
    const MAX_ITERS: i32 = 8;
    const MAX_VERTS: usize = 200000;

    for _iter in 0..MAX_ITERS {
        let to_insert = refinement_points(
            dt,
            surface,
            loops_uv,
            crease_knots,
            deflection,
            cos_max_angle,
        );

        if to_insert.is_empty() {
            break;
        }

        for uv in &to_insert {
            if dt.vertices.len() >= MAX_VERTS {
                break;
            }

            dt.insert(uv[0], uv[1]);
        }

        if dt.vertices.len() >= MAX_VERTS {
            break;
        }
    }
}

/// Drop the super triangle and every triangle whose centroid lies outside the loops.
fn trim_outside(dt: &mut Delaunay2D, loops_uv: &[Vec<Point>]) {
    dt.cleanup();

    for ti in 0..dt.triangles.len() {
        if !dt.triangles[ti].alive {
            continue;
        }

        let [v0, v1, v2] = dt.triangles[ti].v;
        let cu =
            (dt.vertices[v0 as usize].x + dt.vertices[v1 as usize].x + dt.vertices[v2 as usize].x)
                / 3.0;
        let cv =
            (dt.vertices[v0 as usize].y + dt.vertices[v1 as usize].y + dt.vertices[v2 as usize].y)
                / 3.0;

        if !inside_loops(cu, cv, loops_uv) {
            dt.triangles[ti].alive = false;
        }
    }
}

/// True when a triangle spans a crease knot line in either direction.
fn crosses_crease(tris: &[[i32; 3]], dt: &Delaunay2D, crease_knots: &[Vec<f64>; 2]) -> bool {
    for tri in tris {
        for dir in 0..2 {
            let mut low = f64::INFINITY;
            let mut high = f64::NEG_INFINITY;

            for &vi in tri {
                let value = [dt.vertices[vi as usize].x, dt.vertices[vi as usize].y][dir];
                low = low.min(value);
                high = high.max(value);
            }

            for &knot in &crease_knots[dir] {
                if low < knot && knot < high {
                    return true;
                }
            }
        }
    }

    false
}

/// Loop and sample of the 3D point given for each triangulation vertex, None where none is.
fn given_points(
    count: usize,
    loops: &TrimLoops,
    loop_vids: &[Vec<i32>],
) -> Vec<Option<(usize, usize)>> {
    let mut given: Vec<Option<(usize, usize)>> = vec![None; count];

    for (li, vids) in loop_vids.iter().enumerate() {
        if li >= loops.xyz.len() {
            break;
        }

        for (k, &vi) in vids.iter().enumerate() {
            if vi >= 0 && k < loops.xyz[li].len() {
                given[vi as usize] = Some((li, k));
            }
        }
    }

    given
}

/// 3D point of triangulation vertex vi: its given loop point, the loop chord at a knot crossing, else the surface point.
fn vertex_point(
    surface: &NurbsSurface,
    dt: &Delaunay2D,
    vi: usize,
    loops: &TrimLoops,
    given: &[Option<(usize, usize)>],
    boundary_intervals: &BTreeMap<usize, (usize, usize, f64)>,
) -> Point {
    if let Some((li, k)) = given[vi] {
        return loops.xyz[li][k].clone();
    }

    if let Some(&(li, segment, t)) = boundary_intervals.get(&vi) {
        if !loops.xyz.is_empty() {
            let a = &loops.xyz[li][segment];
            let b = &loops.xyz[li][(segment + 1) % loops.xyz[li].len()];

            return a + &(&(b - a) * t);
        }
    }

    surface
        .point_at(dt.vertices[vi].x, dt.vertices[vi].y)
        .unwrap_or_default()
}

/// Welded mesh vertex of every triangulation vertex a triangle uses, None for the others.
fn weld_vertices(
    welder: &mut VertexWelder,
    mesh: &mut Mesh,
    surface: &NurbsSurface,
    dt: &Delaunay2D,
    tris: &[[i32; 3]],
    loops: &TrimLoops,
    loop_vids: &[Vec<i32>],
    boundary_intervals: &BTreeMap<usize, (usize, usize, f64)>,
) -> Vec<Option<usize>> {
    let given = given_points(dt.vertices.len(), loops, loop_vids);
    let mut vert_map: Vec<Option<usize>> = vec![None; dt.vertices.len()];

    for tri in tris {
        for &vi in tri {
            if vert_map[vi as usize].is_none() {
                let p = vertex_point(surface, dt, vi as usize, loops, &given, boundary_intervals);
                vert_map[vi as usize] = Some(welder.weld(mesh, p));
            }
        }
    }

    vert_map
}

/// One face per triangle over its welded vertices, collapsed ones skipped.
fn add_faces(mesh: &mut Mesh, tris: &[[i32; 3]], vert_map: &[Option<usize>]) {
    for &[a, b, c] in tris {
        let (Some(v0), Some(v1), Some(v2)) = (
            vert_map[a as usize],
            vert_map[b as usize],
            vert_map[c as usize],
        ) else {
            continue;
        };

        if v0 == v1 || v1 == v2 || v2 == v0 {
            continue;
        }

        mesh.add_face(vec![v0, v1, v2], None);
    }
}

/// Area-weighted sum of the face normals around each mesh vertex.
fn fan_normals(mesh: &Mesh) -> HashMap<usize, Vector> {
    let mut fan: HashMap<usize, Vector> = HashMap::new();
    let mut fkeys: Vec<usize> = Vec::new();

    for &fk in mesh.face.keys() {
        fkeys.push(fk);
    }

    fkeys.sort_unstable();

    for fk in fkeys {
        let verts = &mesh.face[&fk];
        let a = mesh.vertex[&verts[0]].position();
        let b = mesh.vertex[&verts[1]].position();
        let c = mesh.vertex[&verts[2]].position();
        let n = (&b - &a).cross(&(&c - &a));

        for &vk in verts {
            *fan.entry(vk).or_insert(Vector::new(0.0, 0.0, 0.0)) += &n;
        }
    }

    fan
}

/// Normal of every used vertex from the surface derivatives, the fan normal where they degenerate, and its u and v.
fn set_vertex_normals(
    mesh: &mut Mesh,
    surface: &NurbsSurface,
    dt: &Delaunay2D,
    vert_map: &[Option<usize>],
) {
    let fan = fan_normals(mesh);

    for vi in 0..vert_map.len() {
        let Some(vk) = vert_map[vi] else {
            continue;
        };
        let u = dt.vertices[vi].x;
        let v = dt.vertices[vi].y;
        let derivatives = surface.evaluate(u, v, 1);
        let mut nrm = Vector::new(0.0, 0.0, 0.0);

        if derivatives.len() >= 3 {
            nrm = derivatives[2].cross(&derivatives[1]);
        }

        let nl = nrm.magnitude_squared().sqrt();

        if nl.is_finite() && nl > 0.0 {
            nrm = &nrm / nl;
        } else {
            let f = fan.get(&vk).cloned().unwrap_or(Vector::new(0.0, 0.0, 1.0));
            let fl = f.magnitude_squared().sqrt();
            nrm = if fl.is_finite() && fl > 0.0 {
                &f / fl
            } else {
                Vector::new(0.0, 0.0, 1.0)
            };
        }

        if let Some(vd) = mesh.vertex.get_mut(&vk) {
            vd.set_normal(nrm[0], nrm[1], nrm[2]);
            vd.attributes.insert("u".to_string(), u);
            vd.attributes.insert("v".to_string(), v);
        }
    }
}

/// Tag loop vertices boundary/{loop}/{sample} and knot crossings boundary_interval/{loop}/{segment} with their chord parameter.
fn tag_boundary(
    mesh: &mut Mesh,
    loop_vids: &[Vec<i32>],
    boundary_intervals: &BTreeMap<usize, (usize, usize, f64)>,
    vert_map: &[Option<usize>],
) {
    for (li, vids) in loop_vids.iter().enumerate() {
        for (k, &vi) in vids.iter().enumerate() {
            if vi < 0 {
                continue;
            }

            let key = format!("boundary/{li}/{k}");

            if let Some(vk) = vert_map[vi as usize] {
                if let Some(vd) = mesh.vertex.get_mut(&vk) {
                    vd.attributes.insert(key, 1.0);
                }
            }
        }
    }

    for (&vi, &(li, segment, t)) in boundary_intervals {
        let key = format!("boundary_interval/{li}/{segment}");

        if let Some(vk) = vert_map[vi] {
            if let Some(vd) = mesh.vertex.get_mut(&vk) {
                vd.attributes.insert(key, t);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TrimLoops
// ═══════════════════════════════════════════════════════════════════════════
/// Trim wires of one face as UV polygons, optional 3D points per loop vertex shared bit for bit with the neighbouring face, and interior UV seeds.
#[derive(Debug, Clone, Default)]
pub struct TrimLoops {
    pub uv: Vec<Vec<Point>>,     // UV polygon per loop.
    pub xyz: Vec<Vec<Point>>,    // 3D point per loop vertex, empty when not shared.
    pub interior_uv: Vec<Point>, // UV seeds inside the face.
}

// ═══════════════════════════════════════════════════════════════════════════
// NurbsSurfaceTrimmed
// ═══════════════════════════════════════════════════════════════════════════
/// A NURBS surface bounded by a closed outer loop and optional inner loops in its UV space.
#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(tag = "type", rename = "NurbsSurfaceTrimmed")]
pub struct NurbsSurfaceTrimmed {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: std::sync::OnceLock<String>, // Lazily minted GUID.
    pub name: String,        // Face name.
    pub width: f64,          // Display width.
    pub surfacecolor: Color, // Display color of the surface.
    #[serde(rename = "surface")]
    pub m_surface: NurbsSurface, // Underlying surface.
    #[serde(rename = "outer_loop")]
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(default)]
    pub m_outer_loop: Option<NurbsCurve>, // Closed outer loop in UV space.
    #[serde(rename = "inner_loops")]
    #[serde(default)]
    pub m_inner_loops: Vec<NurbsCurve>, // Closed hole loops in UV space.
    // SESSION_VIEWER
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cut_q0: Option<Point>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cut_n: Option<Vector>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub cut_planes: Vec<(Point, Vector)>,
}

impl Default for NurbsSurfaceTrimmed {
    /// Construct an empty untrimmed face.
    fn default() -> Self {
        Self::new()
    }
}

impl NurbsSurfaceTrimmed {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty untrimmed face.
    pub fn new() -> Self {
        NurbsSurfaceTrimmed {
            guid: std::sync::OnceLock::new(),
            name: "my_nurbssurface_trimmed".to_string(),
            width: 1.0,
            surfacecolor: Color::black(),
            m_surface: NurbsSurface::default(),
            m_outer_loop: None,
            m_inner_loops: Vec::new(),
            cut_q0: None,
            cut_n: None,
            cut_planes: Vec::new(),
        }
    }

    /// Copy with a new guid and the same data.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = std::sync::OnceLock::new();

        copy
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Surface with a closed outer loop given in its UV parameter space.
    pub fn create(surface: &NurbsSurface, outer_loop: &NurbsCurve) -> Self {
        let mut ts = Self::new();
        ts.m_surface = surface.duplicate();
        ts.m_outer_loop = Some(outer_loop.duplicate());

        ts
    }

    /// Planar surface fitted to a closed 3D boundary, the boundary projected as the outer loop.
    pub fn create_planar(boundary: &NurbsCurve) -> Self {
        let srf = Primitives::create_planar(boundary);

        if !srf.is_valid() {
            return Self::new();
        }

        let p00 = srf.get_cv(0, 0).unwrap_or_default();
        let u_axis = &srf.get_cv(1, 0).unwrap_or_default() - &p00;
        let v_axis = &srf.get_cv(0, 1).unwrap_or_default() - &p00;
        let u_len2 = u_axis.magnitude_squared();
        let v_len2 = v_axis.magnitude_squared();

        if u_len2 < 1e-28 || v_len2 < 1e-28 {
            return Self::new();
        }

        let mut uv_pts: Vec<Point> = Vec::new();

        if boundary.degree() <= 1 {
            for i in 0..boundary.cv_count() {
                uv_pts.push(project_to_uv(
                    &boundary.get_cv(i).unwrap_or_default(),
                    &p00,
                    &u_axis,
                    &v_axis,
                    u_len2,
                    v_len2,
                ));
            }
        } else {
            let spans = boundary.get_span_vector();
            let n_sub = 10;

            for si in 0..spans.len().saturating_sub(1) {
                for k in 0..=n_sub {
                    let t = spans[si] + (spans[si + 1] - spans[si]) * k as f64 / n_sub as f64;
                    let uv = project_to_uv(
                        &boundary.point_at(t),
                        &p00,
                        &u_axis,
                        &v_axis,
                        u_len2,
                        v_len2,
                    );

                    if uv_pts.is_empty()
                        || (&uv - &uv_pts[uv_pts.len() - 1]).magnitude_squared() > 1e-24
                    {
                        uv_pts.push(uv);
                    }
                }
            }
        }

        let mut ts = Self::new();
        ts.m_surface = srf;

        if uv_pts.len() >= 3 {
            ts.m_outer_loop = Some(NurbsCurve::create(false, 1, &uv_pts));
        }

        ts
    }

    /// One trimmed face per region of the UV domain carved by the pcurves (x=u, y=v, z=0); dangling cutters are discarded.
    pub fn split_by_uv_curves(
        srf: &NurbsSurface,
        pcurves: &[NurbsCurve],
        tolerance: f64,
    ) -> Vec<NurbsSurfaceTrimmed> {
        if !srf.is_valid() {
            return Vec::new();
        }

        let dom = split_domain(srf, tolerance);
        let polylines = uv_polylines(pcurves, &dom);
        let mut pool = UVVertexPool::new(dom.snap);
        let splits = polyline_crossings(&polylines, pcurves, &dom);
        let live_edges = prune_dangling(&split_edges(&polylines, &splits, &mut pool));

        if live_edges.is_empty() {
            return Vec::new();
        }

        let verts = &pool.verts;
        let hes = half_edges(&live_edges);
        let faces = face_cycles(&next_half_edges(&hes, verts));
        let (pos_faces, neg_faces) = classify_faces(faces, &hes, verts, &live_edges, dom.snap);
        let holes_of = assign_holes(&neg_faces, &pos_faces, &hes, verts);
        let mut result: Vec<NurbsSurfaceTrimmed> = Vec::new();

        for (fi, (cycle, _area)) in pos_faces.iter().enumerate() {
            let mut outer = cycle_to_loop(cycle, &hes, &live_edges, verts, pcurves, dom.snap);

            if !outer.is_valid() || (loop_signed_area(&outer) < 0.0 && !outer.reverse()) {
                continue;
            }

            let mut ts = NurbsSurfaceTrimmed::create(srf, &outer);

            for hole_cycle in &holes_of[fi] {
                let mut hole =
                    cycle_to_loop(hole_cycle, &hes, &live_edges, verts, pcurves, dom.snap);

                if !hole.is_valid() || (loop_signed_area(&hole) > 0.0 && !hole.reverse()) {
                    continue;
                }

                ts.add_inner_loop(hole);
            }

            result.push(ts);
        }

        result
    }

    /// One trimmed face per non-empty region carved by the planes (all 2^K sign combinations).
    pub fn split_by_planes(
        srf: &NurbsSurface,
        planes: &[(Point, Vector)],
    ) -> Vec<NurbsSurfaceTrimmed> {
        let mut out = Vec::new();
        let k = planes.len();

        if k == 0 || k > 16 {
            return out;
        }

        for mask in 0u32..(1u32 << k) {
            let mut cp: Vec<(Point, Vector)> = Vec::new();

            for i in 0..k {
                let q = &planes[i].0;
                let n = &planes[i].1;
                let flip = ((mask >> i) & 1) == 1;
                let nn = if flip {
                    Vector::new(-n[0], -n[1], -n[2])
                } else {
                    Vector::new(n[0], n[1], n[2])
                };
                cp.push((q.clone(), nn));
            }

            let mut ts = NurbsSurfaceTrimmed::new();
            ts.m_surface = srf.duplicate();
            let m = ts.mesh_by_planes(&cp, 20.0, 0.01);

            if m.number_of_faces() > 0 {
                // SESSION_VIEWER
                ts.cut_planes = cp;
                out.push(ts);
            }
        }

        out
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform the surface in place; the loops live in UV and stay.
    pub fn transform(&mut self, xform: &Xform) {
        self.m_surface.transform(xform);

        // SESSION_VIEWER
        if let Some(q0) = self.cut_q0.as_mut() {
            q0.transform(xform);
        }

        if let Some(n) = self.cut_n.as_mut() {
            n.transform(xform);
        }

        for (q, n) in self.cut_planes.iter_mut() {
            q.transform(xform);
            n.transform(xform);
        }
    }

    /// Return a transformed copy.
    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut ts = self.duplicate();
        ts.transform(xform);

        ts
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the guid, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid if it has not already been created.
    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Return the underlying surface.
    pub fn surface(&self) -> &NurbsSurface {
        &self.m_surface
    }

    /// Return the outer loop, None when untrimmed.
    pub fn get_outer_loop(&self) -> Option<&NurbsCurve> {
        self.m_outer_loop.as_ref()
    }

    /// Replace the outer loop.
    pub fn set_outer_loop(&mut self, loop_crv: NurbsCurve) {
        self.m_outer_loop = Some(loop_crv);
    }

    /// Return whether the outer loop is a valid curve.
    pub fn is_trimmed(&self) -> bool {
        self.m_outer_loop.as_ref().is_some_and(|c| c.is_valid())
    }

    /// Return whether the underlying surface is valid.
    pub fn is_valid(&self) -> bool {
        self.m_surface.is_valid()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Inner loops
    // ═══════════════════════════════════════════════════════════════════════════
    /// Hole given directly as a closed 2D curve in UV space.
    pub fn add_inner_loop(&mut self, loop_2d: NurbsCurve) {
        self.m_inner_loops.push(loop_2d);
    }

    /// Hole from a 3D curve pulled onto the surface and normalized into [0,1]^2.
    pub fn add_hole(&mut self, curve_3d: &NurbsCurve) {
        let dom = curve_3d.domain();
        let sdom_u = self.m_surface.domain(0).unwrap_or((0.0, 1.0));
        let sdom_v = self.m_surface.domain(1).unwrap_or((0.0, 1.0));
        let range_u = sdom_u.1 - sdom_u.0;
        let range_v = sdom_v.1 - sdom_v.0;
        let n_samples = (curve_3d.cv_count() * 4).clamp(32, 2048);
        let mut uv_pts = Vec::new();

        for i in 0..n_samples {
            let t = dom.0 + (dom.1 - dom.0) * i as f64 / n_samples as f64;
            let pt3d = curve_3d.point_at(t);
            let (u, v, _dist) = Closest::surface_point(&self.m_surface, &pt3d, 0.0, 0.0, 0.0, 0.0);
            let nu = (u - sdom_u.0) / range_u;
            let nv = (v - sdom_v.0) / range_v;
            uv_pts.push(Point::new(nu, nv, 0.0));
        }

        if uv_pts.len() >= 3 {
            self.m_inner_loops
                .push(NurbsCurve::create(true, 1, &uv_pts));
        }
    }

    /// Add one hole per 3D curve pulled onto the surface.
    pub fn add_holes(&mut self, curves_3d: &[NurbsCurve]) {
        for crv in curves_3d {
            self.add_hole(crv);
        }
    }

    /// Return the inner loop at index.
    pub fn get_inner_loop(&self, index: usize) -> &NurbsCurve {
        &self.m_inner_loops[index]
    }

    /// Return the number of inner loops.
    pub fn inner_loop_count(&self) -> usize {
        self.m_inner_loops.len()
    }

    /// Remove every inner loop.
    pub fn clear_inner_loops(&mut self) {
        self.m_inner_loops.clear();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the surface point at (u, v), None when the surface is invalid.
    pub fn point_at(&self, u: f64, v: f64) -> Option<Point> {
        self.m_surface.point_at(u, v)
    }

    /// Return the unit surface normal at (u, v).
    pub fn normal_at(&self, u: f64, v: f64) -> Vector {
        self.m_surface.normal_at(u, v)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Meshing
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return mesh_q at 20 degrees and a chord factor of 0.005.
    pub fn mesh(&self) -> Mesh {
        self.mesh_q(20.0, 0.005)
    }

    /// Deflection-refined constrained Delaunay of the trim loops: angular bound in degrees, chord factor as a fraction of the bbox diagonal.
    pub fn mesh_q(&self, max_angle_deg: f64, chord_factor: f64) -> Mesh {
        if !self.is_trimmed() {
            return self.m_surface.mesh();
        }

        let deflection = self.bbox_diagonal() * chord_factor;

        let mut loops = TrimLoops::default();
        let Some(outer) = self.m_outer_loop.as_ref() else {
            return self.m_surface.mesh();
        };
        loops.uv.push(self.discretize_loop(outer, deflection));

        for inner in &self.m_inner_loops {
            loops.uv.push(self.discretize_loop(inner, deflection));
        }

        self.triangulate(&loops, max_angle_deg, chord_factor)
    }

    /// Mesh sampled loops (outer first, then holes) keeping every loop vertex, tagged boundary/{loop}/{sample}; knot crossings add boundary_interval/{loop}/{segment}; empty mesh on invalid input.
    pub fn mesh_loops(&self, loops: &TrimLoops, max_angle_deg: f64, chord_factor: f64) -> Mesh {
        if loops.uv.is_empty()
            || !max_angle_deg.is_finite()
            || max_angle_deg <= 0.0
            || !chord_factor.is_finite()
            || chord_factor <= 0.0
            || (!loops.xyz.is_empty() && loops.xyz.len() != loops.uv.len())
        {
            return Mesh::new();
        }

        let mut expected = 0;

        for (li, points) in loops.uv.iter().enumerate() {
            if points.len() < 3 || (!loops.xyz.is_empty() && loops.xyz[li].len() != points.len()) {
                return Mesh::new();
            }

            for point in points {
                if !point[0].is_finite() || !point[1].is_finite() {
                    return Mesh::new();
                }
            }

            if !loops.xyz.is_empty() {
                for point in &loops.xyz[li] {
                    if !point[0].is_finite() || !point[1].is_finite() || !point[2].is_finite() {
                        return Mesh::new();
                    }
                }
            }

            expected += points.len();
        }

        let result = self.triangulate(loops, max_angle_deg, chord_factor);
        let mut actual: HashSet<String> = HashSet::new();

        for vd in result.vertex.values() {
            for name in vd.attributes.keys() {
                if name.starts_with("boundary/") {
                    actual.insert(name.clone());
                }
            }
        }

        if actual.len() == expected {
            result
        } else {
            Mesh::new()
        }
    }

    /// Mesh of the half (S-q0).n <= 0: span-adaptive grid, marching-squares clip with Newton-refined crossings, seams welded.
    pub fn mesh_by_plane(
        &self,
        q0: &Point,
        normal: &Vector,
        max_angle_deg: f64,
        chord_factor: f64,
    ) -> Mesh {
        let srf = &self.m_surface;
        let Some(n) = unit3(normal) else {
            return srf.mesh();
        };
        let q = [q0[0], q0[1], q0[2]];
        let bbox_diag = self.bbox_diagonal();
        let Some((us, vs)) = span_grid(srf, max_angle_deg, bbox_diag * chord_factor) else {
            return srf.mesh();
        };
        let nu = us.len();
        let nv = vs.len();
        let mut field = vec![vec![0.0f64; nv]; nu];

        for i in 0..nu {
            for j in 0..nv {
                field[i][j] = plane_field(srf, &q, &n, us[i], vs[j]);
            }
        }

        let mut result = Mesh::new();
        let weld_tol = bbox_diag * 1e-5;
        let mut welder = VertexWelder::new(weld_tol, weld_tol);

        for i in 0..nu - 1 {
            for j in 0..nv - 1 {
                let cu = [us[i], us[i + 1], us[i + 1], us[i]];
                let cv = [vs[j], vs[j], vs[j + 1], vs[j + 1]];
                let fc = [
                    field[i][j],
                    field[i + 1][j],
                    field[i + 1][j + 1],
                    field[i][j + 1],
                ];
                let poly = clip_cell(&mut welder, &mut result, srf, &q, &n, &cu, &cv, &fc);
                add_fan(&mut result, &poly);
            }
        }

        if result.face.is_empty() {
            return srf.mesh();
        }

        result
    }

    /// Mesh of the region inside every half-space (S-q).n <= 0: triangle soup clipped plane by plane, seams welded.
    pub fn mesh_by_planes(
        &self,
        planes: &[(Point, Vector)],
        max_angle_deg: f64,
        chord_factor: f64,
    ) -> Mesh {
        let srf = &self.m_surface;
        let mut pl: Vec<([f64; 3], [f64; 3])> = Vec::new();

        for (qn_q, qn_n) in planes {
            let Some(n) = unit3(qn_n) else {
                continue;
            };
            pl.push(([qn_q[0], qn_q[1], qn_q[2]], n));
        }

        if pl.is_empty() {
            return srf.mesh();
        }

        let bbox_diag = self.bbox_diagonal();
        let Some((us, vs)) = span_grid(srf, max_angle_deg, bbox_diag * chord_factor) else {
            return srf.mesh();
        };
        let mut tris = grid_triangles(&us, &vs);

        for (q, n) in &pl {
            tris = clip_triangles(srf, q, n, &tris);

            if tris.is_empty() {
                break;
            }
        }

        if tris.is_empty() {
            return Mesh::new();
        }

        let result = weld_triangles(srf, &tris, bbox_diag * 1e-5);

        if result.face.is_empty() {
            return Mesh::new();
        }

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a sorted JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_default()
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = serde_json::to_string_pretty(self)?;
        std::fs::write(filepath, json)?;

        Ok(())
    }

    /// Read from a JSON file.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let contents = std::fs::read_to_string(filepath)?;

        Ok(serde_json::from_str(&contents)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::NurbsSurfaceTrimmed {
        let mut outer_loop = None;

        if self.is_trimmed() {
            if let Some(outer) = self.m_outer_loop.as_ref() {
                outer_loop = Some(outer.to_proto());
            }
        }

        let mut inner_loops = Vec::new();

        for inner in &self.m_inner_loops {
            inner_loops.push(inner.to_proto());
        }

        crate::proto::NurbsSurfaceTrimmed {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            surface: Some(self.m_surface.to_proto()),
            outer_loop,
            inner_loops,
            width: self.width,
            surfacecolor: Some(self.surfacecolor.to_proto()),
        }
    }

    /// Construct from the protobuf message.
    pub fn from_proto(
        proto: crate::proto::NurbsSurfaceTrimmed,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let mut ts = Self::new();

        if !proto.guid.is_empty() {
            ts.set_guid(proto.guid.clone());
        }

        ts.name = proto.name;
        ts.width = proto.width;

        if let Some(surface) = proto.surface {
            ts.m_surface = NurbsSurface::from_proto(surface)?;
        }

        if let Some(outer) = proto.outer_loop {
            ts.m_outer_loop = Some(NurbsCurve::from_proto(outer));
        }

        for inner in proto.inner_loops {
            ts.m_inner_loops.push(NurbsCurve::from_proto(inner));
        }

        ts.surfacecolor = Color::from_proto(proto.surfacecolor.unwrap_or_default());

        Ok(ts)
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Self::from_proto(crate::proto::NurbsSurfaceTrimmed::decode(data)?)
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&self, filepath: &str) {
        let _ = std::fs::write(filepath, self.pb_dumps());
    }

    /// Read from a protobuf file.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).unwrap_or_default();

        Self::pb_loads(&data).unwrap_or_default()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "NurbsSurfaceTrimmed(name=..., trimmed=..., holes=...)".
    pub fn str(&self) -> String {
        format!(
            "NurbsSurfaceTrimmed(name={}, trimmed={}, holes={})",
            self.name,
            self.is_trimmed(),
            self.inner_loop_count()
        )
    }

    /// Return the multi-line form with the surface.
    pub fn repr(&self) -> String {
        format!(
            "NurbsSurfaceTrimmed(\n  name={},\n  trimmed={},\n  holes={},\n  surface={}\n)",
            self.name,
            self.is_trimmed(),
            self.inner_loop_count(),
            self.m_surface.str()
        )
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Private helpers
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the diagonal of the control-point box, the scale every deflection tolerance is a fraction of.
    fn bbox_diagonal(&self) -> f64 {
        let mut bmin = [1e30f64; 3];
        let mut bmax = [-1e30f64; 3];

        for i in 0..self.m_surface.cv_count(0) {
            for j in 0..self.m_surface.cv_count(1) {
                let p = self.m_surface.get_cv(i, j).unwrap_or_default();

                for k in 0..3 {
                    if p[k] < bmin[k] {
                        bmin[k] = p[k];
                    }

                    if p[k] > bmax[k] {
                        bmax[k] = p[k];
                    }
                }
            }
        }

        let bbox_diag = ((bmax[0] - bmin[0]) * (bmax[0] - bmin[0])
            + (bmax[1] - bmin[1]) * (bmax[1] - bmin[1])
            + (bmax[2] - bmin[2]) * (bmax[2] - bmin[2]))
            .sqrt();

        if bbox_diag < 1e-12 {
            1.0
        } else {
            bbox_diag
        }
    }

    /// UV polygon of a trim loop: control points or samples, each edge split until its 3D chord is within deflection.
    fn discretize_loop(&self, crv: &NurbsCurve, deflection: f64) -> Vec<Point> {
        let raw = loop_points(crv);

        if raw.len() < 2 {
            return raw;
        }

        let mut out: Vec<Point> = Vec::with_capacity(raw.len() * 2);

        for i in 0..raw.len() {
            subdivide_edge(
                &self.m_surface,
                &raw[i],
                &raw[(i + 1) % raw.len()],
                deflection,
                &mut out,
            );
        }

        out
    }

    /// Constrained Delaunay of the loops in UV, refined, trimmed, lifted and welded: the one body mesh_q and mesh_loops share.
    fn triangulate(&self, loops: &TrimLoops, max_angle_deg: f64, chord_factor: f64) -> Mesh {
        if loops.uv.is_empty() || loops.uv[0].len() < 3 {
            return self.m_surface.mesh();
        }

        let bbox_diag = self.bbox_diagonal();
        let deflection = bbox_diag * chord_factor;
        let cos_max_angle = (max_angle_deg.clamp(0.1, 179.0) * PI / 180.0).cos();
        let crease_knots = find_crease_knots(&self.m_surface);
        let bounds = loop_bounds(&loops.uv[0]);

        let mut dt = Delaunay2D::new(bounds[0], bounds[1], bounds[2], bounds[3]);
        let mut boundary_intervals: BTreeMap<usize, (usize, usize, f64)> = BTreeMap::new();
        let loop_vids = insert_loops(&mut dt, &loops.uv, &crease_knots, &mut boundary_intervals);
        insert_crease_lines(&mut dt, &loops.uv, &crease_knots);

        for p in &loops.interior_uv {
            if inside_loops(p[0], p[1], &loops.uv) {
                dt.insert(p[0], p[1]);
            }
        }

        refine(
            &mut dt,
            &self.m_surface,
            &loops.uv,
            &crease_knots,
            deflection,
            cos_max_angle,
        );
        trim_outside(&mut dt, &loops.uv);
        let tris = dt.get_triangles();

        if tris.is_empty() || crosses_crease(&tris, &dt, &crease_knots) {
            return Mesh::new();
        }

        let mut result = Mesh::new();
        let weld_tol = if loops.xyz.is_empty() {
            bbox_diag * 1e-5
        } else {
            0.0
        };
        let mut welder = VertexWelder::new(weld_tol, bbox_diag * 1e-5);
        let vert_map = weld_vertices(
            &mut welder,
            &mut result,
            &self.m_surface,
            &dt,
            &tris,
            loops,
            &loop_vids,
            &boundary_intervals,
        );
        add_faces(&mut result, &tris, &vert_map);
        set_vertex_normals(&mut result, &self.m_surface, &dt, &vert_map);
        tag_boundary(&mut result, &loop_vids, &boundary_intervals, &vert_map);
        RemeshNurbsSurfaceGrid::split_crease_normals(&self.m_surface, &mut result);

        result
    }
}

impl std::fmt::Display for NurbsSurfaceTrimmed {
    /// Stream the str() form.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
impl PartialEq for NurbsSurfaceTrimmed {
    /// Compare name, width, color, surface and trim loops; guid ignored.
    fn eq(&self, other: &Self) -> bool {
        if self.name != other.name {
            return false;
        }

        if self.width != other.width {
            return false;
        }

        if self.surfacecolor != other.surfacecolor {
            return false;
        }

        if self.m_surface != other.m_surface {
            return false;
        }

        if self.m_outer_loop != other.m_outer_loop {
            return false;
        }

        self.m_inner_loops == other.m_inner_loops
    }
}
