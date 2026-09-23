use std::collections::HashMap;
use std::collections::HashSet;

use crate::mesh::Mesh;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
use crate::tolerance::Tolerance;
use crate::vector::Vector;

/// Grid mesh of a NURBS surface: spans split by normal turn and chord height, poles fanned, seams closed.
pub struct RemeshNurbsSurfaceGrid;

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════
const MAX_SUBS: usize = 24;

// ═══════════════════════════════════════════════════════════════════════════
// Sampling
// ═══════════════════════════════════════════════════════════════════════════

/// Euclidean length without the zero gate of magnitude().
fn norm(v: &Vector) -> f64 {
    v.magnitude_squared().sqrt()
}

/// Surface point at t along dir with the other parameter fixed.
fn point_along(s: &NurbsSurface, dir: usize, t: f64, fixed: f64) -> Point {
    if dir == 0 {
        s.point_at(t, fixed).unwrap_or_default()
    } else {
        s.point_at(fixed, t).unwrap_or_default()
    }
}

/// Surface normal at t along dir with the other parameter fixed.
fn normal_along(s: &NurbsSurface, dir: usize, t: f64, fixed: f64) -> Vector {
    if dir == 0 {
        s.normal_at(t, fixed)
    } else {
        s.normal_at(fixed, t)
    }
}

/// Sv x Su unnormalized, zero when the surface cannot be evaluated; normal_at would give a +Z sentinel at a pole.
fn raw_normal(s: &NurbsSurface, u: f64, v: f64) -> Vector {
    let derivatives = s.evaluate(u, v, 1);

    if derivatives.len() < 3 {
        return Vector::new(0.0, 0.0, 0.0);
    }

    derivatives[2].cross(&derivatives[1])
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

// ═══════════════════════════════════════════════════════════════════════════
// Subdivisions
// ═══════════════════════════════════════════════════════════════════════════

/// Largest turn of the unit normal in degrees over [t0, t1], sampled at the span midpoints of the other direction.
fn span_angle(s: &NurbsSurface, dir: usize, t0: f64, t1: f64, osp: &[f64]) -> f64 {
    let mut max_angle = 0.0_f64;

    for si in 0..osp.len() - 1 {
        let fixed = (osp[si] + osp[si + 1]) * 0.5;

        let mut first = Vector::new(0.0, 0.0, 0.0);
        let mut last = Vector::new(0.0, 0.0, 0.0);
        let mut has_first = false;

        for k in 0..=4 {
            let n = normal_along(s, dir, t0 + k as f64 * (t1 - t0) / 4.0, fixed);
            let length = norm(&n);

            if length < 1e-10 {
                continue;
            }

            let unit = n / length;

            if !has_first {
                first = unit.clone();
            }

            has_first = true;
            last = unit;
        }

        if !has_first {
            continue;
        }

        let dot = first.dot(&last).clamp(-1.0, 1.0);

        max_angle = max_angle.max(dot.acos() * 180.0 / Tolerance::PI);
    }

    max_angle
}

/// Largest height of [t0, t1] over its chord, at up to four positions across the other direction.
fn span_deviation(s: &NurbsSurface, dir: usize, t0: f64, t1: f64, osp: &[f64]) -> f64 {
    let mut max_dev = 0.0_f64;
    let nc = (osp.len() - 1).min(3);

    for ci in 0..=nc {
        let fixed = osp[0] + ci as f64 * (osp[osp.len() - 1] - osp[0]) / nc.max(1) as f64;
        let p0 = point_along(s, dir, t0, fixed);
        let p1 = point_along(s, dir, t1, fixed);

        for k in 1..=3 {
            let frac = k as f64 / 4.0;
            let pm = point_along(s, dir, t0 + frac * (t1 - t0), fixed);

            max_dev = max_dev.max(norm(&(pm - (&p0 + (&p1 - &p0) * frac))));
        }
    }

    max_dev
}

/// Subdivisions per span along dir: the normal turn against max_angle_deg, the chord height against chord_tol, at least two on a curved span.
fn span_subs(
    s: &NurbsSurface,
    dir: usize,
    sp: &[f64],
    osp: &[f64],
    max_angle_deg: f64,
    chord_tol: f64,
) -> Vec<usize> {
    let degree = s.degree(dir);
    let mut subs = vec![1; sp.len() - 1];

    for i in 0..sp.len() - 1 {
        if degree > 1 {
            let angle = span_angle(s, dir, sp[i], sp[i + 1], osp);

            subs[i] = ((angle / max_angle_deg).ceil() as usize).clamp(1, MAX_SUBS);
        }

        let dev = span_deviation(s, dir, sp[i], sp[i + 1], osp);

        if dev > chord_tol {
            subs[i] = subs[i].max(((dev / chord_tol).sqrt().ceil() as usize).clamp(2, MAX_SUBS));
        }

        if degree > 1 {
            subs[i] = subs[i].max(2);
        }
    }

    subs
}

/// Length of the iso-curve at fixed along dir as a polyline of n steps.
fn isocurve_length(s: &NurbsSurface, dir: usize, sp: &[f64], fixed: f64, n: usize) -> f64 {
    let mut length = 0.0;
    let mut prev = point_along(s, dir, sp[0], fixed);

    for i in 1..=n {
        let next = point_along(
            s,
            dir,
            sp[0] + i as f64 * (sp[sp.len() - 1] - sp[0]) / n as f64,
            fixed,
        );

        length += norm(&(&next - &prev));
        prev = next;
    }

    length
}

/// Scale up the curved direction whose spacing is more than twice the other's.
fn balance_subs(
    s: &NurbsSurface,
    usp: &[f64],
    vsp: &[f64],
    u_subs: &mut [usize],
    v_subs: &mut [usize],
) {
    let mut total_u = 1;
    let mut total_v = 1;

    for sub in u_subs.iter() {
        total_u += sub;
    }

    for sub in v_subs.iter() {
        total_v += sub;
    }

    let u_len = isocurve_length(
        s,
        0,
        usp,
        (vsp[0] + vsp[vsp.len() - 1]) * 0.5,
        total_u.max(10),
    );
    let v_len = isocurve_length(
        s,
        1,
        vsp,
        (usp[0] + usp[usp.len() - 1]) * 0.5,
        total_v.max(10),
    );

    if u_len <= 1e-14 || v_len <= 1e-14 {
        return;
    }

    let ratio = (u_len / total_u as f64) / (v_len / total_v as f64);

    if ratio > 2.0 && s.degree(0) > 1 {
        let scale = ratio.sqrt();

        for sub in u_subs.iter_mut() {
            *sub = MAX_SUBS.min((*sub as f64 * scale).ceil() as usize);
        }
    } else if ratio < 0.5 && s.degree(1) > 1 {
        let scale = (1.0 / ratio).sqrt();

        for sub in v_subs.iter_mut() {
            *sub = MAX_SUBS.min((*sub as f64 * scale).ceil() as usize);
        }
    }
}

/// Subdivisions both directions of a bilinear surface need for its twist, 1 when every span centre lies within twist_tol of its diagonal midpoint.
fn twist_subs(s: &NurbsSurface, usp: &[f64], vsp: &[f64], twist_tol: f64) -> usize {
    let mut max_twist = 0.0_f64;

    for i in 0..usp.len() - 1 {
        for j in 0..vsp.len() - 1 {
            let pm = s
                .point_at((usp[i] + usp[i + 1]) * 0.5, (vsp[j] + vsp[j + 1]) * 0.5)
                .unwrap_or_default();
            let p00 = s.point_at(usp[i], vsp[j]).unwrap_or_default();
            let p11 = s.point_at(usp[i + 1], vsp[j + 1]).unwrap_or_default();

            max_twist = max_twist.max(norm(&(pm - Point::sum(&p00, &p11) * 0.5)));
        }
    }

    if max_twist <= twist_tol {
        return 1;
    }

    ((2.0 * (max_twist / twist_tol).sqrt()).ceil() as usize).clamp(4, MAX_SUBS)
}

/// One more subdivision on the largest span when the total is even, so a closed direction triangulates seamlessly.
fn make_odd(subs: &mut [usize]) {
    let mut total = 0;

    for sub in subs.iter() {
        total += sub;
    }

    if total % 2 != 0 {
        return;
    }

    let mut largest = 0;

    for i in 1..subs.len() {
        if subs[i] > subs[largest] {
            largest = i;
        }
    }

    subs[largest] += 1;
}

// ═══════════════════════════════════════════════════════════════════════════
// Parameters
// ═══════════════════════════════════════════════════════════════════════════

/// n parameters spaced evenly by arc length along the iso-curve at fixed.
fn arclen_params(s: &NurbsSurface, dir: usize, n: usize, sp: &[f64], fixed: f64) -> Vec<f64> {
    let nsample = (n * 20).max(200);

    let mut st = vec![0.0; nsample + 1];
    let mut sl = vec![0.0; nsample + 1];
    let mut prev = point_along(s, dir, sp[0], fixed);

    for k in 0..=nsample {
        st[k] = sp[0] + k as f64 * (sp[sp.len() - 1] - sp[0]) / nsample as f64;

        if k == 0 {
            continue;
        }

        let next = point_along(s, dir, st[k], fixed);

        sl[k] = sl[k - 1] + norm(&(&next - &prev));
        prev = next;
    }

    let mut params = vec![sp[0]];
    let mut j = 0;

    for i in 1..n - 1 {
        let target = sl[nsample] * i as f64 / (n - 1) as f64;

        while j < nsample && sl[j] < target {
            j += 1;
        }

        let a = if j > 0 { j - 1 } else { 0 };
        let frac = if sl[j] > sl[a] {
            (target - sl[a]) / (sl[j] - sl[a])
        } else {
            0.0
        };

        params.push(st[a] + frac * (st[j] - st[a]));
    }

    params.push(sp[sp.len() - 1]);

    params
}

/// Every span split into its subdivisions, ending on the last span boundary.
fn span_params(sp: &[f64], subs: &[usize]) -> Vec<f64> {
    let mut params = Vec::new();

    for i in 0..sp.len() - 1 {
        for sub in 0..subs[i] {
            params.push(sp[i] + sub as f64 * (sp[i + 1] - sp[i]) / subs[i] as f64);
        }
    }

    params.push(sp[sp.len() - 1]);

    params
}

/// Closed direction: drop the duplicate end and fill a wrap gap wider than 1.5 times the largest step.
fn fix_closed_gap(params: &mut Vec<f64>, domain_end: f64) {
    if params.len() < 3 {
        return;
    }

    params.pop();

    let wrap_gap = domain_end - params[params.len() - 1];
    let mut max_gap = 0.0_f64;

    for i in 1..params.len() {
        max_gap = max_gap.max(params[i] - params[i - 1]);
    }

    if max_gap <= 0.0 || wrap_gap <= max_gap * 1.5 {
        return;
    }

    let extra = (wrap_gap / max_gap).ceil() as usize - 1;
    let step = wrap_gap / (extra + 1) as f64;

    for _ in 1..=extra {
        params.push(params[params.len() - 1] + step);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Vertices and faces
// ═══════════════════════════════════════════════════════════════════════════

/// Vertex at S(u, v) tagged with its parameters.
fn add_vertex_uv(s: &NurbsSurface, mesh: &mut Mesh, u: f64, v: f64) -> usize {
    let key = mesh.add_vertex(s.point_at(u, v).unwrap_or_default(), None);
    let vd = mesh.vertex.get_mut(&key).unwrap();

    vd.attributes.insert("u".to_string(), u);
    vd.attributes.insert("v".to_string(), v);

    key
}

/// Grid vertices row by row over us and the rows j_start..j_end of vs.
fn add_grid(
    s: &NurbsSurface,
    mesh: &mut Mesh,
    us: &[f64],
    vs: &[f64],
    j_start: usize,
    j_end: usize,
) -> Vec<usize> {
    let mut grid = Vec::new();

    for &u in us {
        for &v in &vs[j_start..j_end] {
            grid.push(add_vertex_uv(s, mesh, u, v));
        }
    }

    grid
}

/// Fans from the south pole, checkerboard-split quads, fans to the north pole.
fn add_faces(
    mesh: &mut Mesh,
    grid: &[usize],
    nu: usize,
    closed_u: bool,
    wrap_v: bool,
    south: Option<usize>,
    north: Option<usize>,
) {
    let nv = grid.len() / nu;
    let nu_faces = if closed_u { nu } else { nu - 1 };
    let nv_faces = if wrap_v { nv } else { nv - 1 };

    if let Some(south) = south {
        for i in 0..nu_faces {
            mesh.add_face(vec![south, grid[((i + 1) % nu) * nv], grid[i * nv]], None);
        }
    }

    for i in 0..nu_faces {
        for j in 0..nv_faces {
            let i1 = (i + 1) % nu;
            let j1 = (j + 1) % nv;
            let v00 = grid[i * nv + j];
            let v10 = grid[i1 * nv + j];
            let v01 = grid[i * nv + j1];
            let v11 = grid[i1 * nv + j1];

            if (i + j) % 2 == 0 {
                mesh.add_face(vec![v00, v10, v11], None);
                mesh.add_face(vec![v00, v11, v01], None);
            } else {
                mesh.add_face(vec![v00, v10, v01], None);
                mesh.add_face(vec![v10, v11, v01], None);
            }
        }
    }

    if let Some(north) = north {
        for i in 0..nu_faces {
            mesh.add_face(
                vec![
                    grid[i * nv + nv - 1],
                    grid[((i + 1) % nu) * nv + nv - 1],
                    north,
                ],
                None,
            );
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

        if vertices.len() < 3 {
            continue;
        }

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

/// Unit surface normal on the side of the fan normal; the fan normal at the poles and where the surface normal vanishes, +Z when the fan vanishes too.
fn set_normals(s: &NurbsSurface, mesh: &mut Mesh, south: Option<usize>, north: Option<usize>) {
    let sums = fan_normals(mesh);

    for (&key, vd) in mesh.vertex.iter_mut() {
        let fan_length = norm(&sums[key]);

        let mut n = Vector::new(0.0, 0.0, 1.0);

        if fan_length.is_finite() && fan_length > 0.0 {
            n = &sums[key] / fan_length;
        }

        if Some(key) != south && Some(key) != north {
            let raw = raw_normal(
                s,
                *vd.attributes.get("u").unwrap(),
                *vd.attributes.get("v").unwrap(),
            );
            let length = norm(&raw);

            if length.is_finite() && length > 0.0 {
                n = if raw.dot(&n) < 0.0 {
                    -raw / length
                } else {
                    raw / length
                };
            }
        }

        vd.set_normal(n[0], n[1], n[2]);
    }
}

/// Bit per direction where (u, v) sits on an internal knot of full multiplicity whose one-sided normals disagree.
fn crease_flags(s: &NurbsSurface, u: f64, v: f64) -> u32 {
    let uv = [u, v];
    let mut flags = 0;

    for dir in 0..2 {
        let domain = s.domain(dir).unwrap_or_default();
        let value = uv[dir];

        if value <= domain.0 || value >= domain.1 {
            continue;
        }

        let mut multiplicity = 0;

        for &knot in &s.m_nurbsknot[dir] {
            if knot == value {
                multiplicity += 1;
            }
        }

        if multiplicity < s.degree(dir) {
            continue;
        }

        let mut lo = [u, v];
        let mut hi = [u, v];

        lo[dir] = value.next_down();
        hi[dir] = value.next_up();

        let a = s.normal_at(lo[0], lo[1]);
        let b = s.normal_at(hi[0], hi[1]);
        let length = (a.magnitude_squared() * b.magnitude_squared()).sqrt();

        if length == 0.0 {
            continue;
        }

        let dot = a.dot(&b) / length;

        if dot.is_finite() && dot < 1.0 - 64.0 * f64::EPSILON {
            flags |= 1 << dir;
        }
    }

    flags
}

/// Nudge uv one ulp toward center in each flagged direction; bit per direction nudged upward.
fn crease_side(center: &[f64; 2], uv: &mut [f64; 2], flags: u32) -> u32 {
    let mut side = 0;

    for dir in 0..2 {
        if flags & (1 << dir) == 0 {
            continue;
        }

        let high = center[dir] > uv[dir];

        if high {
            side |= 1 << dir;
        }

        uv[dir] = if high {
            uv[dir].next_up()
        } else {
            uv[dir].next_down()
        };
    }

    side
}

/// Vertex carrying a corner: the original the first time its key is met, then one copy per (key, side).
fn crease_target(
    mesh: &mut Mesh,
    copies: &mut HashMap<(usize, u32), usize>,
    used: &mut HashSet<usize>,
    key: usize,
    side: u32,
) -> usize {
    let identity = (key, side);

    if let Some(&target) = copies.get(&identity) {
        return target;
    }

    if used.insert(key) {
        copies.insert(identity, key);

        return key;
    }

    let target = mesh.add_vertex(mesh.vertex[&key].position(), None);

    mesh.vertex.insert(target, mesh.vertex[&key].clone());
    copies.insert(identity, target);

    target
}

impl RemeshNurbsSurfaceGrid {
    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Grid at 20 degrees and 0.5 percent of the bbox diagonal; max_u and max_v fix the parameter counts when positive.
    pub fn from_u_v(s: NurbsSurface, max_u: usize, max_v: usize) -> Mesh {
        Self::from_u_v_q(s, max_u, max_v, 20.0, 0.005)
    }

    /// Grid with the normal turn per subdivision capped at max_angle_deg and the chord height at chord_factor of the bbox diagonal; vertex normals are unit surface normals on the fan side, fan normals at poles.
    pub fn from_u_v_q(
        s: NurbsSurface,
        max_u: usize,
        max_v: usize,
        max_angle_deg: f64,
        chord_factor: f64,
    ) -> Mesh {
        let usp = s.get_span_vector(0);
        let vsp = s.get_span_vector(1);
        let bbox_diag = bbox_diagonal(&s);
        let chord_tol = bbox_diag * chord_factor;

        let mut u_subs = span_subs(&s, 0, &usp, &vsp, max_angle_deg, chord_tol);
        let mut v_subs = span_subs(&s, 1, &vsp, &usp, max_angle_deg, chord_tol);

        balance_subs(&s, &usp, &vsp, &mut u_subs, &mut v_subs);

        let sing_v0 = s.is_singular(0);
        let sing_v1 = s.is_singular(2);

        if s.degree(0) == 1 && s.degree(1) == 1 && !sing_v0 && !sing_v1 {
            let twist = twist_subs(
                &s,
                &usp,
                &vsp,
                if bbox_diag > 0.0 { chord_tol } else { 1e-6 },
            );

            for sub in u_subs.iter_mut() {
                *sub = (*sub).max(twist);
            }

            for sub in v_subs.iter_mut() {
                *sub = (*sub).max(twist);
            }
        }

        let closed_u = s.is_closed(0);
        let closed_v = s.is_closed(1);

        if closed_u && max_u == 0 {
            make_odd(&mut u_subs);
        }

        if closed_v && max_v == 0 {
            make_odd(&mut v_subs);
        }

        let u_mid = (usp[0] + usp[usp.len() - 1]) * 0.5;
        let v_mid = (vsp[0] + vsp[vsp.len() - 1]) * 0.5;

        let mut us = if max_u > 0 {
            arclen_params(&s, 0, max_u.max(2), &usp, v_mid)
        } else {
            span_params(&usp, &u_subs)
        };
        let mut vs = if max_v > 0 {
            arclen_params(&s, 1, max_v.max(2), &vsp, u_mid)
        } else {
            span_params(&vsp, &v_subs)
        };

        if closed_u {
            fix_closed_gap(&mut us, usp[usp.len() - 1]);
        }

        if closed_v {
            fix_closed_gap(&mut vs, vsp[vsp.len() - 1]);
        }

        let nv = vs.len();

        let mut mesh = Mesh::new();
        let mut south = None;
        let mut north = None;

        if sing_v0 {
            south = Some(add_vertex_uv(&s, &mut mesh, us[0], vs[0]));
        }

        if sing_v1 {
            north = Some(add_vertex_uv(&s, &mut mesh, us[0], vs[nv - 1]));
        }

        let grid = add_grid(
            &s,
            &mut mesh,
            &us,
            &vs,
            if sing_v0 { 1 } else { 0 },
            if sing_v1 { nv - 1 } else { nv },
        );

        add_faces(
            &mut mesh,
            &grid,
            us.len(),
            closed_u,
            closed_v && !sing_v0 && !sing_v1,
            south,
            north,
        );
        set_normals(&s, &mut mesh, south, north);
        Self::split_crease_normals(&s, &mut mesh);

        mesh
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Normals
    // ═══════════════════════════════════════════════════════════════════════════
    /// Split shading vertices at internal C0 knots whose one-sided normals disagree.
    pub(crate) fn split_crease_normals(s: &NurbsSurface, mesh: &mut Mesh) {
        let mut candidates = HashMap::<usize, u32>::new();

        for (&key, vd) in &mesh.vertex {
            let (Some(&u), Some(&v)) = (vd.attributes.get("u"), vd.attributes.get("v")) else {
                continue;
            };

            let flags = crease_flags(s, u, v);

            if flags != 0 {
                candidates.insert(key, flags);
            }
        }

        if candidates.is_empty() {
            return;
        }

        let mut copies = HashMap::<(usize, u32), usize>::new();
        let mut used = HashSet::<usize>::new();

        let mut face_keys: Vec<usize> = mesh.face.keys().copied().collect();
        face_keys.sort_unstable();

        for face_key in face_keys {
            let mut vertices = mesh.face[&face_key].clone();
            let mut center = [0.0, 0.0];

            for &key in &vertices {
                center[0] += mesh.vertex[&key].attributes.get("u").unwrap();
                center[1] += mesh.vertex[&key].attributes.get("v").unwrap();
            }

            center[0] /= vertices.len() as f64;
            center[1] /= vertices.len() as f64;

            let face_normal = mesh.face_normal(face_key);

            for slot in vertices.iter_mut() {
                let key = *slot;
                let Some(&flags) = candidates.get(&key) else {
                    continue;
                };

                let mut uv = [
                    *mesh.vertex[&key].attributes.get("u").unwrap(),
                    *mesh.vertex[&key].attributes.get("v").unwrap(),
                ];

                let side = crease_side(&center, &mut uv, flags);
                let target = crease_target(mesh, &mut copies, &mut used, key, side);
                let n = s.normal_at(uv[0], uv[1]);
                let length = norm(&n);

                if length.is_finite() && length > 0.0 {
                    let sign = match &face_normal {
                        Some(normal) if n.dot(normal) < 0.0 => -1.0,
                        _ => 1.0,
                    };

                    mesh.vertex.get_mut(&target).unwrap().set_normal(
                        sign * n[0] / length,
                        sign * n[1] / length,
                        sign * n[2] / length,
                    );
                }

                *slot = target;
            }

            mesh.face.insert(face_key, vertices);
        }

        mesh.rebuild_halfedges();
    }
}
