use std::collections::HashMap;

use crate::mesh::Mesh;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
use crate::tolerance::Tolerance;

pub struct RemeshNurbsSurfaceGrid;

impl RemeshNurbsSurfaceGrid {
    pub fn from_u_v(s: NurbsSurface, max_u: usize, max_v: usize) -> Mesh {
        const MAX_ANGLE: f64 = 20.0;
        let usp = s.get_span_vector(0);
        let vsp = s.get_span_vector(1);
        let ns_u = usp.len() - 1;
        let ns_v = vsp.len() - 1;
        let deg_u = s.degree(0);
        let deg_v = s.degree(1);

        let (mut minx, mut miny, mut minz) = (1e30_f64, 1e30_f64, 1e30_f64);
        let (mut maxx, mut maxy, mut maxz) = (-1e30_f64, -1e30_f64, -1e30_f64);
        for i in 0..s.cv_count_dir(Some(0)) {
            for j in 0..s.cv_count_dir(Some(1)) {
                if let Some(p) = s.get_cv(i, j) {
                    if p[0] < minx { minx = p[0]; }
                    if p[1] < miny { miny = p[1]; }
                    if p[2] < minz { minz = p[2]; }
                    if p[0] > maxx { maxx = p[0]; }
                    if p[1] > maxy { maxy = p[1]; }
                    if p[2] > maxz { maxz = p[2]; }
                }
            }
        }
        let dx = maxx - minx;
        let dy = maxy - miny;
        let dz = maxz - minz;
        let bbox_diag = (dx*dx + dy*dy + dz*dz).sqrt();

        let span_subs = |dir: usize, sp: &[f64], osp: &[f64]| -> Vec<usize> {
            let n = sp.len() - 1;
            let mut subs = vec![1usize; n];
            let n_other = osp.len() - 1;
            let s_positions: Vec<f64> = (0..n_other).map(|k| (osp[k] + osp[k + 1]) * 0.5).collect();
            let degree_dir = if dir == 0 { deg_u } else { deg_v };
            for i in 0..n {
                let t0 = sp[i];
                let t1 = sp[i + 1];
                if degree_dir > 1 {
                    let mut max_angle = 0.0_f64;
                    for si in 0..n_other {
                        let sv = s_positions[si];
                        let mut fn3 = [0.0_f64; 3];
                        let mut ln3 = [0.0_f64; 3];
                        let mut has_first = false;
                        for k in 0..=4 {
                            let t = t0 + k as f64 * (t1 - t0) / 4.0;
                            let nrm = if dir == 0 { s.normal_at(t, sv) } else { s.normal_at(sv, t) };
                            let (nx, ny, nz) = (nrm[0], nrm[1], nrm[2]);
                            let len = (nx*nx + ny*ny + nz*nz).sqrt();
                            if len < 1e-10 { continue; }
                            let (nx, ny, nz) = (nx/len, ny/len, nz/len);
                            if !has_first { fn3 = [nx, ny, nz]; has_first = true; }
                            ln3 = [nx, ny, nz];
                        }
                        let total_angle = if has_first {
                            let dot = (fn3[0]*ln3[0] + fn3[1]*ln3[1] + fn3[2]*ln3[2]).clamp(-1.0, 1.0);
                            dot.acos() * 180.0 / Tolerance::PI
                        } else { 0.0 };
                        if total_angle > max_angle { max_angle = total_angle; }
                    }
                    subs[i] = ((max_angle / MAX_ANGLE).ceil() as usize).clamp(1, 24);
                }

                let chord_tol = bbox_diag * 0.005;
                let mut max_dev = 0.0_f64;
                let nc = n_other.min(3);
                for ci in 0..=nc {
                    let sv = osp[0] + ci as f64 * (osp[osp.len()-1] - osp[0]) / nc.max(1) as f64;
                    let (p0, p1) = if dir == 0 {
                        (s.point_at(t0, sv), s.point_at(t1, sv))
                    } else {
                        (s.point_at(sv, t0), s.point_at(sv, t1))
                    };
                    if let (Some(p0), Some(p1)) = (p0, p1) {
                        let (px0, py0, pz0) = (p0[0], p0[1], p0[2]);
                        let (px1, py1, pz1) = (p1[0], p1[1], p1[2]);
                        for k in 1..=3 {
                            let frac = k as f64 / 4.0;
                            let tm = t0 + frac * (t1 - t0);
                            let pm = if dir == 0 { s.point_at(tm, sv) } else { s.point_at(sv, tm) };
                            if let Some(pm) = pm {
                                let lx = px0 + frac * (px1 - px0);
                                let ly = py0 + frac * (py1 - py0);
                                let lz = pz0 + frac * (pz1 - pz0);
                                let ddx = pm[0] - lx;
                                let ddy = pm[1] - ly;
                                let ddz = pm[2] - lz;
                                let dev = (ddx*ddx + ddy*ddy + ddz*ddz).sqrt();
                                if dev > max_dev { max_dev = dev; }
                            }
                        }
                    }
                }
                if max_dev > chord_tol {
                    let chord_subs = ((max_dev / chord_tol).sqrt().ceil() as usize).clamp(2, 24);
                    if chord_subs > subs[i] { subs[i] = chord_subs; }
                }

                if degree_dir > 1 && subs[i] < 2 { subs[i] = 2; }
            }
            subs
        };

        let mut u_subs = span_subs(0, &usp, &vsp);
        let mut v_subs = span_subs(1, &vsp, &usp);

        // Arc-length aspect ratio balancing
        {
            let total_u = u_subs.iter().sum::<usize>() + 1;
            let total_v = v_subs.iter().sum::<usize>() + 1;
            let v_mid = (vsp[0] + vsp[vsp.len()-1]) * 0.5;
            let u_mid = (usp[0] + usp[usp.len()-1]) * 0.5;
            let mut u_len = 0.0_f64;
            let n_sample_u = total_u.max(10);
            if let Some(p0) = s.point_at(usp[0], v_mid) {
                let mut prev = (p0[0], p0[1], p0[2]);
                for i in 1..=n_sample_u {
                    let u = usp[0] + i as f64 * (usp[usp.len()-1] - usp[0]) / n_sample_u as f64;
                    if let Some(p1) = s.point_at(u, v_mid) {
                        let ddx = p1[0] - prev.0;
                        let ddy = p1[1] - prev.1;
                        let ddz = p1[2] - prev.2;
                        u_len += (ddx*ddx + ddy*ddy + ddz*ddz).sqrt();
                        prev = (p1[0], p1[1], p1[2]);
                    }
                }
            }
            let mut v_len = 0.0_f64;
            let n_sample_v = total_v.max(10);
            if let Some(p0) = s.point_at(u_mid, vsp[0]) {
                let mut prev = (p0[0], p0[1], p0[2]);
                for i in 1..=n_sample_v {
                    let v = vsp[0] + i as f64 * (vsp[vsp.len()-1] - vsp[0]) / n_sample_v as f64;
                    if let Some(p1) = s.point_at(u_mid, v) {
                        let ddx = p1[0] - prev.0;
                        let ddy = p1[1] - prev.1;
                        let ddz = p1[2] - prev.2;
                        v_len += (ddx*ddx + ddy*ddy + ddz*ddz).sqrt();
                        prev = (p1[0], p1[1], p1[2]);
                    }
                }
            }
            if u_len > 1e-14 && v_len > 1e-14 && total_u > 0 && total_v > 0 {
                let spacing_u = u_len / total_u as f64;
                let spacing_v = v_len / total_v as f64;
                let ratio = spacing_u / spacing_v;
                if ratio > 2.0 && deg_u > 1 {
                    let scale = ratio.sqrt();
                    for sv in &mut u_subs {
                        *sv = ((*sv as f64 * scale).ceil() as usize).min(24);
                    }
                } else if ratio < 0.5 && deg_v > 1 {
                    let scale = (1.0 / ratio).sqrt();
                    for sv in &mut v_subs {
                        *sv = ((*sv as f64 * scale).ceil() as usize).min(24);
                    }
                }
            }
        }

        // Bilinear twist check (skip for singular surfaces — fan triangulation handles those)
        if deg_u == 1 && deg_v == 1 && !s.is_singular(0) && !s.is_singular(2) {
            let chord_tol = if bbox_diag > 0.0 { bbox_diag * 0.005 } else { 1e-6 };
            let mut max_twist = 0.0_f64;
            for i in 0..ns_u {
                for j in 0..ns_v {
                    let u0 = usp[i]; let u1 = usp[i + 1];
                    let v0 = vsp[j]; let v1 = vsp[j + 1];
                    let pm = s.point_at((u0 + u1) * 0.5, (v0 + v1) * 0.5);
                    let p00 = s.point_at(u0, v0);
                    let p11 = s.point_at(u1, v1);
                    if let (Some(pm), Some(p00), Some(p11)) = (pm, p00, p11) {
                        let mx = (p00[0] + p11[0]) * 0.5;
                        let my = (p00[1] + p11[1]) * 0.5;
                        let mz = (p00[2] + p11[2]) * 0.5;
                        let ddx = pm[0] - mx;
                        let ddy = pm[1] - my;
                        let ddz = pm[2] - mz;
                        let twist = (ddx*ddx + ddy*ddy + ddz*ddz).sqrt();
                        if twist > max_twist { max_twist = twist; }
                    }
                }
            }
            if max_twist > chord_tol {
                let twist_subs = ((2.0 * (max_twist / chord_tol).sqrt()).ceil() as usize).clamp(4, 24);
                for sv in &mut u_subs { if *sv < twist_subs { *sv = twist_subs; } }
                for sv in &mut v_subs { if *sv < twist_subs { *sv = twist_subs; } }
            }
        }

        let closed_u = s.is_closed(0);
        let closed_v = s.is_closed(1);

        // Ensure odd total subdivisions for closed directions (seamless checkerboard triangulation)
        if closed_u && max_u == 0 {
            let total: usize = u_subs.iter().sum();
            if total % 2 == 0 {
                let m = (0..u_subs.len()).max_by_key(|&i| u_subs[i]).unwrap_or(0);
                u_subs[m] += 1;
            }
        }
        if closed_v && max_v == 0 {
            let total: usize = v_subs.iter().sum();
            if total % 2 == 0 {
                let m = (0..v_subs.len()).max_by_key(|&i| v_subs[i]).unwrap_or(0);
                v_subs[m] += 1;
            }
        }

        let v_mid = (vsp[0] + vsp[vsp.len()-1]) * 0.5;
        let u_mid = (usp[0] + usp[usp.len()-1]) * 0.5;

        let arclen_params = |n: usize, sp: &[f64], fixed: f64, is_u: bool| -> Vec<f64> {
            let nsample = (n * 20).max(200);
            let st: Vec<f64> = (0..=nsample).map(|k| sp[0] + k as f64 * (sp[sp.len()-1] - sp[0]) / nsample as f64).collect();
            let mut sl = vec![0.0_f64; nsample + 1];
            let p0 = if is_u { s.point_at(sp[0], fixed) } else { s.point_at(fixed, sp[0]) };
            let mut prev = p0.map(|p| (p[0], p[1], p[2])).unwrap_or((0.0, 0.0, 0.0));
            for k in 1..=nsample {
                let p1 = if is_u { s.point_at(st[k], fixed) } else { s.point_at(fixed, st[k]) };
                if let Some(p1) = p1 {
                    let d = ((p1[0]-prev.0).powi(2) + (p1[1]-prev.1).powi(2) + (p1[2]-prev.2).powi(2)).sqrt();
                    sl[k] = sl[k-1] + d;
                    prev = (p1[0], p1[1], p1[2]);
                } else {
                    sl[k] = sl[k-1];
                }
            }
            let total_len = sl[nsample];
            let mut params = vec![sp[0]];
            let mut j = 0usize;
            for i in 1..(n - 1) {
                let target = total_len * i as f64 / (n - 1) as f64;
                while j < nsample && sl[j] < target { j += 1; }
                let ta = if j > 0 { st[j-1] } else { st[0] };
                let tb = st[j];
                let la = if j > 0 { sl[j-1] } else { sl[0] };
                let lb = sl[j];
                let frac = if lb > la { (target - la) / (lb - la) } else { 0.0 };
                params.push(ta + frac * (tb - ta));
            }
            params.push(*sp.last().unwrap());
            params
        };

        // Build parameter arrays
        let us: Vec<f64> = if max_u > 0 {
            arclen_params(max_u.max(2), &usp, v_mid, true)
        } else {
            let mut us = Vec::new();
            for i in 0..ns_u {
                for sv in 0..u_subs[i] {
                    us.push(usp[i] + sv as f64 * (usp[i + 1] - usp[i]) / u_subs[i] as f64);
                }
            }
            us.push(*usp.last().unwrap());
            us
        };
        let vs: Vec<f64> = if max_v > 0 {
            arclen_params(max_v.max(2), &vsp, u_mid, false)
        } else {
            let mut vs = Vec::new();
            for i in 0..ns_v {
                for sv in 0..v_subs[i] {
                    vs.push(vsp[i] + sv as f64 * (vsp[i + 1] - vsp[i]) / v_subs[i] as f64);
                }
            }
            vs.push(*vsp.last().unwrap());
            vs
        };

        let fix_closed_gap = |params: Vec<f64>, spans: &[f64], closed: bool| -> Vec<f64> {
            if !closed || params.len() < 3 {
                return params;
            }
            let mut params = params;
            params.pop();
            let domain_end = *spans.last().unwrap();
            let wrap_gap = domain_end - *params.last().unwrap();
            let mut max_gap = 0.0_f64;
            for i in 1..params.len() {
                let g = params[i] - params[i - 1];
                if g > max_gap { max_gap = g; }
            }
            if max_gap > 0.0 && wrap_gap > max_gap * 1.5 {
                let extra = ((wrap_gap / max_gap).ceil() as usize).saturating_sub(1);
                let step = wrap_gap / (extra + 1) as f64;
                for _ in 1..=extra {
                    let last = *params.last().unwrap();
                    params.push(last + step);
                }
            }
            params
        };

        let us = fix_closed_gap(us, &usp, closed_u);
        let vs = fix_closed_gap(vs, &vsp, closed_v);
        let nu = us.len();
        let nv_count = vs.len();

        let sing_v0 = s.is_singular(0);
        let sing_v1 = s.is_singular(2);
        let j_start: usize = if sing_v0 { 1 } else { 0 };
        let j_end: usize = if sing_v1 { nv_count - 1 } else { nv_count };
        let nv_grid = j_end - j_start;

        let mut result = Mesh::new();
        let mut south_pole: usize = 0;
        let mut north_pole: usize = 0;
        if sing_v0 {
            if let Some(p) = s.point_at(us[0], vs[0]) {
                south_pole = result.add_vertex(p, None);
                if let Some(vd) = result.vertex.get_mut(&south_pole) {
                    vd.attributes.insert("u".to_string(), us[0]);
                    vd.attributes.insert("v".to_string(), vs[0]);
                }
            }
        }
        if sing_v1 {
            if let Some(p) = s.point_at(us[0], vs[nv_count - 1]) {
                north_pole = result.add_vertex(p, None);
                if let Some(vd) = result.vertex.get_mut(&north_pole) {
                    vd.attributes.insert("u".to_string(), us[0]);
                    vd.attributes.insert("v".to_string(), vs[nv_count - 1]);
                }
            }
        }
        let mut vkey_grid: Vec<Vec<usize>> = Vec::new();
        for i in 0..nu {
            let mut row: Vec<usize> = Vec::new();
            for j in j_start..j_end {
                let p = s.point_at(us[i], vs[j]).unwrap_or(Point::new(0.0, 0.0, 0.0));
                let vk = result.add_vertex(p, None);
                if let Some(vd) = result.vertex.get_mut(&vk) {
                    vd.attributes.insert("u".to_string(), us[i]);
                    vd.attributes.insert("v".to_string(), vs[j]);
                }
                row.push(vk);
            }
            vkey_grid.push(row);
        }

        let grid_idx = |i: usize, j: usize| -> usize {
            vkey_grid[i][j - j_start]
        };

        let nu_faces = if closed_u { nu } else { nu - 1 };

        // South pole fan
        if sing_v0 {
            for i in 0..nu_faces {
                let i1 = (i + 1) % nu;
                result.add_face(vec![south_pole, grid_idx(i1, j_start), grid_idx(i, j_start)], None);
            }
        }

        // Interior grid faces
        let nv_interior = if closed_v && !sing_v0 && !sing_v1 { nv_grid } else { nv_grid - 1 };
        for i in 0..nu_faces {
            for jj in 0..nv_interior {
                let j = jj + j_start;
                let i1 = (i + 1) % nu;
                let j1 = if closed_v && !sing_v0 && !sing_v1 {
                    (jj + 1) % nv_grid + j_start
                } else {
                    j + 1
                };
                let v00 = grid_idx(i, j);
                let v10 = grid_idx(i1, j);
                let v01 = grid_idx(i, j1);
                let v11 = grid_idx(i1, j1);
                if (i + jj) % 2 == 0 {
                    result.add_face(vec![v00, v10, v11], None);
                    result.add_face(vec![v00, v11, v01], None);
                } else {
                    result.add_face(vec![v00, v10, v01], None);
                    result.add_face(vec![v10, v11, v01], None);
                }
            }
        }

        // North pole fan
        if sing_v1 {
            let j_last = j_end - 1;
            for i in 0..nu_faces {
                let i1 = (i + 1) % nu;
                result.add_face(vec![grid_idx(i, j_last), grid_idx(i1, j_last), north_pole], None);
            }
        }

        // Compute vertex normals from face normals
        let mut vn_map: HashMap<usize, (f64, f64, f64)> = HashMap::new();
        for vk in result.vertex.keys() {
            vn_map.insert(*vk, (0.0, 0.0, 0.0));
        }
        let face_keys: Vec<usize> = result.face.keys().cloned().collect();
        for fk in &face_keys {
            if let Some(vids) = result.face.get(fk) {
                if vids.len() < 3 { continue; }
                let pos0 = result.vertex.get(&vids[0]).map(|v| v.position()).unwrap_or(Point::new(0.0, 0.0, 0.0));
                let pos1 = result.vertex.get(&vids[1]).map(|v| v.position()).unwrap_or(Point::new(0.0, 0.0, 0.0));
                let pos2 = result.vertex.get(&vids[2]).map(|v| v.position()).unwrap_or(Point::new(0.0, 0.0, 0.0));
                let e1x = pos1[0]-pos0[0]; let e1y = pos1[1]-pos0[1]; let e1z = pos1[2]-pos0[2];
                let e2x = pos2[0]-pos0[0]; let e2y = pos2[1]-pos0[1]; let e2z = pos2[2]-pos0[2];
                let fnx = e1y*e2z - e1z*e2y;
                let fny = e1z*e2x - e1x*e2z;
                let fnz = e1x*e2y - e1y*e2x;
                let vids_clone: Vec<usize> = vids.clone();
                for vi in vids_clone {
                    let e = vn_map.entry(vi).or_insert((0.0, 0.0, 0.0));
                    e.0 += fnx; e.1 += fny; e.2 += fnz;
                }
            }
        }
        let vkeys: Vec<usize> = result.vertex.keys().cloned().collect();
        for vk in vkeys {
            if let Some((nx, ny, nz)) = vn_map.get_mut(&vk) {
                let len = (*nx * *nx + *ny * *ny + *nz * *nz).sqrt();
                if len > 1e-15 { *nx /= len; *ny /= len; *nz /= len; }
                if let Some(vd) = result.vertex.get_mut(&vk) {
                    vd.set_normal(*nx, *ny, *nz);
                }
            }
        }

        result
    }
}

pub fn remesh_nurbssurface_grid(surface: NurbsSurface, max_u: usize, max_v: usize) -> Mesh {
    RemeshNurbsSurfaceGrid::from_u_v(surface, max_u, max_v)
}
