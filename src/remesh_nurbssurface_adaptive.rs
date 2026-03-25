use std::collections::BTreeMap;

use crate::mesh::Mesh;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
use crate::tolerance::Tolerance;

pub struct RemeshNurbsSurfaceAdaptive {
    surface: NurbsSurface,
    max_angle: f64,
    max_edge_length: f64,
    min_edge_length: f64,
    max_chord_height: f64,
}

impl RemeshNurbsSurfaceAdaptive {
    pub fn new(surface: NurbsSurface) -> Self {
        RemeshNurbsSurfaceAdaptive {
            surface,
            max_angle: 20.0,
            max_edge_length: 0.0,
            min_edge_length: 0.0,
            max_chord_height: 0.0,
        }
    }

    pub fn set_max_angle(&mut self, degrees: f64) -> &mut Self {
        self.max_angle = degrees;
        self
    }

    pub fn set_max_edge_length(&mut self, length: f64) -> &mut Self {
        self.max_edge_length = length;
        self
    }

    pub fn set_min_edge_length(&mut self, length: f64) -> &mut Self {
        self.min_edge_length = length;
        self
    }

    pub fn set_max_chord_height(&mut self, height: f64) -> &mut Self {
        self.max_chord_height = height;
        self
    }

    pub fn get_max_angle(&self) -> f64 { self.max_angle }
    pub fn get_max_edge_length(&self) -> f64 { self.max_edge_length }
    pub fn get_min_edge_length(&self) -> f64 { self.min_edge_length }
    pub fn get_max_chord_height(&self) -> f64 { self.max_chord_height }

    fn compute_bbox_diagonal(&self) -> f64 {
        let s = &self.surface;
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
        (dx*dx + dy*dy + dz*dz).sqrt()
    }

    pub fn mesh(&self) -> Mesh {
        let s = &self.surface;
        let usp = s.get_span_vector(0);
        let vsp = s.get_span_vector(1);
        let ns_u = usp.len() - 1;
        let ns_v = vsp.len() - 1;
        let bbox_diag = self.compute_bbox_diagonal();

        let norm_tol = 2.0 - 2.0 * (self.max_angle * Tolerance::PI / 180.0).cos();
        let chord_tol = if self.max_chord_height > 0.0 { self.max_chord_height } else { bbox_diag * 0.005 };
        let closed_u = s.is_closed(0);
        let closed_v = s.is_closed(1);
        let sing_v0 = s.is_singular(0);
        let sing_v1 = s.is_singular(2);
        let max_depth = 8i32;

        // Corner: (px, py, pz, nx, ny, nz)
        #[derive(Clone, Copy, Default)]
        struct Corner { px: f64, py: f64, pz: f64, nx: f64, ny: f64, nz: f64 }

        // Node: u0, v0, u1, v1, corners[5], children[4], depth
        #[derive(Clone)]
        struct Node {
            u0: f64, v0: f64, u1: f64, v1: f64,
            c: [Corner; 5],
            ch: [i32; 4],
            depth: i32,
        }

        impl Node {
            fn leaf(&self) -> bool { self.ch[0] < 0 && self.ch[2] < 0 }
        }

        let mut nodes: Vec<Node> = Vec::with_capacity(ns_u * ns_v * 256);

        let eval_corner = |u: f64, v: f64| -> Corner {
            let mut c = Corner::default();
            if let Some(pt) = s.point_at(u, v) {
                c.px = pt[0]; c.py = pt[1]; c.pz = pt[2];
            }
            let derivs = s.evaluate(u, v, 1);
            if derivs.len() >= 3 {
                let (dux, duy, duz) = (derivs[1][0], derivs[1][1], derivs[1][2]);
                let (dvx, dvy, dvz) = (derivs[2][0], derivs[2][1], derivs[2][2]);
                let nx = duy*dvz - duz*dvy;
                let ny = duz*dvx - dux*dvz;
                let nz = dux*dvy - duy*dvx;
                let len = (nx*nx + ny*ny + nz*nz).sqrt();
                if len > 1e-10 {
                    c.nx = nx/len; c.ny = ny/len; c.nz = nz/len;
                }
            }
            c
        };

        let ndiff = |a: &Corner, b: &Corner| -> f64 {
            let (dx, dy, dz) = (a.nx-b.nx, a.ny-b.ny, a.nz-b.nz);
            dx*dx + dy*dy + dz*dz
        };
        let nvalid = |c: &Corner| -> bool { c.nx*c.nx + c.ny*c.ny + c.nz*c.nz > 1e-20 };
        let pdist2 = |a: &Corner, b: &Corner| -> f64 {
            let (dx, dy, dz) = (a.px-b.px, a.py-b.py, a.pz-b.pz);
            dx*dx + dy*dy + dz*dz
        };
        let pmid = |a: &Corner, b: &Corner| -> Corner {
            Corner { px: (a.px+b.px)*0.5, py: (a.py+b.py)*0.5, pz: (a.pz+b.pz)*0.5, nx: 0.0, ny: 0.0, nz: 0.0 }
        };

        let fill_node = |u0: f64, v0: f64, u1: f64, v1: f64,
                         sw: Corner, se: Corner, ne: Corner, nw: Corner,
                         depth: i32, eval_fn: &dyn Fn(f64,f64)->Corner| -> Node {
            let ct = eval_fn((u0+u1)*0.5, (v0+v1)*0.5);
            Node { u0, v0, u1, v1, c: [sw, se, ne, nw, ct], ch: [-1,-1,-1,-1], depth }
        };

        // Iterative subdivision using a stack
        let max_edge_length = self.max_edge_length;
        let min_edge_length = self.min_edge_length;

        let mut subdivide_stack: Vec<usize> = Vec::new();

        macro_rules! do_subdivide {
            ($nodes:expr) => {{
                while let Some(idx) = subdivide_stack.pop() {
                    if $nodes[idx].depth >= max_depth { continue; }
                    let p = $nodes[idx].clone();
                    let um = (p.u0 + p.u1) * 0.5;
                    let vm = (p.v0 + p.v1) * 0.5;

                    if min_edge_length > 0.0 {
                        let mut me = 0.0_f64;
                        for i in 0..4 {
                            let d = pdist2(&p.c[i], &p.c[(i+1)%4]);
                            if d > me { me = d; }
                        }
                        if me < min_edge_length * min_edge_length { continue; }
                    }

                    let mut split_u = false;
                    let mut split_v = false;
                    if nvalid(&p.c[0]) && nvalid(&p.c[1]) && ndiff(&p.c[0], &p.c[1]) > norm_tol { split_u = true; }
                    if nvalid(&p.c[2]) && nvalid(&p.c[3]) && ndiff(&p.c[2], &p.c[3]) > norm_tol { split_u = true; }
                    if nvalid(&p.c[1]) && nvalid(&p.c[2]) && ndiff(&p.c[1], &p.c[2]) > norm_tol { split_v = true; }
                    if nvalid(&p.c[3]) && nvalid(&p.c[0]) && ndiff(&p.c[3], &p.c[0]) > norm_tol { split_v = true; }

                    let mut s_mid = Corner::default(); let mut n_mid = Corner::default();
                    let mut w_mid = Corner::default(); let mut e_mid = Corner::default();
                    let mut have_sn = false; let mut have_we = false;

                    if !split_u {
                        s_mid = eval_corner(um, p.v0); n_mid = eval_corner(um, p.v1); have_sn = true;
                        let s_lin = pmid(&p.c[0], &p.c[1]);
                        let n_lin = pmid(&p.c[3], &p.c[2]);
                        if pdist2(&s_mid, &s_lin) > chord_tol * chord_tol { split_u = true; }
                        if pdist2(&n_mid, &n_lin) > chord_tol * chord_tol { split_u = true; }
                        if nvalid(&s_mid) && nvalid(&p.c[0]) && ndiff(&s_mid, &p.c[0]) > norm_tol { split_u = true; }
                        if nvalid(&n_mid) && nvalid(&p.c[3]) && ndiff(&n_mid, &p.c[3]) > norm_tol { split_u = true; }
                    }
                    if !split_v {
                        w_mid = eval_corner(p.u0, vm); e_mid = eval_corner(p.u1, vm); have_we = true;
                        let w_lin = pmid(&p.c[0], &p.c[3]);
                        let e_lin = pmid(&p.c[1], &p.c[2]);
                        if pdist2(&w_mid, &w_lin) > chord_tol * chord_tol { split_v = true; }
                        if pdist2(&e_mid, &e_lin) > chord_tol * chord_tol { split_v = true; }
                        if nvalid(&w_mid) && nvalid(&p.c[0]) && ndiff(&w_mid, &p.c[0]) > norm_tol { split_v = true; }
                        if nvalid(&e_mid) && nvalid(&p.c[1]) && ndiff(&e_mid, &p.c[1]) > norm_tol { split_v = true; }
                    }

                    if !split_u && !split_v {
                        let degenerate = (0..4).any(|ci| pdist2(&p.c[ci], &p.c[(ci + 1) % 4]) < chord_tol * chord_tol);
                        if !degenerate {
                            let twist_tol2 = 4.0 * chord_tol * chord_tol;
                            for d in 0..2 {
                                let mid = pmid(&p.c[d], &p.c[d+2]);
                                if pdist2(&p.c[4], &mid) > twist_tol2 { split_u = true; split_v = true; }
                            }
                        }
                    }

                    if max_edge_length > 0.0 {
                        let me = max_edge_length * max_edge_length;
                        if pdist2(&p.c[0], &p.c[1]) > me || pdist2(&p.c[2], &p.c[3]) > me { split_u = true; }
                        if pdist2(&p.c[1], &p.c[2]) > me || pdist2(&p.c[3], &p.c[0]) > me { split_v = true; }
                    }

                    if !split_u || !split_v {
                        let derivs2 = s.evaluate(um, vm, 2);
                        if derivs2.len() >= 6 {
                            let (dux, duy, duz) = (derivs2[2][0], derivs2[2][1], derivs2[2][2]);
                            let (dvx, dvy, dvz) = (derivs2[1][0], derivs2[1][1], derivs2[1][2]);
                            let (d2ux, d2uy, d2uz) = (derivs2[5][0], derivs2[5][1], derivs2[5][2]);
                            let (d2vx, d2vy, d2vz) = (derivs2[3][0], derivs2[3][1], derivs2[3][2]);
                            let nx = duy*dvz - duz*dvy;
                            let ny = duz*dvx - dux*dvz;
                            let nz = dux*dvy - duy*dvx;
                            let nlen = (nx*nx + ny*ny + nz*nz).sqrt();
                            let e_cap = dux*dux + duy*duy + duz*duz;
                            let g_cap = dvx*dvx + dvy*dvy + dvz*dvz;
                            if nlen > 1e-10 && e_cap > 1e-20 && g_cap > 1e-20 {
                                let (nnx, nny, nnz) = (nx/nlen, ny/nlen, nz/nlen);
                                let e_coeff = d2ux*nnx + d2uy*nny + d2uz*nnz;
                                let g_coeff = d2vx*nnx + d2vy*nny + d2vz*nnz;
                                let kappa_u = e_coeff.abs() / e_cap;
                                let kappa_v = g_coeff.abs() / g_cap;
                                let span_u = e_cap.sqrt() * (p.u1 - p.u0);
                                let span_v = g_cap.sqrt() * (p.v1 - p.v0);
                                if !split_u && kappa_u > 1e-20 {
                                    let needed_u = span_u * (kappa_u / (8.0 * chord_tol)).sqrt();
                                    if needed_u > 1.0 { split_u = true; }
                                }
                                if !split_v && kappa_v > 1e-20 {
                                    let needed_v = span_v * (kappa_v / (8.0 * chord_tol)).sqrt();
                                    if needed_v > 1.0 { split_v = true; }
                                }
                            }
                        }
                    }

                    if !split_u && !split_v { continue; }

                    let d = p.depth + 1;
                    let b = $nodes.len() as i32;

                    if split_u && split_v {
                        if !have_sn { s_mid = eval_corner(um, p.v0); n_mid = eval_corner(um, p.v1); }
                        if !have_we { w_mid = eval_corner(p.u0, vm); e_mid = eval_corner(p.u1, vm); }
                        $nodes.resize_with($nodes.len() + 4, || Node { u0:0.0,v0:0.0,u1:0.0,v1:0.0,c:[Corner::default();5],ch:[-1,-1,-1,-1],depth:0 });
                        $nodes[idx].ch[0] = b; $nodes[idx].ch[1] = b+1; $nodes[idx].ch[2] = b+2; $nodes[idx].ch[3] = b+3;
                        $nodes[b as usize]   = fill_node(p.u0, p.v0, um,   vm,   p.c[0], s_mid, p.c[4], w_mid, d, &eval_corner);
                        $nodes[b as usize+1] = fill_node(um,   p.v0, p.u1, vm,   s_mid, p.c[1], e_mid, p.c[4], d, &eval_corner);
                        $nodes[b as usize+2] = fill_node(um,   vm,   p.u1, p.v1, p.c[4], e_mid, p.c[2], n_mid, d, &eval_corner);
                        $nodes[b as usize+3] = fill_node(p.u0, vm,   um,   p.v1, w_mid, p.c[4], n_mid, p.c[3], d, &eval_corner);
                        subdivide_stack.extend_from_slice(&[b as usize, b as usize+1, b as usize+2, b as usize+3]);
                    } else if split_u {
                        if !have_sn { s_mid = eval_corner(um, p.v0); n_mid = eval_corner(um, p.v1); }
                        $nodes.resize_with($nodes.len() + 2, || Node { u0:0.0,v0:0.0,u1:0.0,v1:0.0,c:[Corner::default();5],ch:[-1,-1,-1,-1],depth:0 });
                        $nodes[idx].ch[0] = b; $nodes[idx].ch[1] = b+1; $nodes[idx].ch[2] = -1; $nodes[idx].ch[3] = -1;
                        $nodes[b as usize]   = fill_node(p.u0, p.v0, um,   p.v1, p.c[0], s_mid, n_mid, p.c[3], d, &eval_corner);
                        $nodes[b as usize+1] = fill_node(um,   p.v0, p.u1, p.v1, s_mid, p.c[1], p.c[2], n_mid, d, &eval_corner);
                        subdivide_stack.extend_from_slice(&[b as usize, b as usize+1]);
                    } else {
                        if !have_we { w_mid = eval_corner(p.u0, vm); e_mid = eval_corner(p.u1, vm); }
                        $nodes.resize_with($nodes.len() + 2, || Node { u0:0.0,v0:0.0,u1:0.0,v1:0.0,c:[Corner::default();5],ch:[-1,-1,-1,-1],depth:0 });
                        $nodes[idx].ch[0] = b; $nodes[idx].ch[1] = -1; $nodes[idx].ch[2] = b+1; $nodes[idx].ch[3] = -1;
                        $nodes[b as usize]   = fill_node(p.u0, p.v0, p.u1, vm,   p.c[0], p.c[1], e_mid, w_mid, d, &eval_corner);
                        $nodes[b as usize+1] = fill_node(p.u0, vm,   p.u1, p.v1, w_mid, e_mid, p.c[2], p.c[3], d, &eval_corner);
                        subdivide_stack.extend_from_slice(&[b as usize, b as usize+1]);
                    }
                }
            }};
        }

        // Pre-evaluate grid intersections
        let ng_u = ns_u + 1;
        let ng_v = ns_v + 1;
        let mut grid = vec![Corner::default(); ng_u * ng_v];
        for i in 0..ng_u {
            for j in 0..ng_v {
                grid[i * ng_v + j] = eval_corner(usp[i], vsp[j]);
            }
        }

        for i in 0..ns_u {
            for j in 0..ns_v {
                let idx = nodes.len();
                let ct = eval_corner((usp[i]+usp[i+1])*0.5, (vsp[j]+vsp[j+1])*0.5);
                nodes.push(Node {
                    u0: usp[i], v0: vsp[j], u1: usp[i+1], v1: vsp[j+1],
                    c: [
                        grid[i * ng_v + j],
                        grid[(i+1) * ng_v + j],
                        grid[(i+1) * ng_v + (j+1)],
                        grid[i * ng_v + (j+1)],
                        ct,
                    ],
                    ch: [-1,-1,-1,-1],
                    depth: 0,
                });
                subdivide_stack.push(idx);
                do_subdivide!(nodes);
            }
        }

        // Collect leaf corner UVs for T-junction detection
        let qk = |x: f64| -> i64 { (x * 1e10).round() as i64 };
        let norm_u_fn = |u: f64| -> f64 {
            if closed_u && (u - usp[usp.len()-1]).abs() < 1e-10 { usp[0] } else { u }
        };
        let norm_v_fn = |v: f64| -> f64 {
            if closed_v && (v - vsp[vsp.len()-1]).abs() < 1e-10 { vsp[0] } else { v }
        };

        type IKey = (i64, i64);
        let mut key_corner: BTreeMap<IKey, Corner> = BTreeMap::new();
        let mut by_v: BTreeMap<i64, Vec<i64>> = BTreeMap::new();
        let mut by_u: BTreeMap<i64, Vec<i64>> = BTreeMap::new();

        for nd in &nodes {
            if !nd.leaf() { continue; }
            let us4 = [nd.u0, nd.u1, nd.u1, nd.u0];
            let vs4 = [nd.v0, nd.v0, nd.v1, nd.v1];
            for ci in 0..4 {
                let un = norm_u_fn(us4[ci]);
                let vn = norm_v_fn(vs4[ci]);
                let key: IKey = (qk(un), qk(vn));
                key_corner.entry(key).or_insert(nd.c[ci]);
                by_v.entry(qk(vn)).or_default().push(qk(us4[ci]));
                by_u.entry(qk(un)).or_default().push(qk(vs4[ci]));
            }
        }
        for vec in by_v.values_mut() { vec.sort(); vec.dedup(); }
        for vec in by_u.values_mut() { vec.sort(); vec.dedup(); }

        let mut result = Mesh::default();
        let south_pole: Option<usize>;
        let north_pole: Option<usize>;

        if sing_v0 {
            let c0 = eval_corner(usp[0], vsp[0]);
            south_pole = Some(result.add_vertex(Point::new(c0.px, c0.py, c0.pz), None));
        } else {
            south_pole = None;
        }
        if sing_v1 {
            let c1 = eval_corner(usp[0], vsp[vsp.len()-1]);
            north_pole = Some(result.add_vertex(Point::new(c1.px, c1.py, c1.pz), None));
        } else {
            north_pole = None;
        }

        let mut vtx_map: BTreeMap<IKey, usize> = BTreeMap::new();

        let find_h_mids = |u_start: f64, u_end: f64, v_const: f64,
                           by_v: &BTreeMap<i64, Vec<i64>>| -> Vec<f64> {
            let qv = qk(norm_v_fn(v_const));
            let vec = match by_v.get(&qv) { Some(v) => v, None => return Vec::new() };
            let (qa, qb) = (qk(u_start), qk(u_end));
            let (lo, hi) = (qa.min(qb), qa.max(qb));
            let li = vec.partition_point(|&x| x <= lo);
            let ri = vec.partition_point(|&x| x < hi);
            let mut res: Vec<f64> = vec[li..ri].iter().map(|&x| x as f64 / 1e10).collect();
            if qa > qb { res.reverse(); }
            res
        };

        let find_v_mids = |u_const: f64, v_start: f64, v_end: f64,
                           by_u: &BTreeMap<i64, Vec<i64>>| -> Vec<f64> {
            let qu = qk(norm_u_fn(u_const));
            let vec = match by_u.get(&qu) { Some(v) => v, None => return Vec::new() };
            let (qa, qb) = (qk(v_start), qk(v_end));
            let (lo, hi) = (qa.min(qb), qa.max(qb));
            let li = vec.partition_point(|&x| x <= lo);
            let ri = vec.partition_point(|&x| x < hi);
            let mut res: Vec<f64> = vec[li..ri].iter().map(|&x| x as f64 / 1e10).collect();
            if qa > qb { res.reverse(); }
            res
        };

        // Helper macro: resolve UV → mesh vertex index (create if needed)
        // Using a macro avoids closure borrow lifetime conflicts with result/vtx_map
        macro_rules! get_vtx {
            ($u:expr, $v:expr) => {{
                let u = $u; let v = $v;
                if sing_v0 && (v - vsp[0]).abs() < 1e-10 {
                    south_pole.unwrap()
                } else if sing_v1 && (v - vsp[vsp.len()-1]).abs() < 1e-10 {
                    north_pole.unwrap()
                } else {
                    let un = norm_u_fn(u);
                    let vn = norm_v_fn(v);
                    let key: IKey = (qk(un), qk(vn));
                    if let Some(&vi) = vtx_map.get(&key) {
                        vi
                    } else {
                        let c = match key_corner.get(&key) { Some(&corner) => corner, None => eval_corner(un, vn) };
                        let vi = result.add_vertex(Point::new(c.px, c.py, c.pz), None);
                        if let Some(vd) = result.vertex.get_mut(&vi) {
                            vd.attributes.insert("u".to_string(), un);
                            vd.attributes.insert("v".to_string(), vn);
                        }
                        vtx_map.insert(key, vi);
                        vi
                    }
                }
            }};
        }

        // Triangulate leaves
        for ni in 0..nodes.len() {
            if !nodes[ni].leaf() { continue; }
            let nd = nodes[ni].clone();
            let mut poly: Vec<usize> = Vec::new();

            poly.push(get_vtx!(nd.u0, nd.v0));
            for u in find_h_mids(nd.u0, nd.u1, nd.v0, &by_v) { poly.push(get_vtx!(u, nd.v0)); }
            poly.push(get_vtx!(nd.u1, nd.v0));
            for v in find_v_mids(nd.u1, nd.v0, nd.v1, &by_u) { poly.push(get_vtx!(nd.u1, v)); }
            poly.push(get_vtx!(nd.u1, nd.v1));
            for u in find_h_mids(nd.u1, nd.u0, nd.v1, &by_v) { poly.push(get_vtx!(u, nd.v1)); }
            poly.push(get_vtx!(nd.u0, nd.v1));
            for v in find_v_mids(nd.u0, nd.v1, nd.v0, &by_u) { poly.push(get_vtx!(nd.u0, v)); }

            // Remove consecutive duplicates and wrap-around duplicate
            let mut deduped: Vec<usize> = Vec::new();
            for vi in &poly {
                if deduped.last() != Some(vi) { deduped.push(*vi); }
            }
            while deduped.len() > 1 && deduped.first() == deduped.last() { deduped.pop(); }
            let poly = deduped;

            let n = poly.len();
            if n < 3 { continue; }

            if n == 3 {
                result.add_face(vec![poly[0], poly[1], poly[2]], None);
            } else if n == 4 {
                let p0 = result.vertex.get(&poly[0]).unwrap().position();
                let p1 = result.vertex.get(&poly[1]).unwrap().position();
                let p2 = result.vertex.get(&poly[2]).unwrap().position();
                let p3 = result.vertex.get(&poly[3]).unwrap().position();
                let d02 = (p0[0]-p2[0])*(p0[0]-p2[0]) + (p0[1]-p2[1])*(p0[1]-p2[1]) + (p0[2]-p2[2])*(p0[2]-p2[2]);
                let d13 = (p1[0]-p3[0])*(p1[0]-p3[0]) + (p1[1]-p3[1])*(p1[1]-p3[1]) + (p1[2]-p3[2])*(p1[2]-p3[2]);
                if d02 <= d13 {
                    result.add_face(vec![poly[0], poly[1], poly[2]], None);
                    result.add_face(vec![poly[0], poly[2], poly[3]], None);
                } else {
                    result.add_face(vec![poly[0], poly[1], poly[3]], None);
                    result.add_face(vec![poly[1], poly[2], poly[3]], None);
                }
            } else {
                let cu = norm_u_fn((nd.u0+nd.u1)*0.5);
                let cv = norm_v_fn((nd.v0+nd.v1)*0.5);
                let cvi = get_vtx!(cu, cv);
                for i in 0..n {
                    let j = (i+1) % n;
                    if poly[i] != poly[j] && poly[i] != cvi && poly[j] != cvi {
                        result.add_face(vec![poly[i], poly[j], cvi], None);
                    }
                }
            }
        }

        // Compute vertex normals from face normals
        let nv = if result.vertex.is_empty() { 0 } else { *result.vertex.keys().max().unwrap() + 1 };
        let mut vnx = vec![0.0_f64; nv];
        let mut vny = vec![0.0_f64; nv];
        let mut vnz = vec![0.0_f64; nv];
        for vids in result.face.values() {
            if vids.len() < 3 { continue; }
            let pos0 = result.vertex.get(&vids[0]).unwrap().position();
            let pos1 = result.vertex.get(&vids[1]).unwrap().position();
            let pos2 = result.vertex.get(&vids[2]).unwrap().position();
            let (e1x, e1y, e1z) = (pos1[0]-pos0[0], pos1[1]-pos0[1], pos1[2]-pos0[2]);
            let (e2x, e2y, e2z) = (pos2[0]-pos0[0], pos2[1]-pos0[1], pos2[2]-pos0[2]);
            let (fnx, fny, fnz) = (e1y*e2z - e1z*e2y, e1z*e2x - e1x*e2z, e1x*e2y - e1y*e2x);
            for &vi in vids {
                if vi < nv { vnx[vi] += fnx; vny[vi] += fny; vnz[vi] += fnz; }
            }
        }
        for (&vk, vdata) in &mut result.vertex {
            if vk >= nv { continue; }
            let len = (vnx[vk]*vnx[vk] + vny[vk]*vny[vk] + vnz[vk]*vnz[vk]).sqrt();
            if len > 1e-15 { vnx[vk] /= len; vny[vk] /= len; vnz[vk] /= len; }
            vdata.set_normal(vnx[vk], vny[vk], vnz[vk]);
        }
        result
    }
}
