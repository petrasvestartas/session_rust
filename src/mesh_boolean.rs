// Mesh CSG (difference / union / intersection) via a signed-distance field + marching cubes.
// Robust fallback for imported / freeform solids whose exact surface-surface intersection the
// BRep boolean cannot trace: sample a signed-distance field of the combined solids on a grid,
// extract a watertight isosurface (edge-welded vertices, marching-cubes ambiguity holes filled),
// return a closed mesh. Higher resolution trades speed for accuracy.
use crate::aabb::AABB;
use crate::intersection;
use crate::line::Line;
use crate::mc_tritable::{MC_EDGE, MC_TRI};
use crate::mesh::Mesh;
use crate::obb::OBB;
use crate::point::Point;
use crate::spatial_bvh::SpatialBVH;
use crate::vector::Vector;
use std::collections::{BTreeSet, HashMap};

struct Tri {
    a: Point,
    b: Point,
    c: Point,
}

// Squared distance from point p to triangle (a,b,c) — Ericson closest-point-on-triangle.
fn pt_tri_dist2(p: &Point, a: &Point, b: &Point, c: &Point) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let d1 = ab[0] * ap[0] + ab[1] * ap[1] + ab[2] * ap[2];
    let d2 = ac[0] * ap[0] + ac[1] * ap[1] + ac[2] * ap[2];
    let (cx, cy, cz);
    if d1 <= 0.0 && d2 <= 0.0 {
        cx = a[0]; cy = a[1]; cz = a[2];
    } else {
        let bp = [p[0] - b[0], p[1] - b[1], p[2] - b[2]];
        let d3 = ab[0] * bp[0] + ab[1] * bp[1] + ab[2] * bp[2];
        let d4 = ac[0] * bp[0] + ac[1] * bp[1] + ac[2] * bp[2];
        if d3 >= 0.0 && d4 <= d3 {
            cx = b[0]; cy = b[1]; cz = b[2];
        } else {
            let cp = [p[0] - c[0], p[1] - c[1], p[2] - c[2]];
            let d5 = ab[0] * cp[0] + ab[1] * cp[1] + ab[2] * cp[2];
            let d6 = ac[0] * cp[0] + ac[1] * cp[1] + ac[2] * cp[2];
            if d6 >= 0.0 && d5 <= d6 {
                cx = c[0]; cy = c[1]; cz = c[2];
            } else {
                let vc = d1 * d4 - d3 * d2;
                if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
                    let v = d1 / (d1 - d3);
                    cx = a[0] + v * ab[0]; cy = a[1] + v * ab[1]; cz = a[2] + v * ab[2];
                } else {
                    let vb = d5 * d2 - d1 * d6;
                    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
                        let w = d2 / (d2 - d6);
                        cx = a[0] + w * ac[0]; cy = a[1] + w * ac[1]; cz = a[2] + w * ac[2];
                    } else {
                        let va = d3 * d6 - d5 * d4;
                        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
                            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                            cx = b[0] + w * (c[0] - b[0]); cy = b[1] + w * (c[1] - b[1]); cz = b[2] + w * (c[2] - b[2]);
                        } else {
                            let den = 1.0 / (va + vb + vc);
                            let v = vb * den;
                            let w = vc * den;
                            cx = a[0] + ab[0] * v + ac[0] * w;
                            cy = a[1] + ab[1] * v + ac[1] * w;
                            cz = a[2] + ab[2] * v + ac[2] * w;
                        }
                    }
                }
            }
        }
    }
    let dx = p[0] - cx; let dy = p[1] - cy; let dz = p[2] - cz;
    dx * dx + dy * dy + dz * dz
}

// Triangle soup with a BVH: crack-robust point-in-mesh (7-ray majority) + band-capped distance.
struct MeshQuery {
    tris: Vec<Tri>,
    bvh: SpatialBVH,
}

const RAY_DIRS: [[f64; 3]; 7] = [
    [0.5773502691, 0.6539124, 0.5023147],
    [0.8506508084, 0.5257311121, 0.0],
    [0.0, 0.8506508084, 0.5257311121],
    [0.5257311121, 0.0, 0.8506508084],
    [-0.3574067443, 0.7844645405, 0.5057219851],
    [0.7844645405, -0.5057219851, 0.3574067443],
    [-0.5023147, 0.5773502691, -0.6435942529],
];

impl MeshQuery {
    fn build(m: &Mesh) -> MeshQuery {
        let mut tris = Vec::new();
        for vs in m.face.values() {
            if vs.len() < 3 { continue; }
            let p0 = m.vertex[&vs[0]].position();
            for i in 1..vs.len() - 1 {
                tris.push(Tri { a: p0, b: m.vertex[&vs[i]].position(), c: m.vertex[&vs[i + 1]].position() });
            }
        }
        let mut boxes = Vec::with_capacity(tris.len());
        let (mut xmn, mut ymn, mut zmn) = (1e300, 1e300, 1e300);
        let (mut xmx, mut ymx, mut zmx) = (-1e300, -1e300, -1e300);
        for t in &tris {
            boxes.push(OBB::from_points(&[t.a, t.b, t.c], 1e-9));
            for p in [&t.a, &t.b, &t.c] {
                xmn = xmn.min(p[0]); ymn = ymn.min(p[1]); zmn = zmn.min(p[2]);
                xmx = xmx.max(p[0]); ymx = ymx.max(p[1]); zmx = zmx.max(p[2]);
            }
        }
        let mut diag = ((xmx - xmn).powi(2) + (ymx - ymn).powi(2) + (zmx - zmn).powi(2)).sqrt();
        if diag <= 0.0 { diag = 1.0; }
        let bvh = SpatialBVH::from_boxes(&boxes, diag * 2.0);
        MeshQuery { tris, bvh }
    }

    fn inside(&self, p: &Point) -> bool {
        let mut votes = 0;
        for d in &RAY_DIRS {
            let dir = Vector::new(d[0], d[1], d[2]);
            let mut cand: Vec<usize> = Vec::new();
            self.bvh.ray_cast(p, &dir, &mut cand, true);
            let line = Line::new(p[0], p[1], p[2], p[0] + d[0] * 1e6, p[1] + d[1] * 1e6, p[2] + d[2] * 1e6);
            let mut cnt = 0;
            for id in &cand {
                let t = &self.tris[*id];
                if intersection::ray_triangle(&line, &t.a, &t.b, &t.c, 1e-9).is_some() {
                    cnt += 1;
                }
            }
            votes += cnt & 1;
        }
        votes * 2 > 7
    }

    fn dist(&self, p: &Point, band: f64) -> f64 {
        let mut r = band * 0.5;
        let mut best2 = band * band;
        for _ in 0..6 {
            let q = OBB::from_points(
                &[Point::new(p[0] - r, p[1] - r, p[2] - r), Point::new(p[0] + r, p[1] + r, p[2] + r)],
                0.0,
            );
            let cand = self.bvh.query_aabb(&q);
            let mut b2 = band * band;
            for id in &cand {
                let t = &self.tris[*id];
                b2 = b2.min(pt_tri_dist2(p, &t.a, &t.b, &t.c));
            }
            best2 = best2.min(b2);
            if best2 <= r * r { break; }
            r *= 2.0;
            if r > band { break; }
        }
        best2.sqrt()
    }

    fn sdf(&self, p: &Point, band: f64) -> f64 {
        (if self.inside(p) { -1.0 } else { 1.0 }) * self.dist(p, band)
    }
}

#[derive(Clone, Copy)]
enum BoolOp {
    Difference,
    Union,
    Intersection,
}

// combine(sdf_A, sdf_B): difference = A ∩ !B = max(sa,-sb); union = min; intersection = max.
fn combine(op: BoolOp, sa: f64, sb: f64) -> f64 {
    match op {
        BoolOp::Difference => sa.max(-sb),
        BoolOp::Union => sa.min(sb),
        BoolOp::Intersection => sa.max(sb),
    }
}

const CORNER: [[i32; 3]; 8] = [[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]];

fn mesh_boolean_op(a: &Mesh, b: &Mesh, op: BoolOp, resolution: i32) -> Mesh {
    let resolution = resolution.max(8);
    let qa = MeshQuery::build(a);
    let qb = MeshQuery::build(b);
    if qa.tris.is_empty() || qb.tris.is_empty() {
        return Mesh::new();
    }

    let mut lo = [1e300f64; 3];
    let mut hi = [-1e300f64; 3];
    for q in [&qa, &qb] {
        for t in &q.tris {
            for p in [&t.a, &t.b, &t.c] {
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
        }
    }
    let mut cell = 0.0f64;
    for k in 0..3 {
        cell = cell.max((hi[k] - lo[k]) / resolution as f64);
    }
    if cell <= 0.0 {
        return Mesh::new();
    }
    for k in 0..3 {
        lo[k] -= cell * 2.0;
        hi[k] += cell * 2.0;
    }
    let nx = ((hi[0] - lo[0]) / cell).ceil() as i32;
    let ny = ((hi[1] - lo[1]) / cell).ceil() as i32;
    let nz = ((hi[2] - lo[2]) / cell).ceil() as i32;

    // signed-distance field of the combined solids at grid corners (band-capped)
    let band = cell * 3.0;
    let gidx = |i: i32, j: i32, k: i32| -> usize {
        ((i as usize * (ny + 1) as usize + j as usize) * (nz + 1) as usize) + k as usize
    };
    let mut sdf = vec![band as f32; ((nx + 1) as usize) * ((ny + 1) as usize) * ((nz + 1) as usize)];
    for i in 0..=nx {
        for j in 0..=ny {
            for k in 0..=nz {
                let p = Point::new(lo[0] + i as f64 * cell, lo[1] + j as f64 * cell, lo[2] + k as f64 * cell);
                sdf[gidx(i, j, k)] = combine(op, qa.sdf(&p, band), qb.sdf(&p, band)) as f32;
            }
        }
    }

    // marching cubes with edge-keyed vertex welding + zero-crossing interpolation
    let mut out = Mesh::new();
    let mut edge_vert: HashMap<(usize, usize), usize> = HashMap::new();
    let mut edge_vertex = |out: &mut Mesh, sdf: &[f32], i: i32, j: i32, k: i32, e: usize| -> usize {
        let a = MC_EDGE[e][0];
        let b = MC_EDGE[e][1];
        let ga = gidx(i + CORNER[a][0], j + CORNER[a][1], k + CORNER[a][2]);
        let gb = gidx(i + CORNER[b][0], j + CORNER[b][1], k + CORNER[b][2]);
        let key = (ga.min(gb), ga.max(gb));
        if let Some(v) = edge_vert.get(&key) {
            return *v;
        }
        let pa = Point::new(
            lo[0] + (i + CORNER[a][0]) as f64 * cell,
            lo[1] + (j + CORNER[a][1]) as f64 * cell,
            lo[2] + (k + CORNER[a][2]) as f64 * cell,
        );
        let pb = Point::new(
            lo[0] + (i + CORNER[b][0]) as f64 * cell,
            lo[1] + (j + CORNER[b][1]) as f64 * cell,
            lo[2] + (k + CORNER[b][2]) as f64 * cell,
        );
        let va = sdf[ga] as f64;
        let vb = sdf[gb] as f64;
        let mut t = if (va - vb).abs() > 1e-12 { va / (va - vb) } else { 0.5 };
        t = t.clamp(0.0, 1.0);
        let vk = out.add_vertex(
            Point::new(pa[0] + t * (pb[0] - pa[0]), pa[1] + t * (pb[1] - pa[1]), pa[2] + t * (pb[2] - pa[2])),
            None,
        );
        edge_vert.insert(key, vk);
        vk
    };
    for i in 0..nx {
        for j in 0..ny {
            for k in 0..nz {
                let mut ci = 0usize;
                for c in 0..8 {
                    if sdf[gidx(i + CORNER[c][0], j + CORNER[c][1], k + CORNER[c][2])] < 0.0 {
                        ci |= 1 << c;
                    }
                }
                if ci == 0 || ci == 255 { continue; }
                let mut t = 0;
                while MC_TRI[ci][t] != -1 {
                    let v0 = edge_vertex(&mut out, &sdf, i, j, k, MC_TRI[ci][t] as usize);
                    let v1 = edge_vertex(&mut out, &sdf, i, j, k, MC_TRI[ci][t + 1] as usize);
                    let v2 = edge_vertex(&mut out, &sdf, i, j, k, MC_TRI[ci][t + 2] as usize);
                    if v0 != v1 && v1 != v2 && v0 != v2 {
                        out.add_face(vec![v0, v1, v2], None);
                    }
                    t += 3;
                }
            }
        }
    }

    // fill the small marching-cubes ambiguity holes: walk naked-edge loops, fan-triangulate
    let naked = out.naked_edges(true);
    let mut adj: HashMap<usize, Vec<usize>> = HashMap::new();
    let mut eset: BTreeSet<(usize, usize)> = BTreeSet::new();
    for e in &naked {
        adj.entry(e.0).or_default().push(e.1);
        adj.entry(e.1).or_default().push(e.0);
        eset.insert((e.0.min(e.1), e.0.max(e.1)));
    }
    while let Some(&se) = eset.iter().next() {
        eset.remove(&se);
        let start = se.0;
        let mut prev = start;
        let mut cur = se.1;
        let mut loop_v = vec![start, cur];
        let mut ok = true;
        while cur != start {
            let mut nxt: Option<usize> = None;
            if let Some(ns) = adj.get(&cur) {
                for &w in ns {
                    let kk = (cur.min(w), cur.max(w));
                    if w != prev && eset.contains(&kk) {
                        nxt = Some(w);
                        eset.remove(&kk);
                        break;
                    }
                }
            }
            match nxt {
                None => { ok = false; break; }
                Some(w) => {
                    if w == start { break; }
                    loop_v.push(w);
                    prev = cur;
                    cur = w;
                }
            }
        }
        if ok && loop_v.len() >= 3 {
            for i in 1..loop_v.len() - 1 {
                out.add_face(vec![loop_v[0], loop_v[i], loop_v[i + 1]], None);
            }
        }
    }
    out.unify_winding();
    out
}

impl Mesh {
    /// Mesh CSG via a signed-distance field + marching cubes. Robust for imported/freeform
    /// meshes whose exact surface-surface intersection is hard; returns a watertight closed mesh.
    /// Higher resolution trades speed for accuracy.
    pub fn boolean_difference(&self, other: &Mesh, resolution: i32) -> Mesh {
        mesh_boolean_op(self, other, BoolOp::Difference, resolution)
    }
    pub fn boolean_union(&self, other: &Mesh, resolution: i32) -> Mesh {
        mesh_boolean_op(self, other, BoolOp::Union, resolution)
    }
    pub fn boolean_intersection(&self, other: &Mesh, resolution: i32) -> Mesh {
        mesh_boolean_op(self, other, BoolOp::Intersection, resolution)
    }
}
