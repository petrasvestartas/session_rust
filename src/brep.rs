use crate::color::Color;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
use crate::vector::Vector;
use crate::xform::Xform;
use serde::ser::SerializeMap;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

fn aabb_from_surface(srf: &NurbsSurface) -> ([f64; 3], [f64; 3]) {
    let n = 6;
    let (u0, u1) = srf.domain(0).unwrap_or((0.0, 1.0));
    let (v0, v1) = srf.domain(1).unwrap_or((0.0, 1.0));
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for i in 0..=n {
        for j in 0..=n {
            let u = u0 + (u1 - u0) * (i as f64) / (n as f64);
            let v = v0 + (v1 - v0) * (j as f64) / (n as f64);
            if let Some(p) = srf.point_at(u, v) {
                for k in 0..3 {
                    if p[k] < lo[k] { lo[k] = p[k]; }
                    if p[k] > hi[k] { hi[k] = p[k]; }
                }
            }
        }
    }
    (lo, hi)
}

fn aabb_from_curve(crv: &NurbsCurve) -> ([f64; 3], [f64; 3]) {
    let n = 16;
    let (c0, c1) = crv.domain();
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for i in 0..=n {
        let p = crv.point_at(c0 + (c1 - c0) * (i as f64) / (n as f64));
        for k in 0..3 {
            if p[k] < lo[k] { lo[k] = p[k]; }
            if p[k] > hi[k] { hi[k] = p[k]; }
        }
    }
    (lo, hi)
}

fn aabb_overlap(a: &([f64; 3], [f64; 3]), b: &([f64; 3], [f64; 3]), m: f64) -> bool {
    for k in 0..3 {
        if a.0[k] - m > b.1[k] || b.0[k] - m > a.1[k] {
            return false;
        }
    }
    true
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BRepTrimType {
    Boundary = 0,
    Mated = 1,
    Seam = 2,
    Singular = 3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BRepLoopType {
    Outer = 0,
    Inner = 1,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BRepVertex {
    pub point_index: i32,
    pub edge_indices: Vec<i32>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BRepEdge {
    pub curve_3d_index: i32,
    pub start_vertex: i32,
    pub end_vertex: i32,
    pub trim_indices: Vec<i32>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BRepTrim {
    pub curve_2d_index: i32,
    pub edge_index: i32,
    pub loop_index: i32,
    pub reversed: bool,
    pub trim_type: BRepTrimType,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BRepLoop {
    pub trim_indices: Vec<i32>,
    pub face_index: i32,
    pub loop_type: BRepLoopType,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BRepFace {
    pub surface_index: i32,
    pub loop_indices: Vec<i32>,
    pub reversed: bool,
    pub facecolor: Option<Color>,
}

#[derive(Debug, Clone)]
pub struct BRep {
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub width: f64,
    pub surfacecolor: Color,
    pub xform: Xform,
    pub m_surfaces: Vec<NurbsSurface>,
    pub m_curves_3d: Vec<NurbsCurve>,
    pub m_curves_2d: Vec<NurbsCurve>,
    pub m_vertices: Vec<Point>,
    pub m_topology_vertices: Vec<BRepVertex>,
    pub m_topology_edges: Vec<BRepEdge>,
    pub m_trims: Vec<BRepTrim>,
    pub m_loops: Vec<BRepLoop>,
    pub m_faces: Vec<BRepFace>,
}

impl PartialEq for BRep {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && self.width == other.width
            && self.surfacecolor == other.surfacecolor
            && self.xform == other.xform
            && self.m_faces.len() == other.m_faces.len()
            && self.m_surfaces.len() == other.m_surfaces.len()
            && self.m_topology_edges.len() == other.m_topology_edges.len()
            && self.m_vertices.len() == other.m_vertices.len()
    }
}

impl Default for BRep {
    fn default() -> Self {
        Self::new()
    }
}

impl BRep {
    pub fn new() -> Self {
        BRep {
            guid: std::sync::OnceLock::new(),
            name: "my_brep".to_string(),
            width: 1.0,
            surfacecolor: Color::black(),
            xform: Xform::identity(),
            m_surfaces: Vec::new(),
            m_curves_3d: Vec::new(),
            m_curves_2d: Vec::new(),
            m_vertices: Vec::new(),
            m_topology_vertices: Vec::new(),
            m_topology_edges: Vec::new(),
            m_trims: Vec::new(),
            m_loops: Vec::new(),
            m_faces: Vec::new(),
        }
    }

    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = std::sync::OnceLock::new();
        copy
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Factory
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn create_box(sx: f64, sy: f64, sz: f64) -> Self {
        let mut brep = BRep::new();
        brep.name = "box".to_string();
        let hx = sx * 0.5;
        let hy = sy * 0.5;
        let hz = sz * 0.5;

        let corners = [
            Point::new(-hx, -hy, -hz), Point::new(hx, -hy, -hz),
            Point::new(hx, hy, -hz),   Point::new(-hx, hy, -hz),
            Point::new(-hx, -hy, hz),  Point::new(hx, -hy, hz),
            Point::new(hx, hy, hz),    Point::new(-hx, hy, hz),
        ];
        for c in &corners {
            brep.add_vertex(c);
        }

        let face_verts: [[usize; 4]; 6] = [
            [0, 3, 2, 1], // bottom
            [4, 5, 6, 7], // top
            [0, 1, 5, 4], // front
            [1, 2, 6, 5], // right
            [2, 3, 7, 6], // back
            [3, 0, 4, 7], // left
        ];

        let edge_verts: [[usize; 2]; 12] = [
            [0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7],
        ];

        for ev in &edge_verts {
            let p0 = corners[ev[0]].clone();
            let p1 = corners[ev[1]].clone();
            let line = NurbsCurve::create(false, 1, &[p0, p1]);
            brep.add_curve_3d(&line);
        }

        for i in 0..8 {
            brep.m_topology_vertices.push(BRepVertex {
                point_index: i as i32,
                edge_indices: Vec::new(),
            });
        }

        for i in 0..12 {
            brep.add_edge(i as i32, edge_verts[i][0] as i32, edge_verts[i][1] as i32);
        }

        let uv_corners = [
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ];

        for fi in 0..6 {
            let fv = &face_verts[fi];
            let p00 = corners[fv[0]].clone();
            let p10 = corners[fv[1]].clone();
            let p01 = corners[fv[3]].clone();
            let p11 = corners[fv[2]].clone();

            let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
            srf.set_cv(0, 0, &p00);
            srf.set_cv(1, 0, &p10);
            srf.set_cv(0, 1, &p01);
            srf.set_cv(1, 1, &p11);
            let si = brep.add_surface(&srf);

            let face_idx = brep.add_face(si as i32, false);
            let loop_idx = brep.add_loop(face_idx as i32, BRepLoopType::Outer);

            let find_edge = |v0: usize, v1: usize| -> i32 {
                for e in 0..12 {
                    if (edge_verts[e][0] == v0 && edge_verts[e][1] == v1)
                        || (edge_verts[e][0] == v1 && edge_verts[e][1] == v0)
                    {
                        return e as i32;
                    }
                }
                -1
            };

            for ei in 0..4 {
                let next = (ei + 1) % 4;
                let trim_crv = NurbsCurve::create(false, 1, &[uv_corners[ei].clone(), uv_corners[next].clone()]);
                let c2d_idx = brep.add_curve_2d(&trim_crv);
                let edge_idx = find_edge(fv[ei], fv[next]);
                let rev = edge_verts[edge_idx as usize][0] != fv[ei];
                brep.add_trim(c2d_idx as i32, edge_idx, loop_idx as i32, rev, BRepTrimType::Mated);
            }
        }

        for ei in 0..brep.m_topology_edges.len() {
            let sv = brep.m_topology_edges[ei].start_vertex as usize;
            let ev = brep.m_topology_edges[ei].end_vertex as usize;
            brep.m_topology_vertices[sv].edge_indices.push(ei as i32);
            brep.m_topology_vertices[ev].edge_indices.push(ei as i32);
        }

        brep
    }

    pub fn create_cylinder(radius: f64, height: f64) -> Self {
        use crate::primitives::Primitives;
        let mut brep = BRep::new();
        brep.name = "cylinder".to_string();
        let body = Primitives::cylinder_surface(0.0, 0.0, 0.0, radius, height);
        let dom_u = body.domain(0).unwrap();
        let dom_v = body.domain(1).unwrap();
        let p_bot = body.point_at(dom_u.0, dom_v.0).unwrap();
        let p_top = body.point_at(dom_u.0, dom_v.1).unwrap();
        let vi_bot = brep.add_vertex(&p_bot) as i32;
        let vi_top = brep.add_vertex(&p_top) as i32;
        brep.m_topology_vertices.push(BRepVertex { point_index: vi_bot, edge_indices: Vec::new() });
        brep.m_topology_vertices.push(BRepVertex { point_index: vi_top, edge_indices: Vec::new() });
        let circle_bot = Primitives::circle(0.0, 0.0, 0.0, radius);
        let circle_top = Primitives::circle(0.0, 0.0, height, radius);
        let seam_line = NurbsCurve::create(false, 1, &[p_bot, p_top]);
        let ci_bot = brep.add_curve_3d(&circle_bot) as i32;
        let ci_top = brep.add_curve_3d(&circle_top) as i32;
        let ci_seam = brep.add_curve_3d(&seam_line) as i32;
        let ei_bot = brep.add_edge(ci_bot, 0, 0) as i32;
        let ei_top = brep.add_edge(ci_top, 1, 1) as i32;
        let ei_seam = brep.add_edge(ci_seam, 0, 1) as i32;
        let si_body = brep.add_surface(&body) as i32;
        let mut cap_bot = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
        cap_bot.set_cv(0, 0, &Point::new(-radius, -radius, 0.0));
        cap_bot.set_cv(1, 0, &Point::new(radius, -radius, 0.0));
        cap_bot.set_cv(0, 1, &Point::new(-radius, radius, 0.0));
        cap_bot.set_cv(1, 1, &Point::new(radius, radius, 0.0));
        let si_bot = brep.add_surface(&cap_bot) as i32;
        let mut cap_top = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
        cap_top.set_cv(0, 0, &Point::new(-radius, -radius, height));
        cap_top.set_cv(1, 0, &Point::new(radius, -radius, height));
        cap_top.set_cv(0, 1, &Point::new(-radius, radius, height));
        cap_top.set_cv(1, 1, &Point::new(radius, radius, height));
        let si_top = brep.add_surface(&cap_top) as i32;
        let fi_body = brep.add_face(si_body, false) as i32;
        let li_body = brep.add_loop(fi_body, BRepLoopType::Outer) as i32;
        let c2d_bot = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.0, dom_v.0, 0.0), Point::new(dom_u.1, dom_v.0, 0.0),
        ]);
        let ci = brep.add_curve_2d(&c2d_bot) as i32;
        brep.add_trim(ci, ei_bot, li_body, false, BRepTrimType::Mated);
        let c2d_sr = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.1, dom_v.0, 0.0), Point::new(dom_u.1, dom_v.1, 0.0),
        ]);
        let ci = brep.add_curve_2d(&c2d_sr) as i32;
        brep.add_trim(ci, ei_seam, li_body, false, BRepTrimType::Seam);
        let c2d_top = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.1, dom_v.1, 0.0), Point::new(dom_u.0, dom_v.1, 0.0),
        ]);
        let ci = brep.add_curve_2d(&c2d_top) as i32;
        brep.add_trim(ci, ei_top, li_body, true, BRepTrimType::Mated);
        let c2d_sl = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.0, dom_v.1, 0.0), Point::new(dom_u.0, dom_v.0, 0.0),
        ]);
        let ci = brep.add_curve_2d(&c2d_sl) as i32;
        brep.add_trim(ci, ei_seam, li_body, true, BRepTrimType::Seam);
        // Circular 2D trim in UV space: circle at (0.5,0.5) radius 0.5
        let cw = (2.0_f64).sqrt() / 2.0;
        let ccx = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
        let ccy = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
        let cwt = [1.0, cw, 1.0, cw, 1.0, cw, 1.0, cw, 1.0];
        let make_cap_circle = || {
            let mut c = NurbsCurve::new(3, true, 3, 9);
            c.m_nurbsknot = vec![0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];
            for i in 0..9 {
                c.set_cv_4d(i, (0.5+0.5*ccx[i])*cwt[i], (0.5+0.5*ccy[i])*cwt[i], 0.0, cwt[i]);
            }
            c
        };
        let fi_bot = brep.add_face(si_bot, true) as i32;
        let li_bot = brep.add_loop(fi_bot, BRepLoopType::Outer) as i32;
        let ci = brep.add_curve_2d(&make_cap_circle()) as i32;
        brep.add_trim(ci, ei_bot, li_bot, true, BRepTrimType::Mated);
        let fi_top_f = brep.add_face(si_top, false) as i32;
        let li_top_l = brep.add_loop(fi_top_f, BRepLoopType::Outer) as i32;
        let ci = brep.add_curve_2d(&make_cap_circle()) as i32;
        brep.add_trim(ci, ei_top, li_top_l, false, BRepTrimType::Mated);
        for ei in 0..brep.m_topology_edges.len() {
            let sv = brep.m_topology_edges[ei].start_vertex as usize;
            let ev = brep.m_topology_edges[ei].end_vertex as usize;
            brep.m_topology_vertices[sv].edge_indices.push(ei as i32);
            brep.m_topology_vertices[ev].edge_indices.push(ei as i32);
        }
        brep
    }

    pub fn create_sphere(radius: f64) -> Self {
        use crate::primitives::Primitives;
        let mut brep = BRep::new();
        brep.name = "sphere".to_string();
        let srf = Primitives::sphere_surface(0.0, 0.0, 0.0, radius);
        let dom_u = srf.domain(0).unwrap();
        let dom_v = srf.domain(1).unwrap();
        let p_south = Point::new(0.0, 0.0, -radius);
        let p_north = Point::new(0.0, 0.0, radius);
        let vi_south = brep.add_vertex(&p_south) as i32;
        let vi_north = brep.add_vertex(&p_north) as i32;
        brep.m_topology_vertices.push(BRepVertex { point_index: vi_south, edge_indices: Vec::new() });
        brep.m_topology_vertices.push(BRepVertex { point_index: vi_north, edge_indices: Vec::new() });
        let seam_pts: Vec<Point> = {
            let n = 32;
            (0..=n).map(|i| {
                let v = dom_v.0 + i as f64 * (dom_v.1 - dom_v.0) / n as f64;
                srf.point_at(dom_u.0, v).unwrap_or(Point::new(0.0, 0.0, 0.0))
            }).collect()
        };
        let seam_crv = NurbsCurve::create(false, 1, &seam_pts);
        let ci_seam = brep.add_curve_3d(&seam_crv) as i32;
        let ei_seam = brep.add_edge(ci_seam, 0, 1) as i32;
        let si = brep.add_surface(&srf) as i32;
        let fi = brep.add_face(si, false) as i32;
        let li = brep.add_loop(fi, BRepLoopType::Outer) as i32;
        let c2d_south = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.0, dom_v.0, 0.0), Point::new(dom_u.1, dom_v.0, 0.0),
        ]);
        let ci = brep.add_curve_2d(&c2d_south) as i32;
        brep.add_trim(ci, -1, li, false, BRepTrimType::Singular);
        let c2d_sr = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.1, dom_v.0, 0.0), Point::new(dom_u.1, dom_v.1, 0.0),
        ]);
        let ci = brep.add_curve_2d(&c2d_sr) as i32;
        brep.add_trim(ci, ei_seam, li, false, BRepTrimType::Seam);
        let c2d_north = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.1, dom_v.1, 0.0), Point::new(dom_u.0, dom_v.1, 0.0),
        ]);
        let ci = brep.add_curve_2d(&c2d_north) as i32;
        brep.add_trim(ci, -1, li, false, BRepTrimType::Singular);
        let c2d_sl = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.0, dom_v.1, 0.0), Point::new(dom_u.0, dom_v.0, 0.0),
        ]);
        let ci = brep.add_curve_2d(&c2d_sl) as i32;
        brep.add_trim(ci, ei_seam, li, true, BRepTrimType::Seam);
        for ei in 0..brep.m_topology_edges.len() {
            let sv = brep.m_topology_edges[ei].start_vertex as usize;
            let ev = brep.m_topology_edges[ei].end_vertex as usize;
            brep.m_topology_vertices[sv].edge_indices.push(ei as i32);
            brep.m_topology_vertices[ev].edge_indices.push(ei as i32);
        }
        brep
    }

    pub fn create_block_with_hole(sx: f64, sy: f64, sz: f64, hole_radius: f64) -> Self {
        use crate::primitives::Primitives;
        let mut brep = BRep::new();
        brep.name = "block_with_hole".to_string();
        let hx = sx * 0.5; let hy = sy * 0.5; let hz = sz * 0.5;
        let corners = [
            Point::new(-hx, -hy, -hz),
            Point::new(hx, -hy, -hz),
            Point::new(hx, hy, -hz),
            Point::new(-hx, hy, -hz),
            Point::new(-hx, -hy, hz),
            Point::new(hx, -hy, hz),
            Point::new(hx, hy, hz),
            Point::new(-hx, hy, hz),
        ];
        for c in &corners { brep.add_vertex(c); }
        for i in 0..8 {
            brep.m_topology_vertices.push(BRepVertex { point_index: i as i32, edge_indices: vec![] });
        }
        let edge_verts: [[usize; 2]; 12] = [
            [0,1],[1,2],[2,3],[3,0],
            [4,5],[5,6],[6,7],[7,4],
            [0,4],[1,5],[2,6],[3,7],
        ];
        for ev in &edge_verts {
            let line = NurbsCurve::create(false, 1, &[corners[ev[0]].clone(), corners[ev[1]].clone()]);
            brep.add_curve_3d(&line);
        }
        for (i, ev) in edge_verts.iter().enumerate() {
            brep.add_edge(i as i32, ev[0] as i32, ev[1] as i32);
        }
        let side_faces = [[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]];
        let find_edge = |v0: usize, v1: usize| -> i32 {
            for e in 0..12 {
                if (edge_verts[e][0]==v0 && edge_verts[e][1]==v1) || (edge_verts[e][0]==v1 && edge_verts[e][1]==v0) {
                    return e as i32;
                }
            }
            -1
        };
        let uv_pts = [
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        for fv in &side_faces {
            let p00 = &corners[fv[0]]; let p10 = &corners[fv[1]];
            let p01 = &corners[fv[3]]; let p11 = &corners[fv[2]];
            let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
            srf.set_cv(0, 0, p00); srf.set_cv(1, 0, p10);
            srf.set_cv(0, 1, p01); srf.set_cv(1, 1, p11);
            let si = brep.add_surface(&srf) as i32;
            let face_idx = brep.add_face(si, false) as i32;
            let loop_idx = brep.add_loop(face_idx, BRepLoopType::Outer) as i32;
            for ei in 0..4 {
                let nxt = (ei + 1) % 4;
                let tc = NurbsCurve::create(false, 1, &[uv_pts[ei].clone(), uv_pts[nxt].clone()]);
                let c2d = brep.add_curve_2d(&tc) as i32;
                let eidx = find_edge(fv[ei], fv[nxt]);
                let rev = edge_verts[eidx as usize][0] != fv[ei];
                brep.add_trim(c2d, eidx, loop_idx, rev, BRepTrimType::Mated);
            }
        }
        let cyl_srf = Primitives::cylinder_surface(0.0, 0.0, -hz, hole_radius, sz);
        let dom_u = cyl_srf.domain(0).unwrap();
        let dom_v = cyl_srf.domain(1).unwrap();
        let si_cyl = brep.add_surface(&cyl_srf) as i32;
        let fi_cyl = brep.add_face(si_cyl, true) as i32;
        let li_cyl = brep.add_loop(fi_cyl, BRepLoopType::Outer) as i32;
        let circle_bot = Primitives::circle(0.0, 0.0, -hz, hole_radius);
        let circle_top = Primitives::circle(0.0, 0.0, hz, hole_radius);
        let seam_line = NurbsCurve::create(false, 1, &[
            Point::new(hole_radius, 0.0, -hz), Point::new(hole_radius, 0.0, hz),
        ]);
        let ci_bot = brep.add_curve_3d(&circle_bot) as i32;
        let ci_top = brep.add_curve_3d(&circle_top) as i32;
        let ci_seam = brep.add_curve_3d(&seam_line) as i32;
        let vi_seam_bot = brep.add_vertex(&Point::new(hole_radius, 0.0, -hz)) as i32;
        let vi_seam_top = brep.add_vertex(&Point::new(hole_radius, 0.0, hz)) as i32;
        brep.m_topology_vertices.push(BRepVertex { point_index: vi_seam_bot, edge_indices: vec![] });
        brep.m_topology_vertices.push(BRepVertex { point_index: vi_seam_top, edge_indices: vec![] });
        let ei_bot = brep.add_edge(ci_bot, 8, 8) as i32;
        let ei_top = brep.add_edge(ci_top, 9, 9) as i32;
        let ei_seam = brep.add_edge(ci_seam, 8, 9) as i32;
        let c2d = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.0, dom_v.0, 0.0), Point::new(dom_u.1, dom_v.0, 0.0),
        ]);
        let ci = brep.add_curve_2d(&c2d) as i32;
        brep.add_trim(ci, ei_bot, li_cyl, false, BRepTrimType::Mated);
        let c2d = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.1, dom_v.0, 0.0), Point::new(dom_u.1, dom_v.1, 0.0),
        ]);
        let ci = brep.add_curve_2d(&c2d) as i32;
        brep.add_trim(ci, ei_seam, li_cyl, false, BRepTrimType::Seam);
        let c2d = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.1, dom_v.1, 0.0), Point::new(dom_u.0, dom_v.1, 0.0),
        ]);
        let ci = brep.add_curve_2d(&c2d) as i32;
        brep.add_trim(ci, ei_top, li_cyl, true, BRepTrimType::Mated);
        let c2d = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.0, dom_v.1, 0.0), Point::new(dom_u.0, dom_v.0, 0.0),
        ]);
        let ci = brep.add_curve_2d(&c2d) as i32;
        brep.add_trim(ci, ei_seam, li_cyl, true, BRepTrimType::Seam);
        let cw = std::f64::consts::FRAC_1_SQRT_2;
        let ccx = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
        let ccy = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
        let cwt = [1.0, cw, 1.0, cw, 1.0, cw, 1.0, cw, 1.0];
        let ckn = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];
        let mut make_cap = |z: f64, reversed: bool, circle_edge_idx: i32| {
            let r = hx.max(hy);
            let mut cap = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
            cap.set_cv(0, 0, &Point::new(-r, -r, z)); cap.set_cv(1, 0, &Point::new(r, -r, z));
            cap.set_cv(0, 1, &Point::new(-r, r, z)); cap.set_cv(1, 1, &Point::new(r, r, z));
            let si = brep.add_surface(&cap) as i32;
            let fi = brep.add_face(si, reversed) as i32;
            let outer_li = brep.add_loop(fi, BRepLoopType::Outer) as i32;
            let fv: [usize; 4] = if z < 0.0 { [0,3,2,1] } else { [4,5,6,7] };
            for ei in 0..4 {
                let nxt = (ei + 1) % 4;
                let u0 = (corners[fv[ei]][0] + r) / (2.0 * r);
                let v0 = (corners[fv[ei]][1] + r) / (2.0 * r);
                let u1 = (corners[fv[nxt]][0] + r) / (2.0 * r);
                let v1 = (corners[fv[nxt]][1] + r) / (2.0 * r);
                let tc = NurbsCurve::create(false, 1, &[
                    Point::new(u0, v0, 0.0), Point::new(u1, v1, 0.0),
                ]);
                let c2d = brep.add_curve_2d(&tc) as i32;
                let eidx = find_edge(fv[ei], fv[nxt]);
                let rev = edge_verts[eidx as usize][0] != fv[ei];
                brep.add_trim(c2d, eidx, outer_li, rev, BRepTrimType::Mated);
            }
            let inner_li = brep.add_loop(fi, BRepLoopType::Inner) as i32;
            let mut hole_crv = NurbsCurve::new(3, true, 3, 9);
            for i in 0..10 { hole_crv.set_nurbsknot(i, ckn[i]); }
            let cr = hole_radius / (2.0 * r);
            let cx_uv = 0.5; let cy_uv = 0.5;
            for i in 0..9 {
                hole_crv.set_cv_4d(i, (cx_uv+cr*ccx[i])*cwt[i], (cy_uv+cr*ccy[i])*cwt[i], 0.0, cwt[i]);
            }
            let ci = brep.add_curve_2d(&hole_crv) as i32;
            brep.add_trim(ci, circle_edge_idx, inner_li, reversed, BRepTrimType::Mated);
        };
        make_cap(-hz, true, ei_bot);
        make_cap(hz, false, ei_top);
        for ei in 0..brep.m_topology_edges.len() {
            let sv = brep.m_topology_edges[ei].start_vertex as usize;
            let ev = brep.m_topology_edges[ei].end_vertex as usize;
            brep.m_topology_vertices[sv].edge_indices.push(ei as i32);
            brep.m_topology_vertices[ev].edge_indices.push(ei as i32);
        }
        brep
    }

    pub fn from_polylines(polylines: &[crate::polyline::Polyline]) -> Self {
        use std::collections::HashMap;
        let mut brep = BRep::new();
        brep.name = "polysurface".to_string();
        let tol = 1e-6;

        let find_or_add = |p: &Point, brep: &mut BRep| -> usize {
            for i in 0..brep.m_vertices.len() {
                let dx = p[0] - brep.m_vertices[i][0];
                let dy = p[1] - brep.m_vertices[i][1];
                let dz = p[2] - brep.m_vertices[i][2];
                if dx*dx + dy*dy + dz*dz < tol*tol { return i; }
            }
            let idx = brep.add_vertex(p);
            brep.m_topology_vertices.push(BRepVertex { point_index: idx as i32, edge_indices: Vec::new() });
            idx
        };

        let mut poly_vi: Vec<Vec<usize>> = Vec::new();
        for pl in polylines {
            let pts = pl.get_points();
            let n = if pl.is_closed() { pts.len() - 1 } else { pts.len() };
            let vi: Vec<usize> = (0..n).map(|i| find_or_add(&pts[i], &mut brep)).collect();
            poly_vi.push(vi);
        }

        let mut edge_map: HashMap<(usize, usize), usize> = HashMap::new();

        for (pi, pl) in polylines.iter().enumerate() {
            let vi = &poly_vi[pi];
            let n = vi.len();
            if n < 3 { continue; }

            let (_org, plane) = pl.get_fast_plane();
            if !plane.is_valid() { continue; }
            let org = plane.origin();
            let xa = plane.x_axis();
            let ya = plane.y_axis();

            let mut us: Vec<f64> = Vec::with_capacity(n);
            let mut vs: Vec<f64> = Vec::with_capacity(n);
            let mut umin = f64::MAX; let mut umax = f64::MIN;
            let mut vmin = f64::MAX; let mut vmax = f64::MIN;
            for i in 0..n {
                let dx = brep.m_vertices[vi[i]][0] - org[0];
                let dy = brep.m_vertices[vi[i]][1] - org[1];
                let dz = brep.m_vertices[vi[i]][2] - org[2];
                let u = dx*xa[0] + dy*xa[1] + dz*xa[2];
                let v = dx*ya[0] + dy*ya[1] + dz*ya[2];
                us.push(u); vs.push(v);
                umin = umin.min(u); umax = umax.max(u);
                vmin = vmin.min(v); vmax = vmax.max(v);
            }
            let pad = (umax - umin).max(vmax - vmin) * 0.01;
            umin -= pad; umax += pad; vmin -= pad; vmax += pad;
            let du = umax - umin; let dv = vmax - vmin;

            let pt3d = |u: f64, v: f64| -> Point {
                Point::new(org[0]+u*xa[0]+v*ya[0], org[1]+u*xa[1]+v*ya[1], org[2]+u*xa[2]+v*ya[2])
            };
            let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
            srf.set_cv(0, 0, &pt3d(umin, vmin)); srf.set_cv(1, 0, &pt3d(umax, vmin));
            srf.set_cv(0, 1, &pt3d(umin, vmax)); srf.set_cv(1, 1, &pt3d(umax, vmax));
            let si = brep.add_surface(&srf) as i32;
            let f_idx = brep.add_face(si, false) as i32;
            let l_idx = brep.add_loop(f_idx, BRepLoopType::Outer) as i32;

            for i in 0..n {
                let j = (i + 1) % n;
                let u0 = (us[i] - umin) / du; let v0 = (vs[i] - vmin) / dv;
                let u1 = (us[j] - umin) / du; let v1 = (vs[j] - vmin) / dv;
                let tc = NurbsCurve::create(false, 1, &[
                    Point::new(u0, v0, 0.0), Point::new(u1, v1, 0.0),
                ]);
                let c2d = brep.add_curve_2d(&tc) as i32;
                let lo = vi[i].min(vi[j]); let hi = vi[i].max(vi[j]);
                let (ei, rev) = if let Some(&existing) = edge_map.get(&(lo, hi)) {
                    (existing, vi[i] != lo)
                } else {
                    let line = NurbsCurve::create(false, 1, &[brep.m_vertices[vi[i]].clone(), brep.m_vertices[vi[j]].clone()]);
                    let ci = brep.add_curve_3d(&line) as i32;
                    let new_ei = brep.add_edge(ci, lo as i32, hi as i32);
                    edge_map.insert((lo, hi), new_ei);
                    (new_ei, vi[i] != lo)
                };
                let tt = if brep.m_topology_edges[ei].trim_indices.is_empty() {
                    BRepTrimType::Boundary
                } else {
                    for &ti in &brep.m_topology_edges[ei].trim_indices {
                        brep.m_trims[ti as usize].trim_type = BRepTrimType::Mated;
                    }
                    BRepTrimType::Mated
                };
                brep.add_trim(c2d, ei as i32, l_idx, rev, tt);
            }
        }

        for ei in 0..brep.m_topology_edges.len() {
            let sv = brep.m_topology_edges[ei].start_vertex as usize;
            let ev = brep.m_topology_edges[ei].end_vertex as usize;
            brep.m_topology_vertices[sv].edge_indices.push(ei as i32);
            brep.m_topology_vertices[ev].edge_indices.push(ei as i32);
        }
        brep
    }

    pub fn from_nurbscurves(curves: &[NurbsCurve], holes: &[Vec<NurbsCurve>]) -> Self {
        let mut brep = BRep::new();
        brep.name = "polysurface".to_string();
        let tol = 1e-6;

        let find_or_add = |p: &Point, brep: &mut BRep| -> usize {
            for i in 0..brep.m_vertices.len() {
                let dx = p[0] - brep.m_vertices[i][0];
                let dy = p[1] - brep.m_vertices[i][1];
                let dz = p[2] - brep.m_vertices[i][2];
                if dx*dx + dy*dy + dz*dz < tol*tol { return i; }
            }
            let idx = brep.add_vertex(p);
            brep.m_topology_vertices.push(BRepVertex { point_index: idx as i32, edge_indices: Vec::new() });
            idx
        };

        for (ci_idx, crv) in curves.iter().enumerate() {
            let pts = crv.divide_by_count(crv.cv_count().max(2) * 2, true).0;
            let n = if crv.is_closed() { pts.len() - 1 } else { pts.len() };
            if n < 3 { continue; }

            let pl = crate::polyline::Polyline::new(pts.clone());
            let (_org, plane) = pl.get_fast_plane();
            if !plane.is_valid() { continue; }
            let org = plane.origin();
            let xa = plane.x_axis();
            let ya = plane.y_axis();

            let mut us: Vec<f64> = Vec::with_capacity(n);
            let mut vs: Vec<f64> = Vec::with_capacity(n);
            let mut umin = f64::MAX; let mut umax = f64::MIN;
            let mut vmin = f64::MAX; let mut vmax = f64::MIN;
            for i in 0..n {
                let dx = pts[i][0] - org[0]; let dy = pts[i][1] - org[1]; let dz = pts[i][2] - org[2];
                let u = dx*xa[0]+dy*xa[1]+dz*xa[2];
                let v = dx*ya[0]+dy*ya[1]+dz*ya[2];
                us.push(u); vs.push(v);
                umin = umin.min(u); umax = umax.max(u);
                vmin = vmin.min(v); vmax = vmax.max(v);
            }
            if ci_idx < holes.len() {
                for hcrv in &holes[ci_idx] {
                    let hpts = hcrv.divide_by_count(hcrv.cv_count().max(2) * 2, true).0;
                    for hp in &hpts {
                        let dx = hp[0]-org[0]; let dy = hp[1]-org[1]; let dz = hp[2]-org[2];
                        let hu = dx*xa[0]+dy*xa[1]+dz*xa[2]; let hv = dx*ya[0]+dy*ya[1]+dz*ya[2];
                        umin = umin.min(hu); umax = umax.max(hu);
                        vmin = vmin.min(hv); vmax = vmax.max(hv);
                    }
                }
            }
            let pad = (umax - umin).max(vmax - vmin) * 0.01;
            umin -= pad; umax += pad; vmin -= pad; vmax += pad;
            let du = umax - umin; let dv = vmax - vmin;

            let pt3d = |u: f64, v: f64| -> Point {
                Point::new(org[0]+u*xa[0]+v*ya[0], org[1]+u*xa[1]+v*ya[1], org[2]+u*xa[2]+v*ya[2])
            };
            let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
            srf.set_cv(0, 0, &pt3d(umin, vmin)); srf.set_cv(1, 0, &pt3d(umax, vmin));
            srf.set_cv(0, 1, &pt3d(umin, vmax)); srf.set_cv(1, 1, &pt3d(umax, vmax));
            let si = brep.add_surface(&srf) as i32;
            let fi = brep.add_face(si, false) as i32;

            // Helper: project curve CVs to UV space
            let project_curve_to_uv = |crv: &NurbsCurve, org: &Point, xa: &crate::Vector, ya: &crate::Vector,
                                        umin: f64, vmin: f64, du: f64, dv: f64| -> NurbsCurve {
                let mut crv2d = NurbsCurve::new(3, crv.is_rational(), crv.order() as usize, crv.cv_count());
                crv2d.m_nurbsknot = crv.m_nurbsknot.clone();
                for i in 0..crv.cv_count() {
                    if crv.is_rational() {
                        let (wx, wy, wz, w) = crv.get_cv_4d(i).unwrap();
                        let (x, y, z) = (wx/w, wy/w, wz/w);
                        let (dx, dy, dz) = (x-org[0], y-org[1], z-org[2]);
                        let u = (dx*xa[0]+dy*xa[1]+dz*xa[2] - umin) / du;
                        let v = (dx*ya[0]+dy*ya[1]+dz*ya[2] - vmin) / dv;
                        crv2d.set_cv_4d(i, u*w, v*w, 0.0, w);
                    } else {
                        let cv = crv.get_cv(i).unwrap();
                        let (dx, dy, dz) = (cv[0]-org[0], cv[1]-org[1], cv[2]-org[2]);
                        let u = (dx*xa[0]+dy*xa[1]+dz*xa[2] - umin) / du;
                        let v = (dx*ya[0]+dy*ya[1]+dz*ya[2] - vmin) / dv;
                        crv2d.set_cv(i, &Point::new(u, v, 0.0));
                    }
                }
                crv2d
            };

            // Helper: add a full curve as a single loop
            let add_curve_loop = |crv: &NurbsCurve, face_idx: i32, loop_type: BRepLoopType,
                                   brep: &mut BRep| {
                let li = brep.add_loop(face_idx, loop_type) as i32;
                let ci3d = brep.add_curve_3d(crv) as i32;
                let crv2d = project_curve_to_uv(crv, &org, &xa, &ya, umin, vmin, du, dv);
                let c2d = brep.add_curve_2d(&crv2d) as i32;
                let dom = crv.domain();
                let sp = crv.point_at(dom.0);
                let ep = crv.point_at(dom.1);
                let vi_s = find_or_add(&sp, brep);
                let vi_e = if crv.is_closed() { vi_s } else { find_or_add(&ep, brep) };
                let lo = vi_s.min(vi_e) as i32; let hi = vi_s.max(vi_e) as i32;
                let ei = brep.add_edge(ci3d, lo, hi) as i32;
                brep.add_trim(c2d, ei, li, false, BRepTrimType::Boundary);
            };

            // Outer loop
            add_curve_loop(crv, fi, BRepLoopType::Outer, &mut brep);

            // Inner loops (holes)
            if ci_idx < holes.len() {
                for hcrv in &holes[ci_idx] {
                    add_curve_loop(hcrv, fi, BRepLoopType::Inner, &mut brep);
                }
            }
        }

        for ei in 0..brep.m_topology_edges.len() {
            let sv = brep.m_topology_edges[ei].start_vertex as usize;
            let ev = brep.m_topology_edges[ei].end_vertex as usize;
            brep.m_topology_vertices[sv].edge_indices.push(ei as i32);
            brep.m_topology_vertices[ev].edge_indices.push(ei as i32);
        }
        brep
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Accessors
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn face_count(&self) -> usize { self.m_faces.len() }
    pub fn edge_count(&self) -> usize { self.m_topology_edges.len() }
    pub fn vertex_count(&self) -> usize { self.m_vertices.len() }

    pub fn is_valid(&self) -> bool {
        if self.m_faces.is_empty() || self.m_surfaces.is_empty() || self.m_vertices.is_empty() { return false; }
        for f in &self.m_faces {
            if f.surface_index < 0 || f.surface_index as usize >= self.m_surfaces.len() { return false; }
        }
        for l in &self.m_loops {
            if l.face_index < 0 || l.face_index as usize >= self.m_faces.len() { return false; }
        }
        for t in &self.m_trims {
            if t.curve_2d_index < 0 || t.curve_2d_index as usize >= self.m_curves_2d.len() { return false; }
            if t.loop_index < 0 || t.loop_index as usize >= self.m_loops.len() { return false; }
        }
        for e in &self.m_topology_edges {
            if e.start_vertex < 0 || e.start_vertex as usize >= self.m_topology_vertices.len() { return false; }
            if e.end_vertex < 0 || e.end_vertex as usize >= self.m_topology_vertices.len() { return false; }
        }
        true
    }

    pub fn is_solid(&self) -> bool {
        if self.m_topology_edges.is_empty() { return false; }
        for e in &self.m_topology_edges {
            if e.trim_indices.len() != 2 { return false; }
        }
        true
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Building
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_surface(&mut self, srf: &NurbsSurface) -> usize {
        self.m_surfaces.push(srf.clone());
        self.m_surfaces.len() - 1
    }

    pub fn add_curve_3d(&mut self, crv: &NurbsCurve) -> usize {
        self.m_curves_3d.push(crv.clone());
        self.m_curves_3d.len() - 1
    }

    pub fn add_curve_2d(&mut self, crv: &NurbsCurve) -> usize {
        self.m_curves_2d.push(crv.clone());
        self.m_curves_2d.len() - 1
    }

    pub fn add_vertex(&mut self, pt: &Point) -> usize {
        self.m_vertices.push(pt.clone());
        self.m_vertices.len() - 1
    }

    pub fn add_edge(&mut self, curve_3d_idx: i32, start_vertex: i32, end_vertex: i32) -> usize {
        self.m_topology_edges.push(BRepEdge {
            curve_3d_index: curve_3d_idx,
            start_vertex,
            end_vertex,
            trim_indices: Vec::new(),
        });
        self.m_topology_edges.len() - 1
    }

    pub fn add_trim(&mut self, curve_2d_idx: i32, edge_idx: i32, loop_idx: i32, reversed: bool, trim_type: BRepTrimType) -> usize {
        let idx = self.m_trims.len();
        self.m_trims.push(BRepTrim {
            curve_2d_index: curve_2d_idx,
            edge_index: edge_idx,
            loop_index: loop_idx,
            reversed,
            trim_type,
        });
        if loop_idx >= 0 && (loop_idx as usize) < self.m_loops.len() {
            self.m_loops[loop_idx as usize].trim_indices.push(idx as i32);
        }
        if edge_idx >= 0 && (edge_idx as usize) < self.m_topology_edges.len() {
            self.m_topology_edges[edge_idx as usize].trim_indices.push(idx as i32);
        }
        idx
    }

    pub fn add_loop(&mut self, face_idx: i32, loop_type: BRepLoopType) -> usize {
        let idx = self.m_loops.len();
        self.m_loops.push(BRepLoop {
            trim_indices: Vec::new(),
            face_index: face_idx,
            loop_type,
        });
        if face_idx >= 0 && (face_idx as usize) < self.m_faces.len() {
            self.m_faces[face_idx as usize].loop_indices.push(idx as i32);
        }
        idx
    }

    pub fn add_face(&mut self, surface_idx: i32, reversed: bool) -> usize {
        self.m_faces.push(BRepFace {
            surface_index: surface_idx,
            loop_indices: Vec::new(),
            reversed,
            facecolor: None,
        });
        self.m_faces.len() - 1
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Splitting
    ///////////////////////////////////////////////////////////////////////////////////////////

    fn q6(x: f64) -> i64 {
        (x * 1_000_000.0).round() as i64
    }

    fn lift_loop(srf: &NurbsSurface, pc: &NurbsCurve) -> (NurbsCurve, Point, Point, Point) {
        let n = (pc.cv_count() * 4).max(8);
        let (c0, c1) = pc.domain();
        let mut pts3: Vec<Point> = Vec::with_capacity(n + 1);
        for i in 0..=n {
            let uv = pc.point_at(c0 + (c1 - c0) * (i as f64) / (n as f64));
            let p = srf.point_at(uv[0], uv[1]).unwrap_or(Point::new(uv[0], uv[1], 0.0));
            pts3.push(p);
        }
        let c3d = NurbsCurve::create(false, 1, &pts3);
        let p0 = pts3[0].clone();
        let p1 = pts3[n].clone();
        let pm = pts3[n / 2].clone();
        (c3d, p0, p1, pm)
    }

    fn find_or_add_vertex(result: &mut BRep, vmap: &mut std::collections::HashMap<(i64, i64, i64), i32>, p: &Point) -> i32 {
        let key = (Self::q6(p[0]), Self::q6(p[1]), Self::q6(p[2]));
        if let Some(&idx) = vmap.get(&key) {
            return idx;
        }
        let idx = result.add_vertex(p) as i32;
        result.m_topology_vertices.push(BRepVertex { point_index: idx, edge_indices: Vec::new() });
        vmap.insert(key, idx);
        idx
    }

    fn append_face(result: &mut BRep,
                   vmap: &mut std::collections::HashMap<(i64, i64, i64), i32>,
                   emap: &mut std::collections::HashMap<(i32, i32, i64, i64, i64), i32>,
                   srf: &NurbsSurface,
                   loops: &[(BRepLoopType, Vec<NurbsCurve>)]) {
        let si = result.add_surface(srf) as i32;
        let fi = result.add_face(si, false) as i32;
        for (ltype, pcs) in loops {
            let li = result.add_loop(fi, *ltype) as i32;
            for pc in pcs {
                if !pc.is_valid() {
                    continue;
                }
                let (c3d, p0, p1, pm) = Self::lift_loop(srf, pc);
                let ci3d = result.add_curve_3d(&c3d) as i32;
                let va = Self::find_or_add_vertex(result, vmap, &p0);
                let vb = Self::find_or_add_vertex(result, vmap, &p1);
                let (lo, hi) = if va <= vb { (va, vb) } else { (vb, va) };
                let ekey = (lo, hi, Self::q6(pm[0]), Self::q6(pm[1]), Self::q6(pm[2]));
                let (ei, ttype) = if let Some(&prior) = emap.get(&ekey) {
                    (prior, BRepTrimType::Mated)
                } else {
                    let ne = result.add_edge(ci3d, lo, hi) as i32;
                    emap.insert(ekey, ne);
                    (ne, BRepTrimType::Boundary)
                };
                let ci2d = result.add_curve_2d(pc) as i32;
                result.add_trim(ci2d, ei, li, false, ttype);
            }
        }
    }

    fn split_with<F: Fn(&NurbsSurface) -> Vec<NurbsCurve>>(&self, tolerance: Option<f64>, cut_for: F) -> BRep {
        use crate::nurbssurface_trimmed::NurbsSurfaceTrimmed;
        let mut result = BRep::new();
        result.name = self.name.clone();
        let mut vmap: std::collections::HashMap<(i64, i64, i64), i32> = std::collections::HashMap::new();
        let mut emap: std::collections::HashMap<(i32, i32, i64, i64, i64), i32> = std::collections::HashMap::new();

        for face in &self.m_faces {
            if face.surface_index < 0 || face.surface_index as usize >= self.m_surfaces.len() {
                continue;
            }
            let srf = &self.m_surfaces[face.surface_index as usize];
            let mut outer_pcs: Vec<NurbsCurve> = Vec::new();
            let mut inner_loops: Vec<Vec<NurbsCurve>> = Vec::new();
            let mut has_inner = false;
            for &li in &face.loop_indices {
                if li < 0 || li as usize >= self.m_loops.len() {
                    continue;
                }
                let bloop = &self.m_loops[li as usize];
                let mut pcs: Vec<NurbsCurve> = Vec::new();
                for &ti in &bloop.trim_indices {
                    if ti < 0 || ti as usize >= self.m_trims.len() {
                        continue;
                    }
                    let c2 = self.m_trims[ti as usize].curve_2d_index;
                    if c2 >= 0 && (c2 as usize) < self.m_curves_2d.len() {
                        pcs.push(self.m_curves_2d[c2 as usize].clone());
                    }
                }
                if bloop.loop_type == BRepLoopType::Inner {
                    has_inner = true;
                    inner_loops.push(pcs);
                } else {
                    outer_pcs = pcs;
                }
            }

            let cut_pcs = cut_for(srf);
            if cut_pcs.is_empty() || has_inner {
                let mut loops: Vec<(BRepLoopType, Vec<NurbsCurve>)> = vec![(BRepLoopType::Outer, outer_pcs)];
                for il in inner_loops {
                    loops.push((BRepLoopType::Inner, il));
                }
                Self::append_face(&mut result, &mut vmap, &mut emap, srf, &loops);
                continue;
            }

            let n_boundary = outer_pcs.len();
            let mut all_pcs = outer_pcs.clone();
            all_pcs.extend(cut_pcs);
            let parts = NurbsSurfaceTrimmed::split_by_uv_curves_ex(srf, &all_pcs, tolerance, false, n_boundary);
            if parts.len() <= 1 {
                let loops: Vec<(BRepLoopType, Vec<NurbsCurve>)> = vec![(BRepLoopType::Outer, outer_pcs)];
                Self::append_face(&mut result, &mut vmap, &mut emap, srf, &loops);
                continue;
            }
            for part in &parts {
                let mut loops: Vec<(BRepLoopType, Vec<NurbsCurve>)> = Vec::new();
                if let Some(ol) = &part.m_outer_loop {
                    loops.push((BRepLoopType::Outer, vec![ol.clone()]));
                }
                for il in &part.m_inner_loops {
                    loops.push((BRepLoopType::Inner, vec![il.clone()]));
                }
                Self::append_face(&mut result, &mut vmap, &mut emap, &part.m_surface, &loops);
            }
        }

        for ei in 0..result.m_topology_edges.len() {
            let (sv, ev) = (result.m_topology_edges[ei].start_vertex, result.m_topology_edges[ei].end_vertex);
            if sv >= 0 && (sv as usize) < result.m_topology_vertices.len() {
                result.m_topology_vertices[sv as usize].edge_indices.push(ei as i32);
            }
            if ev != sv && ev >= 0 && (ev as usize) < result.m_topology_vertices.len() {
                result.m_topology_vertices[ev as usize].edge_indices.push(ei as i32);
            }
        }
        result
    }

    /// Split this BRep by a plane. Returns a new subdivided BRep.
    pub fn split_by_plane(&self, plane: &crate::plane::Plane, tolerance: Option<f64>) -> BRep {
        self.split_with(tolerance, |srf| {
            crate::intersection::surface_plane_uv(srf, plane, tolerance)
                .into_iter().map(|(_c3, pc)| pc).collect()
        })
    }

    /// Split this BRep by another surface. Returns a new subdivided BRep.
    pub fn split_by_surface(&self, cutter: &NurbsSurface, tolerance: Option<f64>) -> BRep {
        let cutter_bb = aabb_from_surface(cutter);
        self.split_with(tolerance, |srf| {
            let srf_bb = aabb_from_surface(srf);
            let margin = (srf_bb.1[0] - srf_bb.0[0]).max(srf_bb.1[1] - srf_bb.0[1]).max(srf_bb.1[2] - srf_bb.0[2]) * 1e-3;
            if !aabb_overlap(&srf_bb, &cutter_bb, margin) {
                return Vec::new();
            }
            crate::intersection::surface_surface(srf, cutter, tolerance)
                .into_iter().map(|(_c3, pa, _pb)| pa).collect()
        })
    }

    /// Split this BRep by 3D curves pulled onto each face. New BRep.
    pub fn split_by_curves(&self, curves: &[NurbsCurve], tolerance: Option<f64>) -> BRep {
        let curve_bbs: Vec<([f64; 3], [f64; 3])> = curves.iter().map(|c| aabb_from_curve(c)).collect();
        self.split_with(tolerance, |srf| {
            let srf_bb = aabb_from_surface(srf);
            let margin = (srf_bb.1[0] - srf_bb.0[0]).max(srf_bb.1[1] - srf_bb.0[1]).max(srf_bb.1[2] - srf_bb.0[2]) * 1e-3;
            let mut out: Vec<NurbsCurve> = Vec::new();
            for (crv, cbb) in curves.iter().zip(curve_bbs.iter()) {
                if !aabb_overlap(&srf_bb, cbb, margin) {
                    continue;
                }
                for pc in crate::closest::Closest::surface_curve(srf, crv, 0.0, 0.0, tolerance) {
                    out.push(pc);
                }
            }
            out
        })
    }

    /// Split this BRep by a line pulled onto each face. New BRep.
    pub fn split_by_line(&self, line: &crate::line::Line, tolerance: Option<f64>) -> BRep {
        let pts = [line.start(), line.end()];
        let crv = NurbsCurve::create(false, 1, &pts);
        self.split_by_curves(&[crv], tolerance)
    }

    fn sub_map_surface(sub: &mut BRep, m: &mut std::collections::HashMap<i32, i32>, src: &BRep, i: i32) -> i32 {
        if let Some(&x) = m.get(&i) { return x; }
        let x = sub.add_surface(&src.m_surfaces[i as usize]) as i32;
        m.insert(i, x);
        x
    }
    fn sub_map_c2(sub: &mut BRep, m: &mut std::collections::HashMap<i32, i32>, src: &BRep, i: i32) -> i32 {
        if i < 0 || i as usize >= src.m_curves_2d.len() { return -1; }
        if let Some(&x) = m.get(&i) { return x; }
        let x = sub.add_curve_2d(&src.m_curves_2d[i as usize]) as i32;
        m.insert(i, x);
        x
    }
    fn sub_map_vertex(sub: &mut BRep, m: &mut std::collections::HashMap<i32, i32>, src: &BRep, i: i32) -> i32 {
        if i < 0 || i as usize >= src.m_topology_vertices.len() { return -1; }
        if let Some(&x) = m.get(&i) { return x; }
        let pt = src.m_vertices[src.m_topology_vertices[i as usize].point_index as usize].clone();
        let idx = sub.add_vertex(&pt) as i32;
        sub.m_topology_vertices.push(BRepVertex { point_index: idx, edge_indices: Vec::new() });
        let nv = (sub.m_topology_vertices.len() - 1) as i32;
        m.insert(i, nv);
        nv
    }
    fn sub_map_edge(sub: &mut BRep, e_map: &mut std::collections::HashMap<i32, i32>,
                    c3_map: &mut std::collections::HashMap<i32, i32>,
                    v_map: &mut std::collections::HashMap<i32, i32>, src: &BRep, i: i32) -> i32 {
        if i < 0 || i as usize >= src.m_topology_edges.len() { return -1; }
        if let Some(&x) = e_map.get(&i) { return x; }
        let e = src.m_topology_edges[i as usize].clone();
        let mut ci3 = -1;
        if e.curve_3d_index >= 0 && (e.curve_3d_index as usize) < src.m_curves_3d.len() {
            ci3 = match c3_map.get(&e.curve_3d_index) {
                Some(&x) => x,
                None => {
                    let x = sub.add_curve_3d(&src.m_curves_3d[e.curve_3d_index as usize]) as i32;
                    c3_map.insert(e.curve_3d_index, x);
                    x
                }
            };
        }
        let sv = Self::sub_map_vertex(sub, v_map, src, e.start_vertex);
        let ev = Self::sub_map_vertex(sub, v_map, src, e.end_vertex);
        let ne = sub.add_edge(ci3, sv, ev) as i32;
        e_map.insert(i, ne);
        ne
    }

    /// Build a standalone BRep from a subset of this BRep's faces.
    pub fn subset(&self, face_indices: &[usize]) -> BRep {
        let mut sub = BRep::new();
        sub.name = self.name.clone();
        let mut s_map = std::collections::HashMap::new();
        let mut c2_map = std::collections::HashMap::new();
        let mut c3_map = std::collections::HashMap::new();
        let mut v_map = std::collections::HashMap::new();
        let mut e_map = std::collections::HashMap::new();
        for &fi in face_indices {
            let face = &self.m_faces[fi];
            let si = Self::sub_map_surface(&mut sub, &mut s_map, self, face.surface_index);
            let new_fi = sub.add_face(si, face.reversed) as i32;
            for &li in &face.loop_indices {
                let lp = self.m_loops[li as usize].clone();
                let new_li = sub.add_loop(new_fi, lp.loop_type) as i32;
                for &ti in &lp.trim_indices {
                    let trim = self.m_trims[ti as usize].clone();
                    let ci2 = Self::sub_map_c2(&mut sub, &mut c2_map, self, trim.curve_2d_index);
                    let ei = Self::sub_map_edge(&mut sub, &mut e_map, &mut c3_map, &mut v_map, self, trim.edge_index);
                    sub.add_trim(ci2, ei, new_li, trim.reversed, trim.trim_type);
                }
            }
        }
        for ei in 0..sub.m_topology_edges.len() {
            let (sv, ev) = (sub.m_topology_edges[ei].start_vertex, sub.m_topology_edges[ei].end_vertex);
            if sv >= 0 && (sv as usize) < sub.m_topology_vertices.len() {
                sub.m_topology_vertices[sv as usize].edge_indices.push(ei as i32);
            }
            if ev != sv && ev >= 0 && (ev as usize) < sub.m_topology_vertices.len() {
                sub.m_topology_vertices[ev as usize].edge_indices.push(ei as i32);
            }
        }
        sub
    }

    /// Split this BRep by a plane and separate the result into the pieces on
    /// each side of the plane. Returns one BRep per side.
    pub fn split_by_plane_pieces(&self, plane: &crate::plane::Plane, tolerance: Option<f64>) -> Vec<BRep> {
        let whole = self.split_by_plane(plane, tolerance);
        let o = plane.origin();
        let n = plane.z_axis();
        let mut pos: Vec<usize> = Vec::new();
        let mut neg: Vec<usize> = Vec::new();
        for (fi, face) in whole.m_faces.iter().enumerate() {
            let srf = &whole.m_surfaces[face.surface_index as usize];
            let (mut sx, mut sy, mut sz, mut cnt) = (0.0, 0.0, 0.0, 0i32);
            for &li in &face.loop_indices {
                let lp = &whole.m_loops[li as usize];
                if lp.loop_type != BRepLoopType::Outer { continue; }
                for &ti in &lp.trim_indices {
                    let pc = &whole.m_curves_2d[whole.m_trims[ti as usize].curve_2d_index as usize];
                    let (d0, d1) = pc.domain();
                    for k in 0..8 {
                        let uv = pc.point_at(d0 + (d1 - d0) * k as f64 / 8.0);
                        let p = srf.point_at(uv[0], uv[1]).unwrap_or(Point::new(0.0, 0.0, 0.0));
                        sx += p[0]; sy += p[1]; sz += p[2]; cnt += 1;
                    }
                }
            }
            if cnt == 0 { continue; }
            let (cx, cy, cz) = (sx / cnt as f64, sy / cnt as f64, sz / cnt as f64);
            let d = (cx - o[0]) * n[0] + (cy - o[1]) * n[1] + (cz - o[2]) * n[2];
            if d >= 0.0 { pos.push(fi); } else { neg.push(fi); }
        }
        let mut pieces = Vec::new();
        for idxs in [pos, neg] {
            if !idxs.is_empty() {
                pieces.push(whole.subset(&idxs));
            }
        }
        pieces
    }

    /// Split this BRep by every face of another BRep. New BRep.
    pub fn split_by_brep(&self, cutter: &BRep, tolerance: Option<f64>) -> BRep {
        let cutter_bbs: Vec<([f64; 3], [f64; 3])> = cutter.m_surfaces.iter().map(|cs| aabb_from_surface(cs)).collect();
        self.split_with(tolerance, |srf| {
            let srf_bb = aabb_from_surface(srf);
            let margin = (srf_bb.1[0] - srf_bb.0[0]).max(srf_bb.1[1] - srf_bb.0[1]).max(srf_bb.1[2] - srf_bb.0[2]) * 1e-3;
            let mut out: Vec<NurbsCurve> = Vec::new();
            for (cs, cbb) in cutter.m_surfaces.iter().zip(cutter_bbs.iter()) {
                if !aabb_overlap(&srf_bb, cbb, margin) {
                    continue;
                }
                for pc in crate::intersection::cut_curves_on_surface(srf, cs, tolerance) {
                    out.push(pc);
                }
            }
            out
        })
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Meshing
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Returns one tessellated `Mesh` per BRep face, in face order.
    /// Trimmed faces use CDT tessellation; reversed faces have normals flipped.
    /// Vertices are NOT shared across faces so face boundaries are hard edges.
    pub fn face_meshes(&self) -> Vec<Mesh> {
        self.face_meshes_q(None)
    }

    /// Per-face meshes with an optional tessellation-quality override applied to the
    /// grid-meshed (direct) faces: `Some((max_angle_deg, chord_factor))` densifies them
    /// (and, via the shared-edge coordination, the CDT faces follow). `None` keeps the
    /// default `NurbsSurface::mesh()` density.
    pub fn face_meshes_q(&self, quality: Option<(f64, f64)>) -> Vec<Mesh> {
        use crate::nurbssurface_trimmed::NurbsSurfaceTrimmed;
        let nf = self.m_faces.len();

        // Phase 1: classify
        let mut face_direct = vec![false; nf];
        for fi in 0..nf {
            let face = &self.m_faces[fi];
            if face.surface_index < 0 || face.surface_index as usize >= self.m_surfaces.len() { continue; }
            let srf = &self.m_surfaces[face.surface_index as usize];
            let mut has_inner = false;
            let mut all_linear = true;
            let mut outer_pts: Vec<Point> = Vec::new();
            for &li in &face.loop_indices {
                if li < 0 || li as usize >= self.m_loops.len() { continue; }
                let bloop = &self.m_loops[li as usize];
                if bloop.loop_type == BRepLoopType::Inner { has_inner = true; }
                for &ti in &bloop.trim_indices {
                    if ti < 0 || ti as usize >= self.m_trims.len() { continue; }
                    let trim = &self.m_trims[ti as usize];
                    if trim.curve_2d_index < 0 || trim.curve_2d_index as usize >= self.m_curves_2d.len() { continue; }
                    let crv = &self.m_curves_2d[trim.curve_2d_index as usize];
                    if crv.degree() > 1 || crv.is_rational() { all_linear = false; }
                    if bloop.loop_type == BRepLoopType::Outer && crv.degree() <= 1 && !crv.is_rational() {
                        for k in 0..crv.cv_count().saturating_sub(1) {
                            if let Some(p) = crv.get_cv(k) { outer_pts.push(p); }
                        }
                    }
                }
            }
            let mut direct = !has_inner && all_linear;
            if direct && !outer_pts.is_empty() {
                if let (Some((u0, u1)), Some((v0, v1))) = (srf.domain(0), srf.domain(1)) {
                    let tol = (u1 - u0).max(v1 - v0) * 0.01;
                    let mut bb_umin = f64::INFINITY; let mut bb_umax = f64::NEG_INFINITY;
                    let mut bb_vmin = f64::INFINITY; let mut bb_vmax = f64::NEG_INFINITY;
                    for p in &outer_pts {
                        if p[0] < bb_umin { bb_umin = p[0]; }
                        if p[0] > bb_umax { bb_umax = p[0]; }
                        if p[1] < bb_vmin { bb_vmin = p[1]; }
                        if p[1] > bb_vmax { bb_vmax = p[1]; }
                    }
                    if (bb_umin - u0).abs() > tol || (bb_umax - u1).abs() > tol ||
                       (bb_vmin - v0).abs() > tol || (bb_vmax - v1).abs() > tol {
                        direct = false;
                    }
                }
            }
            face_direct[fi] = direct;
        }

        // Phase 2: direct faces. Mesh each via the grid mesher, then record the 3D
        // boundary discretisation along every edge shared with a CDT face, so the CDT
        // face can reuse the exact same points → watertight seams (mirrors C++ BRep::mesh).
        let mut fmesh: Vec<Mesh> = (0..nf).map(|_| Mesh::new()).collect();
        let mut edge_bnd: std::collections::HashMap<i32, Vec<Point>> = std::collections::HashMap::new();
        for fi in 0..nf {
            if !face_direct[fi] { continue; }
            let face = &self.m_faces[fi];
            let srf = &self.m_surfaces[face.surface_index as usize];
            fmesh[fi] = match quality {
                Some((a, c)) => crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid::from_u_v_q(srf.clone(), 0, 0, a, c),
                None => srf.mesh(),
            };
            let (u0, u1) = match srf.domain(0) { Some(d) => d, None => continue };
            let (v0, v1) = match srf.domain(1) { Some(d) => d, None => continue };
            let utol = (u1 - u0) * 0.001;
            let vtol = (v1 - v0) * 0.001;
            for &li in &face.loop_indices {
                if li < 0 || li as usize >= self.m_loops.len() { continue; }
                for &ti in &self.m_loops[li as usize].trim_indices {
                    if ti < 0 || ti as usize >= self.m_trims.len() { continue; }
                    let eidx = self.m_trims[ti as usize].edge_index;
                    if eidx < 0 || eidx as usize >= self.m_topology_edges.len() { continue; }
                    if edge_bnd.contains_key(&eidx) { continue; }
                    // Only extract if this edge is shared with a CDT (non-direct) face.
                    let mut shared = false;
                    for &oti in &self.m_topology_edges[eidx as usize].trim_indices {
                        if oti == ti || oti < 0 || oti as usize >= self.m_trims.len() { continue; }
                        let oli = self.m_trims[oti as usize].loop_index;
                        if oli < 0 || oli as usize >= self.m_loops.len() { continue; }
                        let ofi = self.m_loops[oli as usize].face_index;
                        if ofi >= 0 && (ofi as usize) < nf && !face_direct[ofi as usize] { shared = true; break; }
                    }
                    if !shared { continue; }
                    // Which UV boundary (u0/u1/v0/v1) does this trim lie on?
                    let c2di = self.m_trims[ti as usize].curve_2d_index;
                    if c2di < 0 || c2di as usize >= self.m_curves_2d.len() { continue; }
                    let c2d = &self.m_curves_2d[c2di as usize];
                    let (sp, ep) = match (c2d.get_cv(0), c2d.get_cv(c2d.cv_count().saturating_sub(1))) {
                        (Some(a), Some(b)) => (a, b),
                        _ => continue,
                    };
                    let at_v0 = (sp[1] - v0).abs() < vtol && (ep[1] - v0).abs() < vtol;
                    let at_v1 = (sp[1] - v1).abs() < vtol && (ep[1] - v1).abs() < vtol;
                    let at_u0 = (sp[0] - u0).abs() < utol && (ep[0] - u0).abs() < utol;
                    let at_u1 = (sp[0] - u1).abs() < utol && (ep[0] - u1).abs() < utol;
                    if !at_v0 && !at_v1 && !at_u0 && !at_u1 { continue; }
                    // Collect the grid vertices that sit on that boundary, keyed by the
                    // along-edge parameter (the u/v attributes stored by the grid mesher).
                    let mut pts: Vec<(f64, Point)> = Vec::new();
                    for (_, vd) in fmesh[fi].vertex.iter() {
                        let (iu, iv) = match (vd.attributes.get("u"), vd.attributes.get("v")) {
                            (Some(&a), Some(&b)) => (a, b),
                            _ => continue,
                        };
                        if at_v0 && (iv - v0).abs() < vtol * 0.1 { pts.push((iu, vd.position())); }
                        else if at_v1 && (iv - v1).abs() < vtol * 0.1 { pts.push((iu, vd.position())); }
                        else if at_u0 && (iu - u0).abs() < utol * 0.1 { pts.push((iv, vd.position())); }
                        else if at_u1 && (iu - u1).abs() < utol * 0.1 { pts.push((iv, vd.position())); }
                    }
                    pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
                    if pts.len() >= 2 {
                        edge_bnd.insert(eidx, pts.into_iter().map(|(_, p)| p).collect());
                    }
                }
            }
        }

        // Phase 3: Mesh CDT faces via NurbsSurfaceTrimmed (Bowyer-Watson CDT). For edges
        // shared with a direct face, reuse that face's boundary points (projected into this
        // bilinear face's UV) so the seam vertices coincide exactly.
        for fi in 0..nf {
            if face_direct[fi] { continue; }
            let face = &self.m_faces[fi];
            if face.surface_index < 0 || face.surface_index as usize >= self.m_surfaces.len() { continue; }
            let srf = &self.m_surfaces[face.surface_index as usize];
            // Bilinear 3D→UV projection frame (valid for the planar cap surfaces).
            let proj: Option<(Point, [f64; 3], [f64; 3], f64, f64)> =
                match (srf.get_cv(0, 0), srf.get_cv(1, 0), srf.get_cv(0, 1)) {
                    (Some(a), Some(b), Some(c)) if srf.degree(0) == 1 && srf.degree(1) == 1 => {
                        let eu = [b[0]-a[0], b[1]-a[1], b[2]-a[2]];
                        let ev = [c[0]-a[0], c[1]-a[1], c[2]-a[2]];
                        let eu2 = eu[0]*eu[0] + eu[1]*eu[1] + eu[2]*eu[2];
                        let ev2 = ev[0]*ev[0] + ev[1]*ev[1] + ev[2]*ev[2];
                        if eu2 > 1e-28 && ev2 > 1e-28 { Some((a, eu, ev, eu2, ev2)) } else { None }
                    }
                    _ => None,
                };
            let mut ts = NurbsSurfaceTrimmed::new();
            ts.m_surface = srf.clone();
            for &li in &face.loop_indices {
                if li < 0 || li as usize >= self.m_loops.len() { continue; }
                let bloop = &self.m_loops[li as usize];
                let mut loop_pts: Vec<Point> = Vec::new();
                for &ti in &bloop.trim_indices {
                    if ti < 0 || ti as usize >= self.m_trims.len() { continue; }
                    let trim = &self.m_trims[ti as usize];
                    if trim.trim_type == BRepTrimType::Singular { continue; }
                    let eidx = trim.edge_index;
                    if proj.is_some() && eidx >= 0 && edge_bnd.contains_key(&eidx) {
                        let (a, eu, ev, eu2, ev2) = proj.as_ref().unwrap();
                        for pt in &edge_bnd[&eidx] {
                            let d = [pt[0]-a[0], pt[1]-a[1], pt[2]-a[2]];
                            let u = (d[0]*eu[0] + d[1]*eu[1] + d[2]*eu[2]) / *eu2;
                            let v = (d[0]*ev[0] + d[1]*ev[1] + d[2]*ev[2]) / *ev2;
                            loop_pts.push(Point::new(u, v, 0.0));
                        }
                    } else {
                        if trim.curve_2d_index < 0 || trim.curve_2d_index as usize >= self.m_curves_2d.len() { continue; }
                        let crv = &self.m_curves_2d[trim.curve_2d_index as usize];
                        if crv.degree() <= 1 && !crv.is_rational() {
                            for k in 0..crv.cv_count().saturating_sub(1) {
                                if let Some(p) = crv.get_cv(k) { loop_pts.push(p); }
                            }
                        } else {
                            let n = (crv.cv_count() * 4).max(16);
                            let (pts, _) = crv.divide_by_count(n, true);
                            for k in 0..pts.len().saturating_sub(1) { loop_pts.push(pts[k].clone()); }
                        }
                    }
                }
                if loop_pts.len() >= 3 {
                    let loop_crv = NurbsCurve::create(true, 1, &loop_pts);
                    if bloop.loop_type == BRepLoopType::Outer {
                        ts.m_outer_loop = Some(loop_crv);
                    } else {
                        ts.m_inner_loops.push(loop_crv);
                    }
                }
            }
            fmesh[fi] = ts.mesh();
        }

        // Apply reversed flag: flip BOTH winding and normals. The mesh shader derives
        // front/back from triangle winding (gl_FrontFacing) and flips the lighting normal
        // accordingly, so winding and the stored vertex normal must agree. Flipping only
        // the normal left reversed faces (e.g. the block-with-hole bottom cap and bore)
        // wound for the hidden side → they shaded/highlighted as back-faces (inverted).
        for fi in 0..nf {
            if self.m_faces[fi].reversed {
                fmesh[fi].flip();
                for (_, vd) in fmesh[fi].vertex.iter_mut() {
                    if let Some(n) = vd.normal() {
                        vd.set_normal(-n[0], -n[1], -n[2]);
                    }
                }
            }
        }

        fmesh
    }

    pub fn mesh(&self) -> Mesh {
        use crate::nurbssurface_trimmed::NurbsSurfaceTrimmed;
        let nf = self.m_faces.len();

        // Phase 1: Classify faces as direct (RemeshNurbsSurfaceGrid) or CDT
        let mut face_direct = vec![false; nf];
        for fi in 0..nf {
            let face = &self.m_faces[fi];
            if face.surface_index < 0 || face.surface_index as usize >= self.m_surfaces.len() { continue; }
            let srf = &self.m_surfaces[face.surface_index as usize];
            let mut has_inner = false;
            let mut all_linear = true;
            let mut outer_pts: Vec<Point> = Vec::new();
            for &li in &face.loop_indices {
                if li < 0 || li as usize >= self.m_loops.len() { continue; }
                let bloop = &self.m_loops[li as usize];
                if bloop.loop_type == BRepLoopType::Inner { has_inner = true; }
                for &ti in &bloop.trim_indices {
                    if ti < 0 || ti as usize >= self.m_trims.len() { continue; }
                    let trim = &self.m_trims[ti as usize];
                    if trim.curve_2d_index < 0 || trim.curve_2d_index as usize >= self.m_curves_2d.len() { continue; }
                    let crv = &self.m_curves_2d[trim.curve_2d_index as usize];
                    if crv.degree() > 1 || crv.is_rational() { all_linear = false; }
                    if bloop.loop_type == BRepLoopType::Outer && crv.degree() <= 1 && !crv.is_rational() {
                        for k in 0..crv.cv_count().saturating_sub(1) {
                            if let Some(p) = crv.get_cv(k) { outer_pts.push(p); }
                        }
                    }
                }
            }
            let mut direct = !has_inner && all_linear;
            if direct && !outer_pts.is_empty() {
                if let (Some((u0, u1)), Some((v0, v1))) = (srf.domain(0), srf.domain(1)) {
                    let tol = (u1 - u0).max(v1 - v0) * 0.01;
                    let mut bb_umin = f64::INFINITY; let mut bb_umax = f64::NEG_INFINITY;
                    let mut bb_vmin = f64::INFINITY; let mut bb_vmax = f64::NEG_INFINITY;
                    for p in &outer_pts {
                        if p[0] < bb_umin { bb_umin = p[0]; }
                        if p[0] > bb_umax { bb_umax = p[0]; }
                        if p[1] < bb_vmin { bb_vmin = p[1]; }
                        if p[1] > bb_vmax { bb_vmax = p[1]; }
                    }
                    if (bb_umin - u0).abs() > tol || (bb_umax - u1).abs() > tol ||
                       (bb_vmin - v0).abs() > tol || (bb_vmax - v1).abs() > tol {
                        direct = false;
                    }
                }
            }
            face_direct[fi] = direct;
        }

        // Phase 2: Mesh direct faces
        let mut fmesh: Vec<Mesh> = (0..nf).map(|_| Mesh::new()).collect();
        for fi in 0..nf {
            if !face_direct[fi] { continue; }
            let face = &self.m_faces[fi];
            let srf = &self.m_surfaces[face.surface_index as usize];
            fmesh[fi] = srf.mesh();
        }

        // Phase 3: Fan-tessellate CDT faces (outer loop boundary evaluated on surface)
        for fi in 0..nf {
            if face_direct[fi] { continue; }
            let face = &self.m_faces[fi];
            if face.surface_index < 0 || face.surface_index as usize >= self.m_surfaces.len() { continue; }
            let srf = &self.m_surfaces[face.surface_index as usize];
            let mut ts = NurbsSurfaceTrimmed::new();
            ts.m_surface = srf.clone();
            for &li in &face.loop_indices {
                if li < 0 || li as usize >= self.m_loops.len() { continue; }
                let bloop = &self.m_loops[li as usize];
                let mut loop_pts: Vec<Point> = Vec::new();
                for &ti in &bloop.trim_indices {
                    if ti < 0 || ti as usize >= self.m_trims.len() { continue; }
                    let trim = &self.m_trims[ti as usize];
                    if trim.trim_type == BRepTrimType::Singular { continue; }
                    if trim.curve_2d_index < 0 || trim.curve_2d_index as usize >= self.m_curves_2d.len() { continue; }
                    let crv = &self.m_curves_2d[trim.curve_2d_index as usize];
                    if crv.degree() <= 1 && !crv.is_rational() {
                        for k in 0..crv.cv_count().saturating_sub(1) {
                            if let Some(p) = crv.get_cv(k) { loop_pts.push(p); }
                        }
                    } else {
                        let n = (crv.cv_count() * 4).max(16);
                        let (pts, _) = crv.divide_by_count(n, false);
                        for k in 0..pts.len().saturating_sub(1) { loop_pts.push(pts[k].clone()); }
                    }
                }
                if loop_pts.len() >= 3 {
                    let loop_crv = NurbsCurve::create(true, 1, &loop_pts);
                    if bloop.loop_type == BRepLoopType::Outer {
                        ts.m_outer_loop = Some(loop_crv);
                    } else {
                        ts.m_inner_loops.push(loop_crv);
                    }
                }
            }
            fmesh[fi] = ts.mesh();
        }

        // Phase 4: Combine
        let mut all_polygons: Vec<Vec<Point>> = Vec::new();
        for fi in 0..nf {
            let face = &self.m_faces[fi];
            let fm = &fmesh[fi];
            if fm.face.is_empty() { continue; }
            // Reversed faces must have their triangle winding flipped so the facet
            // orientation matches the face's outward normal (from_polylines rebuilds
            // vertices from positions, so flipping per-vertex normals here has no effect).
            for (_fk, fverts) in &fm.face {
                let mut poly: Vec<Point> = fverts.iter()
                    .filter_map(|vi| fm.vertex.get(vi).map(|v| Point::new(v.x, v.y, v.z)))
                    .collect();
                if face.reversed { poly.reverse(); }
                all_polygons.push(poly);
            }
        }
        Mesh::from_polylines(all_polygons, None)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Evaluation
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn point_at(&self, face_idx: usize, u: f64, v: f64) -> Point {
        if face_idx >= self.m_faces.len() { return Point::new(0.0, 0.0, 0.0); }
        let si = self.m_faces[face_idx].surface_index;
        if si < 0 || si as usize >= self.m_surfaces.len() { return Point::new(0.0, 0.0, 0.0); }
        self.m_surfaces[si as usize].point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0))
    }

    pub fn normal_at(&self, face_idx: usize, u: f64, v: f64) -> Vector {
        if face_idx >= self.m_faces.len() { return Vector::new(0.0, 0.0, 0.0); }
        let si = self.m_faces[face_idx].surface_index;
        if si < 0 || si as usize >= self.m_surfaces.len() { return Vector::new(0.0, 0.0, 0.0); }
        let n = self.m_surfaces[si as usize].normal_at(u, v);
        if self.m_faces[face_idx].reversed { -n } else { n }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn transform(&mut self) {
        let xf = self.xform.clone();
        for srf in &mut self.m_surfaces {
            srf.transform(&xf);
        }
        for crv in &mut self.m_curves_3d {
            crv.transform(Some(&xf));
        }
        for pt in &mut self.m_vertices {
            let x = xf.m[0] * pt[0] + xf.m[4] * pt[1] + xf.m[8] * pt[2] + xf.m[12];
            let y = xf.m[1] * pt[0] + xf.m[5] * pt[1] + xf.m[9] * pt[2] + xf.m[13];
            let z = xf.m[2] * pt[0] + xf.m[6] * pt[1] + xf.m[10] * pt[2] + xf.m[14];
            *pt = Point::new(x, y, z);
        }
        self.xform = Xform::identity();
    }

    pub fn transformed(&self) -> Self {
        let mut b = self.clone();
        b.transform();
        b
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn file_json_dumps(&self) -> String {
        crate::file_encoders::sorted_json_string(self).unwrap_or_default()
    }

    pub fn file_json_loads(s: &str) -> Self {
        serde_json::from_str(s).unwrap_or_else(|_| Self::new())
    }

    pub fn file_json_dump(&self, filepath: &str) {
        let json = crate::file_encoders::sorted_json_string(self).unwrap_or_default();
        std::fs::write(filepath, json).expect("Failed to write JSON file");
    }

    pub fn file_json_load(filepath: &str) -> Self {
        let json = std::fs::read_to_string(filepath).expect("Failed to read JSON file");
        serde_json::from_str(&json).unwrap_or_else(|_| Self::new())
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        let curves_2d: Vec<crate::proto::NurbsCurve> = self.m_curves_2d.iter()
            .map(|c| crate::proto::NurbsCurve::decode(c.pb_dumps().as_slice()).unwrap())
            .collect();
        let curves_3d: Vec<crate::proto::NurbsCurve> = self.m_curves_3d.iter()
            .map(|c| crate::proto::NurbsCurve::decode(c.pb_dumps().as_slice()).unwrap())
            .collect();
        let surfaces: Vec<crate::proto::NurbsSurface> = self.m_surfaces.iter()
            .map(|s| crate::proto::NurbsSurface::decode(s.pb_dumps().as_slice()).unwrap())
            .collect();
        let vertices: Vec<crate::proto::Point> = self.m_vertices.iter()
            .map(|v| crate::proto::Point { guid: String::new(), name: String::new(), x: v[0] as f64, y: v[1] as f64, z: v[2] as f64, width: 0.0,
                pointcolor: None, xform: None })
            .collect();
        let topology_vertices: Vec<crate::proto::BRepVertex> = self.m_topology_vertices.iter()
            .map(|tv| crate::proto::BRepVertex { point_index: tv.point_index, edge_indices: tv.edge_indices.clone() })
            .collect();
        let topology_edges: Vec<crate::proto::BRepEdge> = self.m_topology_edges.iter()
            .map(|te| crate::proto::BRepEdge {
                curve_3d_index: te.curve_3d_index, start_vertex: te.start_vertex,
                end_vertex: te.end_vertex, trim_indices: te.trim_indices.clone(),
            })
            .collect();
        let trims: Vec<crate::proto::BRepTrim> = self.m_trims.iter()
            .map(|t| crate::proto::BRepTrim {
                curve_2d_index: t.curve_2d_index, edge_index: t.edge_index,
                loop_index: t.loop_index, reversed: t.reversed,
                r#type: t.trim_type as i32,
            })
            .collect();
        let loops: Vec<crate::proto::BRepLoop> = self.m_loops.iter()
            .map(|l| crate::proto::BRepLoop {
                trim_indices: l.trim_indices.clone(), face_index: l.face_index,
                r#type: l.loop_type as i32,
            })
            .collect();
        let faces: Vec<crate::proto::BRepFace> = self.m_faces.iter()
            .map(|f| crate::proto::BRepFace {
                surface_index: f.surface_index, loop_indices: f.loop_indices.clone(),
                reversed: f.reversed,
                facecolor: f.facecolor.as_ref().map(|c| crate::proto::Color {
                    guid: String::new(), name: String::new(),
                    r: c.r, g: c.g, b: c.b, a: c.a,
                }),
            })
            .collect();

        let proto = crate::proto::BRep {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            curves_2d,
            curves_3d,
            surfaces,
            vertices,
            topology_vertices,
            topology_edges,
            trims,
            loops,
            faces,
            width: self.width as f64,
            surfacecolor: Some(crate::proto::Color {
                guid: self.surfacecolor.guid().to_string(),
                name: self.surfacecolor.name.clone(),
                r: self.surfacecolor.r,
                g: self.surfacecolor.g,
                b: self.surfacecolor.b,
                a: self.surfacecolor.a,
            }),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid().to_string(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.iter().map(|&v| v as f64).collect(),
            }),
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::BRep::decode(data)?;
        let mut b = BRep::new();
        b.set_guid(proto.guid.clone());
        b.name = proto.name;
        b.width = proto.width as f64;

        for c in &proto.curves_2d {
            b.m_curves_2d.push(NurbsCurve::pb_loads(&c.encode_to_vec())?);
        }
        for c in &proto.curves_3d {
            b.m_curves_3d.push(NurbsCurve::pb_loads(&c.encode_to_vec())?);
        }
        for s in &proto.surfaces {
            b.m_surfaces.push(NurbsSurface::pb_loads(&s.encode_to_vec())?);
        }
        for v in &proto.vertices {
            b.m_vertices.push(Point::new(v.x as f64, v.y as f64, v.z as f64));
        }
        for tv in &proto.topology_vertices {
            b.m_topology_vertices.push(BRepVertex {
                point_index: tv.point_index,
                edge_indices: tv.edge_indices.clone(),
            });
        }
        for te in &proto.topology_edges {
            b.m_topology_edges.push(BRepEdge {
                curve_3d_index: te.curve_3d_index,
                start_vertex: te.start_vertex,
                end_vertex: te.end_vertex,
                trim_indices: te.trim_indices.clone(),
            });
        }
        for t in &proto.trims {
            b.m_trims.push(BRepTrim {
                curve_2d_index: t.curve_2d_index,
                edge_index: t.edge_index,
                loop_index: t.loop_index,
                reversed: t.reversed,
                trim_type: match t.r#type {
                    1 => BRepTrimType::Mated,
                    2 => BRepTrimType::Seam,
                    3 => BRepTrimType::Singular,
                    _ => BRepTrimType::Boundary,
                },
            });
        }
        for l in &proto.loops {
            b.m_loops.push(BRepLoop {
                trim_indices: l.trim_indices.clone(),
                face_index: l.face_index,
                loop_type: if l.r#type == 1 { BRepLoopType::Inner } else { BRepLoopType::Outer },
            });
        }
        for f in &proto.faces {
            b.m_faces.push(BRepFace {
                surface_index: f.surface_index,
                loop_indices: f.loop_indices.clone(),
                reversed: f.reversed,
                facecolor: f.facecolor.as_ref().map(|c| Color::new(c.r, c.g, c.b, c.a)),
            });
        }

        if let Some(color) = proto.surfacecolor {
            b.surfacecolor.set_guid(color.guid.clone());
            b.surfacecolor.name = color.name;
            b.surfacecolor.r = color.r;
            b.surfacecolor.g = color.g;
            b.surfacecolor.b = color.b;
            b.surfacecolor.a = color.a;
        }
        if let Some(xform) = proto.xform {
            b.xform.set_guid(xform.guid.clone());
            b.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 { b.xform.m[i] = *val as f64; }
            }
        }

        Ok(b)
    }

    pub fn pb_dump(&self, filepath: &str) {
        std::fs::write(filepath, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representation
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn str(&self) -> String {
        format!("BRep(name={}, faces={}, edges={}, vertices={})",
                self.name, self.face_count(), self.edge_count(), self.vertex_count())
    }

    pub fn repr(&self) -> String {
        format!("BRep(\n  name={},\n  faces={},\n  edges={},\n  vertices={},\n  solid={}\n)",
                self.name, self.face_count(), self.edge_count(), self.vertex_count(),
                if self.is_solid() { "true" } else { "false" })
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// Serde: custom Serialize for alphabetical JSON field order
///////////////////////////////////////////////////////////////////////////////////////////

impl Serialize for BRep {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where S: Serializer {
        let mut map = serializer.serialize_map(None)?;
        map.serialize_entry("curves_2d", &self.m_curves_2d)?;
        map.serialize_entry("curves_3d", &self.m_curves_3d)?;
        // faces
        let faces_json: Vec<BRepFaceJson> = self.m_faces.iter().map(|f| BRepFaceJson {
            facecolor: f.facecolor.clone(),
            loop_indices: &f.loop_indices, reversed: f.reversed, surface_index: f.surface_index,
        }).collect();
        map.serialize_entry("faces", &faces_json)?;
        map.serialize_entry("guid", &self.guid())?;
        // loops
        let loops_json: Vec<BRepLoopJson> = self.m_loops.iter().map(|l| BRepLoopJson {
            face_index: l.face_index, trim_indices: &l.trim_indices,
            loop_type: match l.loop_type { BRepLoopType::Inner => "inner", _ => "outer" },
        }).collect();
        map.serialize_entry("loops", &loops_json)?;
        map.serialize_entry("name", &self.name)?;
        map.serialize_entry("surfaces", &self.m_surfaces)?;
        map.serialize_entry("surfacecolor", &self.surfacecolor)?;
        // topology_edges
        let te_json: Vec<BRepEdgeJson> = self.m_topology_edges.iter().map(|e| BRepEdgeJson {
            curve_3d_index: e.curve_3d_index, end_vertex: e.end_vertex,
            start_vertex: e.start_vertex, trim_indices: &e.trim_indices,
        }).collect();
        map.serialize_entry("topology_edges", &te_json)?;
        // topology_vertices
        let tv_json: Vec<BRepVertexJson> = self.m_topology_vertices.iter().map(|v| BRepVertexJson {
            edge_indices: &v.edge_indices, point_index: v.point_index,
        }).collect();
        map.serialize_entry("topology_vertices", &tv_json)?;
        // trims
        let trims_json: Vec<BRepTrimJson> = self.m_trims.iter().map(|t| BRepTrimJson {
            curve_2d_index: t.curve_2d_index, edge_index: t.edge_index,
            loop_index: t.loop_index, reversed: t.reversed,
            trim_type: match t.trim_type {
                BRepTrimType::Mated => "mated", BRepTrimType::Seam => "seam",
                BRepTrimType::Singular => "singular", _ => "boundary",
            },
        }).collect();
        map.serialize_entry("trims", &trims_json)?;
        map.serialize_entry("type", "BRep")?;
        // vertices as [x, y, z] arrays
        let verts: Vec<[f64; 3]> = self.m_vertices.iter().map(|v| [v[0], v[1], v[2]]).collect();
        map.serialize_entry("vertices", &verts)?;
        map.serialize_entry("width", &self.width)?;
        map.serialize_entry("xform", &self.xform)?;
        map.end()
    }
}

// Helper structs for alphabetical field ordering in nested JSON objects
#[derive(Serialize)]
struct BRepFaceJson<'a> {
    #[serde(skip_serializing_if = "Option::is_none")]
    facecolor: Option<Color>,
    loop_indices: &'a Vec<i32>,
    reversed: bool,
    surface_index: i32,
}

#[derive(Serialize)]
struct BRepLoopJson<'a> {
    face_index: i32,
    trim_indices: &'a Vec<i32>,
    #[serde(rename = "type")]
    loop_type: &'a str,
}

#[derive(Serialize)]
struct BRepEdgeJson<'a> {
    curve_3d_index: i32,
    end_vertex: i32,
    start_vertex: i32,
    trim_indices: &'a Vec<i32>,
}

#[derive(Serialize)]
struct BRepVertexJson<'a> {
    edge_indices: &'a Vec<i32>,
    point_index: i32,
}

#[derive(Serialize)]
struct BRepTrimJson<'a> {
    curve_2d_index: i32,
    edge_index: i32,
    loop_index: i32,
    reversed: bool,
    #[serde(rename = "type")]
    trim_type: &'a str,
}

impl<'de> Deserialize<'de> for BRep {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where D: Deserializer<'de> {
        #[derive(Deserialize)]
        struct BRepData {
            #[serde(default)]
            guid: Option<String>,
            #[serde(default)]
            name: Option<String>,
            #[serde(default)]
            width: Option<f64>,
            #[serde(default)]
            surfacecolor: Option<Color>,
            #[serde(default)]
            xform: Option<Xform>,
            #[serde(default)]
            curves_2d: Option<Vec<NurbsCurve>>,
            #[serde(default)]
            curves_3d: Option<Vec<NurbsCurve>>,
            #[serde(default)]
            surfaces: Option<Vec<NurbsSurface>>,
            #[serde(default)]
            vertices: Option<Vec<[f64; 3]>>,
            #[serde(default)]
            topology_vertices: Option<Vec<TopologyVertexData>>,
            #[serde(default)]
            topology_edges: Option<Vec<TopologyEdgeData>>,
            #[serde(default)]
            trims: Option<Vec<TrimData>>,
            #[serde(default)]
            loops: Option<Vec<LoopData>>,
            #[serde(default)]
            faces: Option<Vec<FaceData>>,
        }

        #[derive(Deserialize)]
        struct TopologyVertexData {
            point_index: i32,
            edge_indices: Vec<i32>,
        }

        #[derive(Deserialize)]
        struct TopologyEdgeData {
            curve_3d_index: i32,
            start_vertex: i32,
            end_vertex: i32,
            trim_indices: Vec<i32>,
        }

        #[derive(Deserialize)]
        struct TrimData {
            curve_2d_index: i32,
            edge_index: i32,
            loop_index: i32,
            reversed: bool,
            #[serde(rename = "type")]
            trim_type: String,
        }

        #[derive(Deserialize)]
        struct LoopData {
            face_index: i32,
            trim_indices: Vec<i32>,
            #[serde(rename = "type")]
            loop_type: String,
        }

        #[derive(Deserialize)]
        struct FaceData {
            surface_index: i32,
            loop_indices: Vec<i32>,
            reversed: bool,
            #[serde(default)]
            facecolor: Option<Color>,
        }

        let data = BRepData::deserialize(deserializer)?;
        let mut b = BRep::new();
        if let Some(g) = data.guid { b.set_guid(g); }
        if let Some(n) = data.name { b.name = n; }
        if let Some(w) = data.width { b.width = w; }
        if let Some(c) = data.surfacecolor { b.surfacecolor = c; }
        if let Some(x) = data.xform { b.xform = x; }
        if let Some(c) = data.curves_2d { b.m_curves_2d = c; }
        if let Some(c) = data.curves_3d { b.m_curves_3d = c; }
        if let Some(s) = data.surfaces { b.m_surfaces = s; }
        if let Some(v) = data.vertices {
            b.m_vertices = v.iter().map(|a| Point::new(a[0], a[1], a[2])).collect();
        }
        if let Some(tv) = data.topology_vertices {
            b.m_topology_vertices = tv.into_iter().map(|t| BRepVertex {
                point_index: t.point_index, edge_indices: t.edge_indices,
            }).collect();
        }
        if let Some(te) = data.topology_edges {
            b.m_topology_edges = te.into_iter().map(|t| BRepEdge {
                curve_3d_index: t.curve_3d_index, start_vertex: t.start_vertex,
                end_vertex: t.end_vertex, trim_indices: t.trim_indices,
            }).collect();
        }
        if let Some(tr) = data.trims {
            b.m_trims = tr.into_iter().map(|t| BRepTrim {
                curve_2d_index: t.curve_2d_index, edge_index: t.edge_index,
                loop_index: t.loop_index, reversed: t.reversed,
                trim_type: match t.trim_type.as_str() {
                    "mated" => BRepTrimType::Mated, "seam" => BRepTrimType::Seam,
                    "singular" => BRepTrimType::Singular, _ => BRepTrimType::Boundary,
                },
            }).collect();
        }
        if let Some(l) = data.loops {
            b.m_loops = l.into_iter().map(|l| BRepLoop {
                trim_indices: l.trim_indices, face_index: l.face_index,
                loop_type: if l.loop_type == "inner" { BRepLoopType::Inner } else { BRepLoopType::Outer },
            }).collect();
        }
        if let Some(f) = data.faces {
            b.m_faces = f.into_iter().map(|f| BRepFace {
                surface_index: f.surface_index, loop_indices: f.loop_indices, reversed: f.reversed,
                facecolor: f.facecolor,
            }).collect();
        }
        Ok(b)
    }
}

impl std::fmt::Display for BRep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.str())
    }
}
