use crate::color::Color;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
use crate::vector::Vector;
use crate::xform::Xform;
use serde::ser::SerializeMap;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

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
            c.m_knot = vec![0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];
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
        let seam_crv = NurbsCurve::create(false, 1, &[p_south, p_north]);
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
            for i in 0..10 { hole_crv.set_knot(i, ckn[i]); }
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
                crv2d.m_knot = crv.m_knot.clone();
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
    // Meshing
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn mesh(&self) -> Mesh {
        use crate::trimmedsurface::TrimmedSurface;
        let mut all_polygons: Vec<Vec<Point>> = Vec::new();
        for face in &self.m_faces {
            if face.surface_index < 0 || face.surface_index as usize >= self.m_surfaces.len() {
                continue;
            }
            let srf = &self.m_surfaces[face.surface_index as usize];
            let mut ts = TrimmedSurface::new();
            ts.m_surface = srf.clone();
            for &li in &face.loop_indices {
                if li < 0 || (li as usize) >= self.m_loops.len() { continue; }
                let bloop = &self.m_loops[li as usize];
                let mut loop_pts = Vec::new();
                for &ti in &bloop.trim_indices {
                    if ti < 0 || (ti as usize) >= self.m_trims.len() { continue; }
                    let trim = &self.m_trims[ti as usize];
                    if trim.curve_2d_index < 0 || (trim.curve_2d_index as usize) >= self.m_curves_2d.len() { continue; }
                    let crv = &self.m_curves_2d[trim.curve_2d_index as usize];
                    let ndiv = if crv.degree() > 1 { (crv.cv_count() * 4).max(16) } else { (crv.cv_count().saturating_sub(1)).max(4) };
                    let (pts, _) = crv.divide_by_count(ndiv, true);
                    for k in 0..pts.len().saturating_sub(1) {
                        loop_pts.push(pts[k].clone());
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
            let mut face_mesh = ts.mesh();
            if face.reversed {
                for (_, vd) in face_mesh.vertex.iter_mut() {
                    let n = vd.normal();
                    if let Some(n) = n {
                        vd.set_normal(-n[0], -n[1], -n[2]);
                    }
                }
            }
            for (_fk, fverts) in &face_mesh.face {
                let poly: Vec<Point> = fverts.iter()
                    .filter_map(|vi| face_mesh.vertex.get(vi).map(|v| Point::new(v.x, v.y, v.z)))
                    .collect();
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
        self.m_surfaces[si as usize].normal_at(u, v)
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

    pub fn json_dumps(&self) -> String {
        crate::encoders::sorted_json_string(self).unwrap_or_default()
    }

    pub fn json_loads(s: &str) -> Self {
        serde_json::from_str(s).unwrap_or_else(|_| Self::new())
    }

    pub fn json_dump(&self, filepath: &str) {
        let json = crate::encoders::sorted_json_string(self).unwrap_or_default();
        std::fs::write(filepath, json).expect("Failed to write JSON file");
    }

    pub fn json_load(filepath: &str) -> Self {
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
            .map(|v| crate::proto::Point { guid: String::new(), name: String::new(), x: v[0], y: v[1], z: v[2], width: 0.0,
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
                    r: c.r as i32, g: c.g as i32, b: c.b as i32, a: c.a as i32,
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
            width: self.width,
            surfacecolor: Some(crate::proto::Color {
                guid: self.surfacecolor.guid().to_string(),
                name: self.surfacecolor.name.clone(),
                r: self.surfacecolor.r as i32,
                g: self.surfacecolor.g as i32,
                b: self.surfacecolor.b as i32,
                a: self.surfacecolor.a as i32,
            }),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid().to_string(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
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
        b.width = proto.width;

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
            b.m_vertices.push(Point::new(v.x, v.y, v.z));
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
                facecolor: f.facecolor.as_ref().map(|c| Color::new(c.r as u8, c.g as u8, c.b as u8, c.a as u8)),
            });
        }

        if let Some(color) = proto.surfacecolor {
            b.surfacecolor.set_guid(color.guid.clone());
            b.surfacecolor.name = color.name;
            b.surfacecolor.r = color.r as u8;
            b.surfacecolor.g = color.g as u8;
            b.surfacecolor.b = color.b as u8;
            b.surfacecolor.a = color.a as u8;
        }
        if let Some(xform) = proto.xform {
            b.xform.set_guid(xform.guid.clone());
            b.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 { b.xform.m[i] = *val; }
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
