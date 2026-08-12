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

    /// Clear the guid so a FRESH one mints lazily on next read — the duplicate/copy enabler.
    pub fn refresh_guid(&mut self) {
        self.guid = std::sync::OnceLock::new();
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

    pub fn create_cone(radius: f64, height: f64) -> Self {
        // Side face = cone_surface (u in [0,4] = circle, v in [0,1] = base->apex; v=1 is a SINGULAR
        // apex pole, like a sphere pole) + one planar base cap. Mirrors create_cylinder's base+seam but
        // with a singular apex instead of a top cap.
        use crate::primitives::Primitives;
        let mut brep = BRep::new();
        brep.name = "cone".to_string();
        let body = Primitives::cone_surface(0.0, 0.0, 0.0, radius, height);
        let dom_u = body.domain(0).unwrap();
        let dom_v = body.domain(1).unwrap();
        let p_base = body.point_at(dom_u.0, dom_v.0).unwrap();   // on base circle at u=0
        let p_apex = Point::new(0.0, 0.0, height);
        let vi_base = brep.add_vertex(&p_base) as i32;
        let vi_apex = brep.add_vertex(&p_apex) as i32;
        brep.m_topology_vertices.push(BRepVertex { point_index: vi_base, edge_indices: Vec::new() });
        brep.m_topology_vertices.push(BRepVertex { point_index: vi_apex, edge_indices: Vec::new() });
        let circle_base = Primitives::circle(0.0, 0.0, 0.0, radius);
        let seam_line = NurbsCurve::create(false, 1, &[p_base, p_apex]);
        let ci_base = brep.add_curve_3d(&circle_base) as i32;
        let ci_seam = brep.add_curve_3d(&seam_line) as i32;
        let ei_base = brep.add_edge(ci_base, 0, 0) as i32;
        let ei_seam = brep.add_edge(ci_seam, 0, 1) as i32;
        let si_body = brep.add_surface(&body) as i32;
        let mut cap_base = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
        cap_base.set_cv(0, 0, &Point::new(-radius, -radius, 0.0));
        cap_base.set_cv(1, 0, &Point::new(radius, -radius, 0.0));
        cap_base.set_cv(0, 1, &Point::new(-radius, radius, 0.0));
        cap_base.set_cv(1, 1, &Point::new(radius, radius, 0.0));
        let si_base = brep.add_surface(&cap_base) as i32;
        let fi_body = brep.add_face(si_body, false) as i32;
        let li_body = brep.add_loop(fi_body, BRepLoopType::Outer) as i32;
        let c2d_base = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.0, dom_v.0, 0.0), Point::new(dom_u.1, dom_v.0, 0.0)]);
        let ci = brep.add_curve_2d(&c2d_base) as i32;
        brep.add_trim(ci, ei_base, li_body, false, BRepTrimType::Mated);
        let c2d_sr = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.1, dom_v.0, 0.0), Point::new(dom_u.1, dom_v.1, 0.0)]);
        let ci = brep.add_curve_2d(&c2d_sr) as i32;
        brep.add_trim(ci, ei_seam, li_body, false, BRepTrimType::Seam);
        let c2d_apex = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.1, dom_v.1, 0.0), Point::new(dom_u.0, dom_v.1, 0.0)]);
        let ci = brep.add_curve_2d(&c2d_apex) as i32;
        brep.add_trim(ci, -1, li_body, false, BRepTrimType::Singular);
        let c2d_sl = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.0, dom_v.1, 0.0), Point::new(dom_u.0, dom_v.0, 0.0)]);
        let ci = brep.add_curve_2d(&c2d_sl) as i32;
        brep.add_trim(ci, ei_seam, li_body, true, BRepTrimType::Seam);
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
        let fi_base = brep.add_face(si_base, true) as i32;
        let li_base = brep.add_loop(fi_base, BRepLoopType::Outer) as i32;
        let ci = brep.add_curve_2d(&make_cap_circle()) as i32;
        brep.add_trim(ci, ei_base, li_base, true, BRepTrimType::Mated);
        for ei in 0..brep.m_topology_edges.len() {
            let sv = brep.m_topology_edges[ei].start_vertex as usize;
            let ev = brep.m_topology_edges[ei].end_vertex as usize;
            brep.m_topology_vertices[sv].edge_indices.push(ei as i32);
            brep.m_topology_vertices[ev].edge_indices.push(ei as i32);
        }
        brep
    }

    pub fn create_torus(major_radius: f64, minor_radius: f64) -> Self {
        // Torus: a single closed face, periodic in BOTH u (major circle) and v (minor circle). No caps,
        // no poles -- two seams: the u-seam (minor circle at u=0) and the v-seam (outer major circle at
        // v=0), meeting at one corner vertex. The loop is the UV rectangle [0,4]x[0,4] = 4 Seam trims.
        use crate::primitives::Primitives;
        let mut brep = BRep::new();
        brep.name = "torus".to_string();
        let body = Primitives::torus_surface(0.0, 0.0, 0.0, major_radius, minor_radius);
        let dom_u = body.domain(0).unwrap();
        let dom_v = body.domain(1).unwrap();
        let p_corner = body.point_at(dom_u.0, dom_v.0).unwrap();   // (major+minor, 0, 0)
        let vi = brep.add_vertex(&p_corner) as i32;
        brep.m_topology_vertices.push(BRepVertex { point_index: vi, edge_indices: Vec::new() });
        // u-seam: v -> point_at(u0, v), the minor circle at u=0. v-seam: u -> point_at(u, v0), outer
        // major circle at v=0. Sample each as a closed polyline (the exact rational pcurve is a circle).
        let iso_curve = |along_v: bool| -> NurbsCurve {
            let n = 64;
            let mut pts: Vec<Point> = Vec::new();
            for i in 0..=n {
                let t = i as f64 / n as f64;
                let p = if along_v {
                    body.point_at(dom_u.0, dom_v.0 + (dom_v.1 - dom_v.0) * t)
                } else {
                    body.point_at(dom_u.0 + (dom_u.1 - dom_u.0) * t, dom_v.0)
                }.unwrap_or(Point::new(0.0, 0.0, 0.0));
                pts.push(p);
            }
            NurbsCurve::create(false, 3, &pts)
        };
        let c_useam = iso_curve(true);    // minor circle (varies v)
        let c_vseam = iso_curve(false);   // major circle (varies u)
        let ci_useam = brep.add_curve_3d(&c_useam) as i32;
        let ci_vseam = brep.add_curve_3d(&c_vseam) as i32;
        let ei_useam = brep.add_edge(ci_useam, 0, 0) as i32;
        let ei_vseam = brep.add_edge(ci_vseam, 0, 0) as i32;
        let si_body = brep.add_surface(&body) as i32;
        let fi_body = brep.add_face(si_body, false) as i32;
        let li_body = brep.add_loop(fi_body, BRepLoopType::Outer) as i32;
        // Loop around the UV rectangle: bottom(v-seam), right(u-seam), top(v-seam), left(u-seam).
        let c2d_bottom = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.0, dom_v.0, 0.0), Point::new(dom_u.1, dom_v.0, 0.0)]);
        let ci = brep.add_curve_2d(&c2d_bottom) as i32;
        brep.add_trim(ci, ei_vseam, li_body, false, BRepTrimType::Seam);
        let c2d_right = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.1, dom_v.0, 0.0), Point::new(dom_u.1, dom_v.1, 0.0)]);
        let ci = brep.add_curve_2d(&c2d_right) as i32;
        brep.add_trim(ci, ei_useam, li_body, false, BRepTrimType::Seam);
        let c2d_top = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.1, dom_v.1, 0.0), Point::new(dom_u.0, dom_v.1, 0.0)]);
        let ci = brep.add_curve_2d(&c2d_top) as i32;
        brep.add_trim(ci, ei_vseam, li_body, true, BRepTrimType::Seam);
        let c2d_left = NurbsCurve::create(false, 1, &[
            Point::new(dom_u.0, dom_v.1, 0.0), Point::new(dom_u.0, dom_v.0, 0.0)]);
        let ci = brep.add_curve_2d(&c2d_left) as i32;
        brep.add_trim(ci, ei_useam, li_body, true, BRepTrimType::Seam);
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
        let diag = self.brep_bbox_diag();
        let deg_tol = (diag * 1e-7).max(1e-12);
        for e in &self.m_topology_edges {
            if e.trim_indices.len() == 2 { continue; }
            // A DEGENERATE edge (3D curve collapsed to a point, e.g. a sphere/cone pole) is
            // watertight by construction and OCCT excludes such degenerate edges from manifold
            // checks. Skip them; only genuine (non-zero-length) edges must be 2-trim.
            let ci = e.curve_3d_index;
            if ci >= 0 && (ci as usize) < self.m_curves_3d.len() {
                let c = &self.m_curves_3d[ci as usize];
                let (d0, d1) = c.domain();
                let p0 = c.point_at(d0);
                let mut ext = 0.0_f64;
                for k in 1..=4 {
                    let pk = c.point_at(d0 + (d1 - d0) * k as f64 / 4.0);
                    let d = ((pk[0]-p0[0]).powi(2) + (pk[1]-p0[1]).powi(2) + (pk[2]-p0[2]).powi(2)).sqrt();
                    if d > ext { ext = d; }
                }
                if ext < deg_tol { continue; }
            }
            return false;
        }
        true
    }

    /// Diagonal length of the BRep's vertex bounding box (>=1).
    pub fn brep_bbox_diag(&self) -> f64 {
        let mut mn = [1e300_f64; 3];
        let mut mx = [-1e300_f64; 3];
        for p in &self.m_vertices {
            for d in 0..3 {
                if p[d] < mn[d] { mn[d] = p[d]; }
                if p[d] > mx[d] { mx[d] = p[d]; }
            }
        }
        let d = ((mx[0]-mn[0]).powi(2) + (mx[1]-mn[1]).powi(2) + (mx[2]-mn[2]).powi(2)).sqrt();
        if d > 0.0 { d } else { 1.0 }
    }

    /// Volume via the divergence theorem: V = (1/3) sum_faces flux_outward. The per-face
    /// OUTWARD sign is determined GEOMETRICALLY (step off the face along its natural normal and
    /// test inside/outside), so it is independent of stored orientation flags. Planar faces use
    /// the exact chained boundary integral; curved faces use trim-masked composite Gauss of
    /// S.(Su x Sv). Matches OCCT BRepGProp to machine precision.
    pub fn volume(&self) -> f64 {
        use crate::polyline::Polyline;
        use crate::remesh_cdt::RemeshCDT;
        const GN: [f64; 5] = [-0.9061798459386640, -0.5384693101056831, 0.0,
                              0.5384693101056831, 0.9061798459386640];
        const GW: [f64; 5] = [0.2369268850561891, 0.4786286704993665, 0.5688888888888889,
                              0.4786286704993665, 0.2369268850561891];
        let cross = |a: &Vector, b: &Vector| Vector::new(
            a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]);

        let is_planar = |s: &NurbsSurface| -> bool {
            let (u0,u1) = s.domain(0).unwrap_or((0.0,1.0));
            let (v0,v1) = s.domain(1).unwrap_or((0.0,1.0));
            let n0 = s.normal_at(u0+(u1-u0)*0.5, v0+(v1-v0)*0.5);
            let uu = [0.25, 0.5, 0.75]; let vv = [0.3, 0.6, 0.8];
            for i in 0..3 {
                let n = s.normal_at(u0+(u1-u0)*uu[i], v0+(v1-v0)*vv[i]);
                if cross(&n0, &n).magnitude() > 1e-7 { return false; }
            }
            true
        };

        // 1/2 closed integral C x C', chaining the loop's pcurves head-to-tail by matching UV
        // endpoints (orientation-independent of stored reversed flags).
        let loop_vector_area = |srf: &NurbsSurface, bloop: &BRepLoop| -> Vector {
            let mut segs: Vec<(NurbsCurve, f64, f64, Point, Point)> = Vec::new();
            for &ti in &bloop.trim_indices {
                if ti < 0 || ti as usize >= self.m_trims.len() { continue; }
                let c2 = self.m_trims[ti as usize].curve_2d_index;
                if c2 < 0 || c2 as usize >= self.m_curves_2d.len() { continue; }
                let pc = self.m_curves_2d[c2 as usize].clone();
                let (d0, d1) = pc.domain();
                let ps = pc.point_at(d0);
                let pe = pc.point_at(d1);
                segs.push((pc, d0, d1, ps, pe));
            }
            if segs.is_empty() { return Vector::new(0.0,0.0,0.0); }
            let d2 = |a: &Point, b: &Point| (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]);
            let mut order: Vec<(usize, bool)> = vec![(0, true)];
            let mut used = vec![false; segs.len()]; used[0] = true;
            let mut tail = segs[0].4.clone();
            for _ in 1..segs.len() {
                let mut best: i32 = -1; let mut fwd = true; let mut bd = 1e300;
                for j in 0..segs.len() {
                    if used[j] { continue; }
                    let ds = d2(&segs[j].3, &tail); let de = d2(&segs[j].4, &tail);
                    if ds < bd { bd = ds; best = j as i32; fwd = true; }
                    if de < bd { bd = de; best = j as i32; fwd = false; }
                }
                if best < 0 { break; }
                let b = best as usize;
                used[b] = true; order.push((b, fwd));
                tail = if fwd { segs[b].4.clone() } else { segs[b].3.clone() };
            }
            let mut ax = 0.0; let mut ay = 0.0; let mut az = 0.0;
            let ns = 24;
            for (idx, fwd) in &order {
                let (pc, t0, t1, _ps, _pe) = &segs[*idx];
                for s in 0..ns {
                    let a = t0 + (t1-t0)*s as f64/ns as f64;
                    let b = t0 + (t1-t0)*(s+1) as f64/ns as f64;
                    let mid = 0.5*(a+b); let half = 0.5*(b-a);
                    for g in 0..5 {
                        let t = mid + half*GN[g];
                        let pe = pc.evaluate(t, 1);
                        if pe.len() < 2 { continue; }
                        let uv = pe[0].clone();
                        let mut duv = pe[1].clone();
                        if !*fwd { duv = Vector::new(-duv[0], -duv[1], -duv[2]); }
                        let se = srf.evaluate(uv[0], uv[1], 1);
                        if se.len() < 3 { continue; }
                        let (s_, sv, su) = (se[0].clone(), se[1].clone(), se[2].clone());
                        let cp = Vector::new(su[0]*duv[0]+sv[0]*duv[1], su[1]*duv[0]+sv[1]*duv[1],
                                             su[2]*duv[0]+sv[2]*duv[1]);
                        let cr = cross(&s_, &cp);
                        let w = GW[g]*half;
                        ax += w*cr[0]; ay += w*cr[1]; az += w*cr[2];
                    }
                }
            }
            Vector::new(0.5*ax, 0.5*ay, 0.5*az)
        };

        let face_interior = |face: &BRepFace, srf: &NurbsSurface| -> (Point, Vector) {
            let mut outers: Vec<Polyline> = Vec::new();
            let mut inners: Vec<Polyline> = Vec::new();
            for &li in &face.loop_indices {
                if li < 0 || li as usize >= self.m_loops.len() { continue; }
                let mut pts: Vec<Point> = Vec::new();
                for &ti in &self.m_loops[li as usize].trim_indices {
                    if ti < 0 || ti as usize >= self.m_trims.len() { continue; }
                    let c2 = self.m_trims[ti as usize].curve_2d_index;
                    if c2 < 0 || c2 as usize >= self.m_curves_2d.len() { continue; }
                    let pc = &self.m_curves_2d[c2 as usize];
                    let (d0, d1) = pc.domain();
                    let n = (pc.cv_count()*3).max(6);
                    for i in 0..n {
                        let uv = pc.point_at(d0 + (d1-d0)*i as f64/n as f64);
                        pts.push(Point::new(uv[0], uv[1], 0.0));
                    }
                }
                if pts.len() < 3 { continue; }
                if self.m_loops[li as usize].loop_type == BRepLoopType::Outer {
                    outers.push(Polyline::new(pts));
                } else {
                    inners.push(Polyline::new(pts));
                }
            }
            let (u0,u1) = srf.domain(0).unwrap_or((0.0,1.0));
            let (v0,v1) = srf.domain(1).unwrap_or((0.0,1.0));
            let mut cu = 0.5*(u0+u1); let mut cv = 0.5*(v0+v1);
            // Point-in-polygon (ray cast) over a Polyline's points.
            let pip = |uu: f64, vv: f64, poly: &Polyline| -> bool {
                let pp = poly.get_points();
                let mut inside = false; let mut j = pp.len() - 1;
                for i in 0..pp.len() {
                    let mut denom = pp[j][1] - pp[i][1]; if denom == 0.0 { denom = 1e-300; }
                    if ((pp[i][1] > vv) != (pp[j][1] > vv))
                        && (uu < (pp[j][0]-pp[i][0])*(vv-pp[i][1])/denom + pp[i][0]) {
                        inside = !inside;
                    }
                    j = i;
                }
                inside
            };
            let in_material = |uu: f64, vv: f64| -> bool {
                if !outers.is_empty() {
                    let mut any = false;
                    for op in &outers { if pip(uu,vv,op) { any = true; break; } }
                    if !any { return false; }
                }
                for ip in &inners { if pip(uu,vv,ip) { return false; } }
                true
            };
            if !outers.is_empty() {
                let mut allp: Vec<Polyline> = vec![outers[0].clone()];
                for ip in &inners { allp.push(ip.clone()); }
                let tris = RemeshCDT::triangulate(&allp);
                let mut flat: Vec<Point> = Vec::new();
                for pl in &allp { for p in pl.get_points() { flat.push(p); } }
                // Largest-area triangle whose centroid is on the face MATERIAL (inside outer,
                // outside every hole). The first triangle / domain centre can land in a hole
                // (e.g. an annulus' centre) -> wrong outward-sign probe -> wrong flux.
                let mut best_a = -1.0_f64;
                for t in &tris {
                    if t.0 >= flat.len() || t.1 >= flat.len() || t.2 >= flat.len() { continue; }
                    let (a,b,c) = (&flat[t.0], &flat[t.1], &flat[t.2]);
                    let tcu = (a[0]+b[0]+c[0])/3.0; let tcv = (a[1]+b[1]+c[1])/3.0;
                    let ar = ((b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1])).abs();
                    if ar > best_a && in_material(tcu, tcv) { best_a = ar; cu = tcu; cv = tcv; }
                }
                if best_a < 0.0 {
                    let mut found = false;
                    for iu in 1..12 {
                        if found { break; }
                        for iv in 1..12 {
                            let su = u0 + (u1-u0)*iu as f64/12.0;
                            let sv = v0 + (v1-v0)*iv as f64/12.0;
                            if in_material(su, sv) { cu = su; cv = sv; found = true; break; }
                        }
                    }
                }
            }
            let p = srf.point_at(cu, cv).unwrap_or(Point::new(0.0,0.0,0.0));
            let nrm = srf.normal_at(cu, cv);
            (p, nrm)
        };

        let bmesh = self.mesh();
        let diag = self.brep_bbox_diag();
        let eps = diag * 1e-3;

        let mut total = 0.0;
        for face in &self.m_faces {
            let si = face.surface_index;
            if si < 0 || si as usize >= self.m_surfaces.len() { continue; }
            let srf = &self.m_surfaces[si as usize];
            let (p3, nnat) = face_interior(face, srf);
            if nnat.magnitude() < 1e-12 { continue; }
            let probe = Point::new(p3[0]+eps*nnat[0], p3[1]+eps*nnat[1], p3[2]+eps*nnat[2]);
            let sign = if self.contains_point_with(&bmesh, &probe) { -1.0 } else { 1.0 };

            let mut flux_nat;
            let mut curved_rect = false;
            if is_planar(srf) {
                let mut area = 0.0;
                for &li in &face.loop_indices {
                    if li < 0 || li as usize >= self.m_loops.len() { continue; }
                    let la = loop_vector_area(srf, &self.m_loops[li as usize]).magnitude();
                    if self.m_loops[li as usize].loop_type == BRepLoopType::Outer { area += la; }
                    else { area -= la; }
                }
                let qn = p3[0]*nnat[0] + p3[1]*nnat[1] + p3[2]*nnat[2];
                flux_nat = qn * area;
            } else {
                let mut umin: f64 = 1e300; let mut umax: f64 = -1e300;
                let mut vmin: f64 = 1e300; let mut vmax: f64 = -1e300;
                let mut outer_polys: Vec<Vec<(f64,f64)>> = Vec::new();
                let mut inner_polys: Vec<Vec<(f64,f64)>> = Vec::new();
                for &li in &face.loop_indices {
                    if li < 0 || li as usize >= self.m_loops.len() { continue; }
                    let mut poly: Vec<(f64,f64)> = Vec::new();
                    for &ti in &self.m_loops[li as usize].trim_indices {
                        if ti < 0 || ti as usize >= self.m_trims.len() { continue; }
                        let c2 = self.m_trims[ti as usize].curve_2d_index;
                        if c2 < 0 || c2 as usize >= self.m_curves_2d.len() { continue; }
                        let pc = &self.m_curves_2d[c2 as usize];
                        let (d0, d1) = pc.domain();
                        let n = (pc.cv_count()*4).max(12);
                        for i in 0..n {
                            let uv = pc.point_at(d0 + (d1-d0)*i as f64/n as f64);
                            umin = umin.min(uv[0]); umax = umax.max(uv[0]);
                            vmin = vmin.min(uv[1]); vmax = vmax.max(uv[1]);
                            poly.push((uv[0], uv[1]));
                        }
                    }
                    if poly.len() >= 3 {
                        if self.m_loops[li as usize].loop_type == BRepLoopType::Outer { outer_polys.push(poly); }
                        else { inner_polys.push(poly); }
                    }
                }
                let (u0,u1) = srf.domain(0).unwrap_or((0.0,1.0));
                let (v0,v1) = srf.domain(1).unwrap_or((0.0,1.0));
                if umax <= umin || vmax <= vmin { umin = u0; umax = u1; vmin = v0; vmax = v1; }

                let in_poly = |u: f64, v: f64, p: &Vec<(f64,f64)>| -> bool {
                    let mut inside = false;
                    let mut j = p.len() - 1;
                    for i in 0..p.len() {
                        if (p[i].1 > v) != (p[j].1 > v)
                            && u < (p[j].0-p[i].0)*(v-p[i].1)/(p[j].1-p[i].1) + p[i].0 {
                            inside = !inside;
                        }
                        j = i;
                    }
                    inside
                };
                let in_trim = |u: f64, v: f64| -> bool {
                    let mut ok = outer_polys.is_empty();
                    for op in &outer_polys { if in_poly(u, v, op) { ok = true; break; } }
                    if !ok { return false; }
                    for ip in &inner_polys { if in_poly(u, v, ip) { return false; } }
                    true
                };

                // A RECTANGULAR trim (cylinder band, full sphere) needs only NU=24 -- every Gauss
                // point is inside, so the quadrature is exact. A NON-rectangular trim (sphere caps,
                // a band with circular holes) has a curved mask boundary whose staircase error
                // scales ~1/NU, so use a finer grid there. (Sphere cap-cut faces are handled
                // exactly below by the analytic boundary-integral flux; this Gauss is the fallback
                // for other curved faces.)
                let rect_trim = inner_polys.is_empty()
                    && (umin - u0).abs() < (u1 - u0) * 1e-3 && (umax - u1).abs() < (u1 - u0) * 1e-3
                    && (vmin - v0).abs() < (v1 - v0) * 1e-3 && (vmax - v1).abs() < (v1 - v0) * 1e-3;
                curved_rect = rect_trim;
                let nu = if rect_trim { 24 } else { 96 };
                let nv = nu;
                let mut f = 0.0;
                for iu in 0..nu {
                    let ua = umin+(umax-umin)*iu as f64/nu as f64;
                    let ub = umin+(umax-umin)*(iu+1) as f64/nu as f64;
                    let um = 0.5*(ua+ub); let uh = 0.5*(ub-ua);
                    for iv in 0..nv {
                        let va_ = vmin+(vmax-vmin)*iv as f64/nv as f64;
                        let vb = vmin+(vmax-vmin)*(iv+1) as f64/nv as f64;
                        let vm = 0.5*(va_+vb); let vh = 0.5*(vb-va_);
                        for gu in 0..5 {
                            let u = um + uh*GN[gu];
                            for gv in 0..5 {
                                let v = vm + vh*GN[gv];
                                if !in_trim(u, v) { continue; }
                                let d = srf.evaluate(u, v, 1);
                                if d.len() < 3 { continue; }
                                let nrm = cross(&d[2], &d[1]);  // Su x Sv (evaluate = [S, Sv, Su])
                                let integ = d[0][0]*nrm[0] + d[0][1]*nrm[1] + d[0][2]*nrm[2];
                                f += GW[gu]*GW[gv]*uh*vh*integ;
                            }
                        }
                    }
                }
                flux_nat = f;
            }
            // Analytic sphere boundary-integral flux (exact, ~300x fewer surface evals than the
            // masked Gauss above; used for sphere cap-cut faces -- a sphere minus circular caps).
            // flux_nat = integral P.(Su x Sv) du dv. With P = C + R*er (er radial) and Su x Sv || er,
            //   P.(Su x Sv) = C.(Su x Sv) + R^3 cos(phi) theta' phi'.
            // The first term integrates to C . (vector area) = C . (1/2) closed-int P x dP (boundary,
            // exact). The second, via Green's: -R^2 closed-int h dtheta, with h=(P-C).Zs and
            // theta = GEOMETRIC longitude atan2((P-C).Ys,(P-C).Xs) (seam-independent).
            // So flux_nat = sum_loops [ C.A_loop - R^2 * H_loop ] -- depends ONLY on the boundary
            // curve's 3D geometry, sidestepping the masked-Gauss region/staircase error.
            let mut flux_analytic = 0.0;
            let mut have_analytic = false;
            let mut pole_face = false;
            if !is_planar(srf) {
                let (su0, su1) = srf.domain(0).unwrap_or((0.0, 1.0));
                let (sv0, sv1) = srf.domain(1).unwrap_or((0.0, 1.0));
                let um = 0.5 * (su0 + su1);
                let ps = srf.point_at(um, sv0).unwrap_or(Point::new(0.0, 0.0, 0.0));
                let pn = srf.point_at(um, sv1).unwrap_or(Point::new(0.0, 0.0, 0.0));
                let pm = srf.point_at(um, 0.5 * (sv0 + sv1)).unwrap_or(Point::new(0.0, 0.0, 0.0));
                let axis = Vector::new(pn[0] - ps[0], pn[1] - ps[1], pn[2] - ps[2]);
                let rr = 0.5 * axis.magnitude();
                if rr > 1e-9 {
                    let cpt = Point::new(0.5 * (ps[0] + pn[0]), 0.5 * (ps[1] + pn[1]), 0.5 * (ps[2] + pn[2]));
                    let zs = Vector::new(axis[0] / (2.0 * rr), axis[1] / (2.0 * rr), axis[2] / (2.0 * rr));
                    // verify sphere: sample grid, |P-C| ~ R
                    let mut is_sphere = true;
                    'grid: for i in 0..=3 {
                        for j in 0..=3 {
                            let p = srf.point_at(su0 + (su1 - su0) * i as f64 / 3.0, sv0 + (sv1 - sv0) * j as f64 / 3.0)
                                .unwrap_or(Point::new(0.0, 0.0, 0.0));
                            let dd = ((p[0] - cpt[0]).powi(2) + (p[1] - cpt[1]).powi(2) + (p[2] - cpt[2]).powi(2)).sqrt();
                            if (dd - rr).abs() > rr * 1e-4 + 1e-6 { is_sphere = false; break 'grid; }
                        }
                    }
                    if is_sphere {
                        let dm = Vector::new(pm[0] - cpt[0], pm[1] - cpt[1], pm[2] - cpt[2]);
                        let dz = dm[0] * zs[0] + dm[1] * zs[1] + dm[2] * zs[2];
                        let mut xs = Vector::new(dm[0] - dz * zs[0], dm[1] - dz * zs[1], dm[2] - dz * zs[2]);
                        let xn = xs.magnitude();
                        xs = Vector::new(xs[0] / xn, xs[1] / xn, xs[2] / xn);
                        let ys = cross(&zs, &xs);
                        let dot3 = |a: &Vector, b: &Vector| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
                        have_analytic = true;
                        for &li in &face.loop_indices {
                            if li < 0 || li as usize >= self.m_loops.len() { continue; }
                            // chained, ordered 3D polyline of the loop (greedy walk like loop_vector_area)
                            let mut segs: Vec<(&NurbsCurve, f64, f64, Point, Point)> = Vec::new();
                            for &ti in &self.m_loops[li as usize].trim_indices {
                                if ti < 0 || ti as usize >= self.m_trims.len() { continue; }
                                let c2 = self.m_trims[ti as usize].curve_2d_index;
                                if c2 < 0 || c2 as usize >= self.m_curves_2d.len() { continue; }
                                let pc = &self.m_curves_2d[c2 as usize];
                                let (d0, d1) = pc.domain();
                                let ps_ = pc.point_at(d0);
                                let pe_ = pc.point_at(d1);
                                segs.push((pc, d0, d1, ps_, pe_));
                            }
                            if segs.is_empty() { continue; }
                            let d2 = |a: &Point, b: &Point| (a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]);
                            let mut order: Vec<(usize, bool)> = vec![(0, true)];
                            let mut used = vec![false; segs.len()]; used[0] = true;
                            let mut tail = segs[0].4.clone();
                            for _ in 1..segs.len() {
                                let mut best: i32 = -1; let mut fwd = true; let mut bd = 1e300;
                                for j in 0..segs.len() {
                                    if used[j] { continue; }
                                    let ds = d2(&segs[j].3, &tail); let de = d2(&segs[j].4, &tail);
                                    if ds < bd { bd = ds; best = j as i32; fwd = true; }
                                    if de < bd { bd = de; best = j as i32; fwd = false; }
                                }
                                if best < 0 { break; }
                                let b = best as usize;
                                used[b] = true; order.push((b, fwd));
                                tail = if fwd { segs[b].4.clone() } else { segs[b].3.clone() };
                            }
                            // sample ordered UV polyline (dense -> boundary integral converges to the
                            // pcurve's enclosed flux; residual is the pullback pcurve's own CV density).
                            let mut uvpts: Vec<(f64, f64)> = Vec::new();
                            for (idx, fwd) in &order {
                                let (pc, t0, t1, _ps, _pe) = &segs[*idx];
                                let n = 200;
                                for s in 0..n {
                                    let tt = if *fwd { t0 + (t1 - t0) * s as f64 / n as f64 }
                                             else { t1 - (t1 - t0) * s as f64 / n as f64 };
                                    let uv = pc.point_at(tt);
                                    uvpts.push((uv[0], uv[1]));
                                }
                            }
                            if uvpts.len() < 3 { continue; }
                            // close it
                            uvpts.push(uvpts[0]);
                            let mut uv_area = 0.0;
                            for i in 0..uvpts.len() - 1 {
                                uv_area += uvpts[i].0 * uvpts[i + 1].1 - uvpts[i + 1].0 * uvpts[i].1;
                            }
                            uv_area *= 0.5;
                            // 3D points + integrals
                            let mut acc = Vector::new(0.0, 0.0, 0.0);
                            let mut h_loop = 0.0;
                            let mut wind = 0.0;
                            let mut prev_theta = 0.0;
                            let mut prev_p = Point::new(0.0, 0.0, 0.0);
                            let mut prev_h = 0.0;
                            for i in 0..uvpts.len() {
                                let p = srf.point_at(uvpts[i].0, uvpts[i].1).unwrap_or(Point::new(0.0, 0.0, 0.0));
                                let d = Vector::new(p[0] - cpt[0], p[1] - cpt[1], p[2] - cpt[2]);
                                let theta = dot3(&d, &ys).atan2(dot3(&d, &xs));
                                let h = dot3(&d, &zs);
                                if i > 0 {
                                    // vector area: 0.5 * sum P_i x P_{i+1}
                                    acc = Vector::new(
                                        acc[0] + prev_p[1] * p[2] - prev_p[2] * p[1],
                                        acc[1] + prev_p[2] * p[0] - prev_p[0] * p[2],
                                        acc[2] + prev_p[0] * p[1] - prev_p[1] * p[0]);
                                    let pi = std::f64::consts::PI;
                                    let mut dth = theta - prev_theta;
                                    while dth > pi { dth -= 2.0 * pi; }
                                    while dth < -pi { dth += 2.0 * pi; }
                                    h_loop += 0.5 * (h + prev_h) * dth;
                                    wind += dth;
                                }
                                prev_p = p; prev_theta = theta; prev_h = h;
                            }
                            if wind.abs() > std::f64::consts::PI { pole_face = true; }
                            let a = Vector::new(0.5 * acc[0], 0.5 * acc[1], 0.5 * acc[2]);
                            let loopflux = (cpt[0] * a[0] + cpt[1] * a[1] + cpt[2] * a[2]) - rr * rr * h_loop;
                            let osign = (if self.m_loops[li as usize].loop_type == BRepLoopType::Outer { 1.0 } else { -1.0 })
                                * (if uv_area >= 0.0 { 1.0 } else { -1.0 });
                            flux_analytic += osign * loopflux;
                        }
                    }
                }
            }
            // For a sphere cap-cut face (non-rectangular trim) the analytic boundary integral is
            // exact; the masked Gauss has a ~1% region error there. Keep Gauss for rectangular trims
            // (full sphere / equatorial band, already exact) and for non-sphere curved faces.
            if have_analytic && !curved_rect && !pole_face { flux_nat = flux_analytic; }
            total += sign * flux_nat;
        }
        total.abs() / 3.0
    }

    /// True if `p` is strictly inside the (closed) solid, by ray-cast parity against the
    /// tessellated boundary. Robust for interior points; matches OCCT
    /// BRepClass3d_SolidClassifier for IN/OUT.
    pub fn contains_point(&self, p: &Point) -> bool {
        self.contains_point_with(&self.mesh(), p)
    }

    /// As contains_point but reusing a precomputed boundary mesh.
    pub fn contains_point_with(&self, boundary: &crate::Mesh, p: &Point) -> bool {
        let (dx, dy, dz) = (0.5773502691_f64, 0.6539124_f64, 0.5023147_f64);  // irregular
        let big = 1e6;
        let ray = crate::line::Line::new(p[0], p[1], p[2],
                                         p[0] + dx*big, p[1] + dy*big, p[2] + dz*big);
        match crate::intersection::ray_mesh(&ray, boundary, 1e-9, true) {
            Some(hits) => hits.len() % 2 == 1,
            None => false,
        }
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

    /// Adaptive lift of a UV pcurve onto its surface: a straight 3D lift stays ~2 points; a
    /// 2-CV UV line that wraps a cylinder/sphere into a circle is subdivided to the chord
    /// tolerance, so the same intersection lifted from two surfaces agrees within the sew
    /// tolerance. Returns (c3d, start, end, parameter-midpoint) — the midpoint keys the edge.
    fn lift_loop(srf: &NurbsSurface, devtol: f64, pc: &NurbsCurve) -> (NurbsCurve, Point, Point, Point) {
        let (c0, c1) = pc.domain();
        let ev = |t: f64| -> Point {
            let uv = pc.point_at(t);
            srf.point_at(uv[0], uv[1]).unwrap_or(Point::new(uv[0], uv[1], 0.0))
        };

        fn rec(ev: &dyn Fn(f64) -> Point, ta: f64, pa: &Point, tb: f64, pb: &Point,
               depth: i32, devtol: f64, acc: &mut Vec<Point>) {
            let tm = 0.5 * (ta + tb);
            let pmid = ev(tm);
            let (ex, ey, ez) = (pb[0]-pa[0], pb[1]-pa[1], pb[2]-pa[2]);
            let l2 = ex*ex + ey*ey + ez*ez;
            let dev = if l2 > 1e-30 {
                let tt = ((pmid[0]-pa[0])*ex + (pmid[1]-pa[1])*ey + (pmid[2]-pa[2])*ez) / l2;
                let (cx, cy, cz) = (pa[0]+tt*ex, pa[1]+tt*ey, pa[2]+tt*ez);
                ((pmid[0]-cx).powi(2) + (pmid[1]-cy).powi(2) + (pmid[2]-cz).powi(2)).sqrt()
            } else {
                ((pmid[0]-pa[0]).powi(2) + (pmid[1]-pa[1]).powi(2) + (pmid[2]-pa[2]).powi(2)).sqrt()
            };
            if dev > devtol && depth < 9 {
                rec(ev, ta, pa, tm, &pmid, depth+1, devtol, acc);
                acc.push(pmid.clone());
                rec(ev, tm, &pmid, tb, pb, depth+1, devtol, acc);
            }
        }

        let n0 = pc.cv_count().saturating_sub(1).max(1);
        let mut pts3: Vec<Point> = Vec::new();
        let p_start = ev(c0);
        pts3.push(p_start.clone());
        let mut prev = p_start;
        for i in 0..n0 {
            let ta = c0 + (c1 - c0) * i as f64 / n0 as f64;
            let tb = c0 + (c1 - c0) * (i + 1) as f64 / n0 as f64;
            let pa = if i == 0 { prev.clone() } else { ev(ta) };
            let pb = ev(tb);
            rec(&ev, ta, &pa, tb, &pb, 0, devtol, &mut pts3);
            pts3.push(pb.clone());
            prev = pb;
        }
        let c3d = NurbsCurve::create(false, 1, &pts3);
        let pm = ev(0.5 * (c0 + c1));
        let p0 = pts3[0].clone();
        let p1 = pts3[pts3.len() - 1].clone();
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
        // One deviation tolerance per face (chord error target = 2e-3 of surface CV-box size,
        // still ~3x inside the sew tol diag*5e-3): straight lifts stay 2-pt, wrapped circles
        // subdivide to within the sew tolerance.
        let mut bmn = [1e30_f64; 3];
        let mut bmx = [-1e30_f64; 3];
        for ii in 0..srf.cv_count_dir(Some(0)) {
            for jj in 0..srf.cv_count_dir(Some(1)) {
                if let Some(p) = srf.get_cv(ii, jj) {
                    for k in 0..3 {
                        if p[k] < bmn[k] { bmn[k] = p[k]; }
                        if p[k] > bmx[k] { bmx[k] = p[k]; }
                    }
                }
            }
        }
        let sd = ((bmx[0]-bmn[0]).powi(2) + (bmx[1]-bmn[1]).powi(2) + (bmx[2]-bmn[2]).powi(2)).sqrt();
        let devtol = if sd < 1e-12 { 1.0 } else { sd } * 2e-3;
        for (ltype, pcs) in loops {
            let li = result.add_loop(fi, *ltype) as i32;
            for pc in pcs {
                if !pc.is_valid() {
                    continue;
                }
                let (c3d, p0, p1, pm) = Self::lift_loop(srf, devtol, pc);
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
                // Prefer per-run segmentation (each boundary run a separate pcurve edge) so runs
                // mate with the matching segment edge of an adjacent face -> watertight imprint.
                let mut loops: Vec<(BRepLoopType, Vec<NurbsCurve>)> = Vec::new();
                if !part.m_outer_segments.is_empty() {
                    loops.push((BRepLoopType::Outer, part.m_outer_segments.clone()));
                } else if let Some(ol) = &part.m_outer_loop {
                    loops.push((BRepLoopType::Outer, vec![ol.clone()]));
                }
                for (k, il) in part.m_inner_loops.iter().enumerate() {
                    match part.m_inner_segments.get(k) {
                        Some(s) if !s.is_empty() => loops.push((BRepLoopType::Inner, s.clone())),
                        _ => loops.push((BRepLoopType::Inner, vec![il.clone()])),
                    }
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
    // Boolean operations
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Split an under-mated edge at interior points coinciding with another edge's endpoint,
    /// so a long edge spanning shorter coincident edges is broken to match them (T-junctions).
    pub fn imprint_edges(&mut self, tol: f64) {
        let mut tol = tol;
        if tol <= 0.0 { tol = self.brep_bbox_diag() * 1e-6; }
        let mut vpos: Vec<Option<Point>> = Vec::with_capacity(self.m_topology_vertices.len());
        for tv in &self.m_topology_vertices {
            if tv.point_index >= 0 && (tv.point_index as usize) < self.m_vertices.len() {
                vpos.push(Some(self.m_vertices[tv.point_index as usize].clone()));
            } else { vpos.push(None); }
        }
        let split_multi = |c: &NurbsCurve, params: &[f64]| -> Option<Vec<NurbsCurve>> {
            let mut sorted: Vec<f64> = params.to_vec();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let mut out: Vec<NurbsCurve> = Vec::new();
            let mut rem = c.clone();
            for &t in &sorted {
                let (l, r) = rem.split(t);
                if !l.is_valid() || !r.is_valid() { return None; }
                out.push(l); rem = r;
            }
            out.push(rem);
            Some(out)
        };
        let ne0 = self.m_topology_edges.len();
        for ei in 0..ne0 {
            if self.m_topology_edges[ei].trim_indices.len() >= 2 { continue; }
            let ci = self.m_topology_edges[ei].curve_3d_index;
            if ci < 0 || ci as usize >= self.m_curves_3d.len() { continue; }
            let c = self.m_curves_3d[ci as usize].clone();
            if !c.is_valid() { continue; }
            let pa = c.point_at_start();
            let pb = c.point_at_end();
            if pa.distance(&pb, None) < tol { continue; }
            let (cd0, cd1) = c.domain();
            let mut ebb = [1e300, 1e300, 1e300, -1e300, -1e300, -1e300];
            for k in 0..7 {
                let p = c.point_at(cd0 + (cd1 - cd0) * k as f64 / 6.0);
                for d in 0..3 {
                    if p[d] < ebb[d] { ebb[d] = p[d]; }
                    if p[d] > ebb[d + 3] { ebb[d + 3] = p[d]; }
                }
            }
            let mut splits: Vec<(f64, Point)> = Vec::new();
            for vo in &vpos {
                let v = match vo { Some(v) => v, None => continue };
                if v[0] < ebb[0] - tol || v[0] > ebb[3] + tol || v[1] < ebb[1] - tol ||
                   v[1] > ebb[4] + tol || v[2] < ebb[2] - tol || v[2] > ebb[5] + tol { continue; }
                if v.distance(&pa, None) < tol || v.distance(&pb, None) < tol { continue; }
                let tc = c.closest_parameter(v);
                if c.point_at(tc).distance(v, None) > tol { continue; }
                let frac = (tc - cd0) / (cd1 - cd0);
                if frac <= 1e-6 || frac >= 1.0 - 1e-6 { continue; }
                if splits.iter().any(|s| s.1.distance(v, None) < tol) { continue; }
                splits.push((tc, v.clone()));
            }
            if splits.is_empty() { continue; }
            splits.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            let params3: Vec<f64> = splits.iter().map(|s| s.0).collect();
            let c3pieces = match split_multi(&c, &params3) {
                Some(p) if p.len() == splits.len() + 1 => p,
                _ => continue,
            };
            let orig_trims: Vec<i32> = self.m_topology_edges[ei].trim_indices.clone();
            let mut vids: Vec<i32> = vec![self.m_topology_edges[ei].start_vertex];
            for (_, v) in &splits {
                let pidx = self.add_vertex(v) as i32;
                self.m_topology_vertices.push(BRepVertex { point_index: pidx, edge_indices: Vec::new() });
                vids.push((self.m_topology_vertices.len() - 1) as i32);
            }
            vids.push(self.m_topology_edges[ei].end_vertex);
            let mut edge_ids: Vec<i32> = Vec::new();
            for j in 0..c3pieces.len() {
                let (c3idx, eidx);
                if j == 0 {
                    c3idx = ci; eidx = ei as i32;
                    self.m_curves_3d[ci as usize] = c3pieces[0].clone();
                } else {
                    c3idx = self.add_curve_3d(&c3pieces[j]) as i32;
                    eidx = self.m_topology_edges.len() as i32;
                    self.m_topology_edges.push(BRepEdge { curve_3d_index: -1, start_vertex: -1, end_vertex: -1, trim_indices: Vec::new() });
                }
                let e = &mut self.m_topology_edges[eidx as usize];
                e.curve_3d_index = c3idx; e.start_vertex = vids[j]; e.end_vertex = vids[j + 1];
                e.trim_indices = Vec::new();
                edge_ids.push(eidx);
            }
            for ti in orig_trims {
                if ti < 0 || ti as usize >= self.m_trims.len() { continue; }
                let (li, treversed, ttype, c2);
                {
                    let t = &self.m_trims[ti as usize];
                    li = t.loop_index; treversed = t.reversed; ttype = t.trim_type; c2 = t.curve_2d_index;
                }
                let fi = if li >= 0 && (li as usize) < self.m_loops.len() { self.m_loops[li as usize].face_index } else { -1 };
                let si = if fi >= 0 && (fi as usize) < self.m_faces.len() { self.m_faces[fi as usize].surface_index } else { -1 };
                let mut p2pieces: Option<Vec<NurbsCurve>> = None;
                if si >= 0 && (si as usize) < self.m_surfaces.len() && c2 >= 0 && (c2 as usize) < self.m_curves_2d.len() {
                    let s = self.m_surfaces[si as usize].clone();
                    let p = self.m_curves_2d[c2 as usize].clone();
                    let mut tps: Vec<f64> = Vec::new();
                    for (_, v) in &splits {
                        let (uu, vv, _dd) = crate::closest::Closest::surface_point(&s, v, 0.0, 0.0, 0.0, 0.0);
                        tps.push(p.closest_parameter(&Point::new(uu, vv, 0.0)));
                    }
                    p2pieces = split_multi(&p, &tps);
                }
                let ok = match &p2pieces { Some(pp) => pp.len() == edge_ids.len(), None => false };
                let mut newtrims: Vec<i32> = Vec::new();
                for j in 0..edge_ids.len() {
                    let pc: NurbsCurve = if ok {
                        p2pieces.as_ref().unwrap()[j].clone()
                    } else if j == 0 && c2 >= 0 {
                        self.m_curves_2d[c2 as usize].clone()
                    } else {
                        NurbsCurve::default()
                    };
                    let (c2idx, tidx);
                    if j == 0 {
                        if c2 >= 0 { self.m_curves_2d[c2 as usize] = pc; c2idx = c2; }
                        else { c2idx = self.add_curve_2d(&pc) as i32; }
                        tidx = ti;
                    } else {
                        c2idx = self.add_curve_2d(&pc) as i32;
                        tidx = self.m_trims.len() as i32;
                        self.m_trims.push(BRepTrim { curve_2d_index: -1, edge_index: -1, loop_index: -1, reversed: false, trim_type: BRepTrimType::Boundary });
                    }
                    {
                        let tr = &mut self.m_trims[tidx as usize];
                        tr.curve_2d_index = c2idx; tr.edge_index = edge_ids[j];
                        tr.loop_index = li; tr.reversed = treversed; tr.trim_type = ttype;
                    }
                    newtrims.push(tidx);
                    self.m_topology_edges[edge_ids[j] as usize].trim_indices.push(tidx);
                }
                if treversed { newtrims.reverse(); }
                if li >= 0 && (li as usize) < self.m_loops.len() {
                    let tl = &mut self.m_loops[li as usize].trim_indices;
                    if let Some(pos) = tl.iter().position(|&x| x == ti) {
                        tl.splice(pos..pos + 1, newtrims.iter().cloned());
                    }
                }
            }
        }
        for v in &mut self.m_topology_vertices { v.edge_indices.clear(); }
        for ei in 0..self.m_topology_edges.len() {
            let (sv, ev) = (self.m_topology_edges[ei].start_vertex, self.m_topology_edges[ei].end_vertex);
            if sv >= 0 && (sv as usize) < self.m_topology_vertices.len() {
                self.m_topology_vertices[sv as usize].edge_indices.push(ei as i32);
            }
            if ev != sv && ev >= 0 && (ev as usize) < self.m_topology_vertices.len() {
                self.m_topology_vertices[ev as usize].edge_indices.push(ei as i32);
            }
        }
    }

    #[inline]
    fn coref_cand(&self, e: usize) -> bool {
        self.m_topology_edges[e].trim_indices.len() < 2
            && self.m_topology_edges[e].curve_3d_index >= 0
            && (self.m_topology_edges[e].curve_3d_index as usize) < self.m_curves_3d.len()
    }

    /// Co-refine the A<->B section: where one operand imprinted the shared cut curve as a single
    /// closed circle and the other as 2+ open arcs (or partially-overlapping arcs), split the
    /// longer at the shorter's endpoints so they mate 1:1. Strictly coincidence-gated. After this,
    /// sew merges segments that are arc-for-arc identical (the OCCT "shared section edge" guarantee).
    pub fn co_refine_coincident_edges(&mut self, tol: f64) {
        // point-to-polyline distance (nested fn: no environment capture)
        fn p2pl(p: &Point, pts: &[Point]) -> f64 {
            let mut best = 1e300;
            for j in 0..pts.len().saturating_sub(1) {
                let a = &pts[j]; let b = &pts[j + 1];
                let (ex, ey, ez) = (b[0] - a[0], b[1] - a[1], b[2] - a[2]);
                let l2 = ex * ex + ey * ey + ez * ez;
                let mut t = if l2 > 1e-30 { ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey + (p[2] - a[2]) * ez) / l2 } else { 0.0 };
                t = t.max(0.0).min(1.0);
                let (cx, cy, cz) = (a[0] + t * ex, a[1] + t * ey, a[2] + t * ez);
                let d = ((p[0] - cx).powi(2) + (p[1] - cy).powi(2) + (p[2] - cz).powi(2)).sqrt();
                if d < best { best = d; }
            }
            best
        }
        let diag = self.brep_bbox_diag();
        let mut tol = tol;
        if tol <= 0.0 { tol = if diag > 0.0 { diag * 5e-3 } else { 5e-3 }; }

        let split_multi = |c: &NurbsCurve, params: &[f64]| -> Option<Vec<NurbsCurve>> {
            let mut sorted: Vec<f64> = params.to_vec();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let mut out: Vec<NurbsCurve> = Vec::new();
            let mut rem = c.clone();
            for &t in &sorted {
                let (l, r) = rem.split(t);
                if !l.is_valid() || !r.is_valid() { return None; }
                out.push(l); rem = r;
            }
            out.push(rem);
            Some(out)
        };

        let ne0 = self.m_topology_edges.len();
        const NS: usize = 24;
        let mut samp: Vec<Vec<Point>> = vec![Vec::new(); ne0];
        let mut bbox: Vec<Option<[f64; 6]>> = vec![None; ne0];
        for e in 0..ne0 {
            if !self.coref_cand(e) { continue; }
            let ci = self.m_topology_edges[e].curve_3d_index as usize;
            let cc = &self.m_curves_3d[ci];
            let (t0, t1) = cc.domain();
            let mut bb = [1e300, 1e300, 1e300, -1e300, -1e300, -1e300];
            for k in 0..=NS {
                let p = cc.point_at(t0 + (t1 - t0) * k as f64 / NS as f64);
                for d in 0..3 {
                    if p[d] < bb[d] { bb[d] = p[d]; }
                    if p[d] > bb[d + 3] { bb[d + 3] = p[d]; }
                }
                samp[e].push(p);
            }
            bbox[e] = Some(bb);
        }
        let bbox_far = |i: usize, j: usize| -> bool {
            let a = match bbox[i] { Some(a) => a, None => return true };
            let b = match bbox[j] { Some(b) => b, None => return true };
            a[0] > b[3] + tol || b[0] > a[3] + tol || a[1] > b[4] + tol ||
            b[1] > a[4] + tol || a[2] > b[5] + tol || b[2] > a[5] + tol
        };
        // ej is an arc-SUBSET of ei: every ej sample lies within tol of ei's polyline. (One-directional;
        // a full circle is NOT a subset of one of its arcs, but each arc IS a subset of the circle.)
        let subset_of = |ej: usize, ei: usize| -> bool {
            if samp[ej].len() < 2 || samp[ei].len() < 2 { return false; }
            for p in &samp[ej] { if p2pl(p, &samp[ei]) > tol { return false; } }
            true
        };

        for ei in 0..ne0 {
            if !self.coref_cand(ei) { continue; }
            let c = self.m_curves_3d[self.m_topology_edges[ei].curve_3d_index as usize].clone();
            if !c.is_valid() { continue; }
            let dom = c.domain();
            let pa = c.point_at_start();
            let pb = c.point_at_end();
            let closed = pa.distance(&pb, None) < tol;

            // Split points on C = endpoints of DISTINCT under-mated edges that are arc-subsets of C and
            // land strictly interior on C (a circle split at its coincident arcs' shared endpoints).
            let mut sp: Vec<(f64, Point)> = Vec::new();
            let mut seam_has_split = false;
            for ej in 0..ne0 {
                if ej == ei || !self.coref_cand(ej) || bbox_far(ei, ej) || !subset_of(ej, ei) { continue; }
                let cj_ci = self.m_topology_edges[ej].curve_3d_index as usize;
                let cj_start = self.m_curves_3d[cj_ci].point_at_start();
                let cj_end = self.m_curves_3d[cj_ci].point_at_end();
                // Only an OPEN arc subdivides ei at its endpoints. A closed circle coincident with ei
                // has only its arbitrary param-seam as an "endpoint" -- splitting there is spurious.
                if cj_start.distance(&cj_end, None) < tol { continue; }
                for v in [&cj_start, &cj_end] {
                    let tc = c.closest_parameter(v);
                    if c.point_at(tc).distance(v, None) > tol { continue; }
                    let frac = (tc - dom.0) / (dom.1 - dom.0);
                    if frac <= 1e-6 || frac >= 1.0 - 1e-6 { seam_has_split = true; continue; }
                    if sp.iter().any(|s| s.1.distance(v, None) < tol) { continue; }
                    sp.push((tc, v.clone()));
                }
            }
            if sp.is_empty() { continue; }
            sp.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            let iparams: Vec<f64> = sp.iter().map(|s| s.0).collect();
            let ipts: Vec<Point> = sp.iter().map(|s| s.1.clone()).collect();

            let c3pieces = match split_multi(&c, &iparams) {
                Some(p) if p.len() == iparams.len() + 1 => p,
                _ => continue,
            };

            // wrap-join the first+last 3D piece across the param seam ONLY for a closed edge whose seam
            // is interior to an arc (no split point at the seam).
            let do_wrap = closed && !seam_has_split;

            let orig_trims: Vec<i32> = self.m_topology_edges[ei].trim_indices.clone();
            // vertices at the interior split points
            let mut svids: Vec<i32> = Vec::new();
            for v in &ipts {
                let pidx = self.add_vertex(v) as i32;
                self.m_topology_vertices.push(BRepVertex { point_index: pidx, edge_indices: Vec::new() });
                svids.push((self.m_topology_vertices.len() - 1) as i32);
            }
            // edge-piece curves + their (startV,endV)
            let mut arcs: Vec<NurbsCurve> = Vec::new();
            let mut arc_v: Vec<(i32, i32)> = Vec::new();
            if do_wrap {
                let wrap = NurbsCurve::join(&[c3pieces[c3pieces.len() - 1].clone(), c3pieces[0].clone()], Some(tol));
                if wrap.len() != 1 || !wrap[0].is_valid() { continue; }
                for k in 1..c3pieces.len() - 1 { arcs.push(c3pieces[k].clone()); arc_v.push((svids[k - 1], svids[k])); }
                arcs.push(wrap[0].clone());
                arc_v.push((svids[svids.len() - 1], svids[0]));
            } else {
                let mut vids: Vec<i32> = vec![self.m_topology_edges[ei].start_vertex];
                for &v in &svids { vids.push(v); }
                vids.push(self.m_topology_edges[ei].end_vertex);
                for k in 0..c3pieces.len() { arcs.push(c3pieces[k].clone()); arc_v.push((vids[k], vids[k + 1])); }
            }

            // create the piece edges (piece 0 reuses ei + its 3D-curve slot)
            let mut edge_ids: Vec<i32> = Vec::new();
            for k in 0..arcs.len() {
                let (c3idx, eidx);
                if k == 0 {
                    c3idx = self.m_topology_edges[ei].curve_3d_index;
                    eidx = ei as i32;
                    self.m_curves_3d[c3idx as usize] = arcs[0].clone();
                } else {
                    c3idx = self.add_curve_3d(&arcs[k]) as i32;
                    eidx = self.m_topology_edges.len() as i32;
                    self.m_topology_edges.push(BRepEdge { curve_3d_index: -1, start_vertex: -1, end_vertex: -1, trim_indices: Vec::new() });
                }
                let e = &mut self.m_topology_edges[eidx as usize];
                e.curve_3d_index = c3idx;
                e.start_vertex = arc_v[k].0;
                e.end_vertex = arc_v[k].1;
                e.trim_indices.clear();
                edge_ids.push(eidx);
            }

            // split each trim's 2D pcurve at the same params (project the split points onto the surface)
            for ti in orig_trims {
                if ti < 0 || ti as usize >= self.m_trims.len() { continue; }
                let (li, treversed, ttype, c2);
                {
                    let t = &self.m_trims[ti as usize];
                    li = t.loop_index; treversed = t.reversed; ttype = t.trim_type; c2 = t.curve_2d_index;
                }
                let fi = if li >= 0 && (li as usize) < self.m_loops.len() { self.m_loops[li as usize].face_index } else { -1 };
                let si = if fi >= 0 && (fi as usize) < self.m_faces.len() { self.m_faces[fi as usize].surface_index } else { -1 };
                // Build each arc's 2D pcurve. PLANAR face: project the (already-split) 3D arc onto the
                // plane (exact, cheap). CURVED face: split the original open pcurve at the projected
                // params (projecting onto a curved surface per-sample is slow/unstable near singularities).
                let mut p2: Vec<NurbsCurve> = Vec::new();
                if si >= 0 && (si as usize) < self.m_surfaces.len() {
                    let s = self.m_surfaces[si as usize].clone();
                    if s.is_planar(1e-6) {
                        for arc3 in &arcs {
                            let ad = arc3.domain();
                            let n = (arc3.cv_count() * 2).max(12);
                            let mut uvs: Vec<Point> = Vec::new();
                            for sidx in 0..=n {
                                let p3 = arc3.point_at(ad.0 + (ad.1 - ad.0) * sidx as f64 / n as f64);
                                let (uu, vv, _dd) = crate::closest::Closest::surface_point(&s, &p3, 0.0, 0.0, 0.0, 0.0);
                                uvs.push(Point::new(uu, vv, 0.0));
                            }
                            p2.push(NurbsCurve::create(false, 1, &uvs));
                        }
                    } else if c2 >= 0 && (c2 as usize) < self.m_curves_2d.len() {
                        let p = self.m_curves_2d[c2 as usize].clone();
                        let mut tps: Vec<f64> = Vec::new();
                        for v in &ipts {
                            let (uu, vv, _dd) = crate::closest::Closest::surface_point(&s, v, 0.0, 0.0, 0.0, 0.0);
                            tps.push(p.closest_parameter(&Point::new(uu, vv, 0.0)));
                        }
                        p2 = split_multi(&p, &tps).unwrap_or_default();
                        if do_wrap && p2.len() >= 2 {
                            let w2 = NurbsCurve::join(&[p2[p2.len() - 1].clone(), p2[0].clone()], Some(tol));
                            if w2.len() == 1 && w2[0].is_valid() {
                                let mut a2: Vec<NurbsCurve> = p2[1..p2.len() - 1].to_vec();
                                a2.push(w2[0].clone());
                                p2 = a2;
                            } else {
                                p2.clear();
                            }
                        }
                    }
                }
                let ok = p2.len() == edge_ids.len();
                let mut newtrims: Vec<i32> = Vec::new();
                for k in 0..edge_ids.len() {
                    let pc: NurbsCurve = if ok {
                        p2[k].clone()
                    } else if k == 0 && c2 >= 0 {
                        self.m_curves_2d[c2 as usize].clone()
                    } else {
                        NurbsCurve::default()
                    };
                    let (c2idx, tidx);
                    if k == 0 {
                        if c2 >= 0 { self.m_curves_2d[c2 as usize] = pc; c2idx = c2; }
                        else { c2idx = self.add_curve_2d(&pc) as i32; }
                        tidx = ti;
                    } else {
                        c2idx = self.add_curve_2d(&pc) as i32;
                        tidx = self.m_trims.len() as i32;
                        self.m_trims.push(BRepTrim { curve_2d_index: -1, edge_index: -1, loop_index: -1, reversed: false, trim_type: BRepTrimType::Boundary });
                    }
                    {
                        let tr = &mut self.m_trims[tidx as usize];
                        tr.curve_2d_index = c2idx; tr.edge_index = edge_ids[k];
                        tr.loop_index = li; tr.reversed = treversed; tr.trim_type = ttype;
                    }
                    newtrims.push(tidx);
                    self.m_topology_edges[edge_ids[k] as usize].trim_indices.push(tidx);
                }
                if treversed { newtrims.reverse(); }
                if li >= 0 && (li as usize) < self.m_loops.len() {
                    let tl = &mut self.m_loops[li as usize].trim_indices;
                    if let Some(pos) = tl.iter().position(|&x| x == ti) {
                        tl.splice(pos..pos + 1, newtrims.iter().cloned());
                    }
                }
            }
        }
        // rebuild vertex->edge adjacency
        for v in &mut self.m_topology_vertices { v.edge_indices.clear(); }
        for ei in 0..self.m_topology_edges.len() {
            let (sv, ev) = (self.m_topology_edges[ei].start_vertex, self.m_topology_edges[ei].end_vertex);
            if sv >= 0 && (sv as usize) < self.m_topology_vertices.len() {
                self.m_topology_vertices[sv as usize].edge_indices.push(ei as i32);
            }
            if ev != sv && ev >= 0 && (ev as usize) < self.m_topology_vertices.len() {
                self.m_topology_vertices[ev as usize].edge_indices.push(ei as i32);
            }
        }
    }

    /// Merge edges whose 3D curves coincide (point-to-segment Hausdorff < tol) into single
    /// mated edges, so independently-imprinted intersection curves on A and B share one edge.
    pub fn sew_coincident_edges(&mut self, tol: f64) {
        let diag = self.brep_bbox_diag();
        let mut tol = tol;
        if tol <= 0.0 { tol = diag * 5e-3; }
        let ne = self.m_topology_edges.len();
        let ns = 16;
        let mut samp: Vec<Vec<Point>> = vec![Vec::new(); ne];
        let mut bbox: Vec<Option<[f64; 6]>> = vec![None; ne];
        // Only UNDER-mated edges (fewer than 2 trims) are sewing candidates: a shared
        // intersection curve is imprinted as a 1-trim edge on each side and the two halves must
        // merge into one 2-trim edge. Edges already 2-trim (the solids' own box/cyl/sphere edges)
        // are watertight and cannot legitimately coincide with another edge, so we skip sampling
        // AND comparing them -- turning the O(ne^2) sew into O(k^2) over the few intersection edges.
        let is_candidate = |ei: usize| self.m_topology_edges[ei].trim_indices.len() < 2;
        for ei in 0..ne {
            if !is_candidate(ei) { continue; }
            let ci = self.m_topology_edges[ei].curve_3d_index;
            if ci < 0 || ci as usize >= self.m_curves_3d.len() { continue; }
            let (t0, t1) = self.m_curves_3d[ci as usize].domain();
            let mut bb = [1e300, 1e300, 1e300, -1e300, -1e300, -1e300];
            for k in 0..=ns {
                let p = self.m_curves_3d[ci as usize].point_at(t0 + (t1 - t0) * k as f64 / ns as f64);
                for d in 0..3 {
                    if p[d] < bb[d] { bb[d] = p[d]; }
                    if p[d] > bb[d + 3] { bb[d + 3] = p[d]; }
                }
                samp[ei].push(p);
            }
            bbox[ei] = Some(bb);
        }
        let bbox_far = |i: usize, j: usize| -> bool {
            let a = bbox[i].unwrap(); let b = bbox[j].unwrap();
            a[0] > b[3] + tol || b[0] > a[3] + tol || a[1] > b[4] + tol ||
            b[1] > a[4] + tol || a[2] > b[5] + tol || b[2] > a[5] + tol
        };
        let pt_to_polyline = |p: &Point, pts: &[Point]| -> f64 {
            let mut best = 1e300;
            for j in 0..pts.len().saturating_sub(1) {
                let a = &pts[j]; let b = &pts[j + 1];
                let (ex, ey, ez) = (b[0] - a[0], b[1] - a[1], b[2] - a[2]);
                let l2 = ex * ex + ey * ey + ez * ez;
                let mut t = if l2 > 1e-30 { ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey + (p[2] - a[2]) * ez) / l2 } else { 0.0 };
                t = t.max(0.0).min(1.0);
                let (cx, cy, cz) = (a[0] + t * ex, a[1] + t * ey, a[2] + t * ez);
                let d = ((p[0] - cx).powi(2) + (p[1] - cy).powi(2) + (p[2] - cz).powi(2)).sqrt();
                if d < best { best = d; }
            }
            best
        };
        // Directed point-to-polyline Hausdorff both ways, but EARLY-EXIT as soon as any sample
        // exceeds `tol` (the curves are then provably not coincident). Identical accept/reject to
        // a full max-Hausdorff < tol, but rejection is O(1) instead of O(NS^2) in the common case.
        // The midpoint is checked first because non-coincident edges that share an endpoint
        // diverge most in the middle, so it rejects fastest.
        let coincident_within = |a: &[Point], b: &[Point]| -> bool {
            if a.len() < 2 || b.len() < 2 { return false; }
            if pt_to_polyline(&a[a.len()/2], b) > tol { return false; }
            if pt_to_polyline(&b[b.len()/2], a) > tol { return false; }
            for p in a { if pt_to_polyline(p, b) > tol { return false; } }
            for p in b { if pt_to_polyline(p, a) > tol { return false; } }
            true
        };
        let mut rep: Vec<i32> = vec![-1; ne];
        let mut reps: Vec<usize> = Vec::new();
        for ei in 0..ne {
            if samp[ei].is_empty() { rep[ei] = ei as i32; reps.push(ei); continue; }
            for &r in &reps {
                if !samp[r].is_empty() && bbox[ei].is_some() && bbox[r].is_some()
                   && !bbox_far(ei, r) && coincident_within(&samp[ei], &samp[r]) {
                    rep[ei] = r as i32; break;
                }
            }
            if rep[ei] < 0 { rep[ei] = ei as i32; reps.push(ei); }
        }
        let mut old2new: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
        let mut newedges: Vec<BRepEdge> = Vec::new();
        for &r in &reps {
            old2new.insert(r, newedges.len());
            let e = &self.m_topology_edges[r];
            newedges.push(BRepEdge { curve_3d_index: e.curve_3d_index, start_vertex: e.start_vertex, end_vertex: e.end_vertex, trim_indices: Vec::new() });
        }
        for ti in 0..self.m_trims.len() {
            let oe = self.m_trims[ti].edge_index;
            if oe < 0 || oe as usize >= ne { continue; }
            let ni = old2new[&(rep[oe as usize] as usize)];
            self.m_trims[ti].edge_index = ni as i32;
            newedges[ni].trim_indices.push(ti as i32);
        }
        self.m_topology_edges = newedges;
        for v in &mut self.m_topology_vertices { v.edge_indices.clear(); }
        for ei in 0..self.m_topology_edges.len() {
            let (sv, ev) = (self.m_topology_edges[ei].start_vertex, self.m_topology_edges[ei].end_vertex);
            if sv >= 0 && (sv as usize) < self.m_topology_vertices.len() {
                self.m_topology_vertices[sv as usize].edge_indices.push(ei as i32);
            }
            if ev != sv && ev >= 0 && (ev as usize) < self.m_topology_vertices.len() {
                self.m_topology_vertices[ev as usize].edge_indices.push(ei as i32);
            }
        }
    }

    /// Boolean of two solids via imprint -> classify -> select -> sew.
    /// op in {"union", "difference", "intersection"}.
    pub fn boolean(&self, other: &BRep, op: &str, tolerance: Option<f64>) -> BRep {
        use crate::polyline::Polyline;
        use crate::remesh_cdt::RemeshCDT;
        let a2 = self.split_by_brep(other, tolerance);
        let b2 = other.split_by_brep(self, tolerance);
        // Classify fragments against the OTHER solid. When an operand is a recognized primitive
        // (box/cylinder/sphere) the point-in-solid test is analytic (O(1)) so we skip building
        // its mesh AND the ray-cast; unrecognized (general/freeform) operands fall back to the
        // mesh ray-cast unchanged.
        let prim_a = recognize_solid(self);
        let prim_b = recognize_solid(other);
        let mesh_a = if prim_a.is_none() { Some(self.mesh()) } else { None };
        let mesh_b = if prim_b.is_none() { Some(other.mesh()) } else { None };

        let face_sample = |x: &BRep, fi: usize| -> Point {
            let face = &x.m_faces[fi];
            let srf = &x.m_surfaces[face.surface_index as usize];
            let mut outers: Vec<Polyline> = Vec::new();
            let mut inners: Vec<Polyline> = Vec::new();
            for &li in &face.loop_indices {
                if li < 0 || li as usize >= x.m_loops.len() { continue; }
                let mut pts: Vec<Point> = Vec::new();
                for &ti in &x.m_loops[li as usize].trim_indices {
                    if ti < 0 || ti as usize >= x.m_trims.len() { continue; }
                    let c2 = x.m_trims[ti as usize].curve_2d_index;
                    if c2 < 0 || c2 as usize >= x.m_curves_2d.len() { continue; }
                    let pc = &x.m_curves_2d[c2 as usize];
                    let (d0, d1) = pc.domain();
                    let n = (pc.cv_count() * 3).max(6);
                    for i in 0..n {
                        let uv = pc.point_at(d0 + (d1 - d0) * i as f64 / n as f64);
                        pts.push(Point::new(uv[0], uv[1], 0.0));
                    }
                }
                if pts.len() < 3 { continue; }
                if x.m_loops[li as usize].loop_type == BRepLoopType::Outer { outers.push(Polyline::new(pts)); }
                else { inners.push(Polyline::new(pts)); }
            }
            let fallback = || {
                let (u0, u1) = srf.domain(0).unwrap_or((0.0, 1.0));
                let (v0, v1) = srf.domain(1).unwrap_or((0.0, 1.0));
                srf.point_at(0.5 * (u0 + u1), 0.5 * (v0 + v1)).unwrap_or(Point::new(0.0, 0.0, 0.0))
            };
            if outers.is_empty() { return fallback(); }
            let mut allp: Vec<Polyline> = vec![outers[0].clone()];
            for ip in &inners { allp.push(ip.clone()); }
            let tris = RemeshCDT::triangulate(&allp);
            let mut flat: Vec<Point> = Vec::new();
            for pl in &allp { for p in pl.get_points() { flat.push(p); } }
            if tris.is_empty() || flat.is_empty() { return fallback(); }
            let mut best: i32 = -1; let mut best_area = -1.0;
            for (ti, t) in tris.iter().enumerate() {
                if t.2 >= flat.len() { continue; }
                let ar = ((flat[t.1][0] - flat[t.0][0]) * (flat[t.2][1] - flat[t.0][1])
                         - (flat[t.2][0] - flat[t.0][0]) * (flat[t.1][1] - flat[t.0][1])).abs() * 0.5;
                if ar > best_area { best_area = ar; best = ti as i32; }
            }
            if best < 0 { return fallback(); }
            let t = tris[best as usize];
            let cu = (flat[t.0][0] + flat[t.1][0] + flat[t.2][0]) / 3.0;
            let cv = (flat[t.0][1] + flat[t.1][1] + flat[t.2][1]) / 3.0;
            srf.point_at(cu, cv).unwrap_or(Point::new(0.0, 0.0, 0.0))
        };

        let classify = |x2: &BRep, solid: &BRep, solid_mesh: &Option<Mesh>, prim: &Option<PrimSolid>, is_first: bool| -> (Vec<usize>, Vec<bool>) {
            let mut kept = Vec::new();
            let mut rev = Vec::new();
            for fi in 0..x2.m_faces.len() {
                if x2.m_faces[fi].surface_index < 0 { continue; }
                let sp = face_sample(x2, fi);
                let inside = match prim {
                    Some(pr) => inside_prim(pr, &sp),
                    None => solid.contains_point_with(solid_mesh.as_ref().unwrap(), &sp),
                };
                let (keep, r);
                if op == "union" { keep = !inside; r = false; }
                else if op == "intersection" { keep = inside; r = false; }
                else if is_first { keep = !inside; r = false; }
                else { keep = inside; r = true; }
                if keep { kept.push(fi); rev.push(r); }
            }
            (kept, rev)
        };

        let (kept_a, _) = classify(&a2, other, &mesh_b, &prim_b, true);
        let (kept_b, rev_b) = classify(&b2, self, &mesh_a, &prim_a, false);

        let sub_a = a2.subset(&kept_a);
        let mut sub_b = b2.subset(&kept_b);
        for k in 0..rev_b.len().min(sub_b.m_faces.len()) {
            if rev_b[k] { sub_b.m_faces[k].reversed = !sub_b.m_faces[k].reversed; }
        }

        let mut result = sub_a;
        result.name = "boolean".to_string();
        let voff = result.m_vertices.len() as i32;
        let tvoff = result.m_topology_vertices.len() as i32;
        let soff = result.m_surfaces.len() as i32;
        let c2off = result.m_curves_2d.len() as i32;
        let c3off = result.m_curves_3d.len() as i32;
        let eoff = result.m_topology_edges.len() as i32;
        let loff = result.m_loops.len() as i32;
        let foff = result.m_faces.len() as i32;
        let toff = result.m_trims.len() as i32;
        for p in &sub_b.m_vertices { result.m_vertices.push(p.clone()); }
        for s in &sub_b.m_surfaces { result.m_surfaces.push(s.clone()); }
        for c in &sub_b.m_curves_2d { result.m_curves_2d.push(c.clone()); }
        for c in &sub_b.m_curves_3d { result.m_curves_3d.push(c.clone()); }
        for tv in &sub_b.m_topology_vertices {
            result.m_topology_vertices.push(BRepVertex { point_index: tv.point_index + voff, edge_indices: Vec::new() });
        }
        for e in &sub_b.m_topology_edges {
            result.m_topology_edges.push(BRepEdge {
                curve_3d_index: if e.curve_3d_index >= 0 { e.curve_3d_index + c3off } else { e.curve_3d_index },
                start_vertex: if e.start_vertex >= 0 { e.start_vertex + tvoff } else { e.start_vertex },
                end_vertex: if e.end_vertex >= 0 { e.end_vertex + tvoff } else { e.end_vertex },
                trim_indices: Vec::new(),
            });
        }
        for t in &sub_b.m_trims {
            result.m_trims.push(BRepTrim {
                curve_2d_index: if t.curve_2d_index >= 0 { t.curve_2d_index + c2off } else { t.curve_2d_index },
                edge_index: if t.edge_index >= 0 { t.edge_index + eoff } else { t.edge_index },
                loop_index: if t.loop_index >= 0 { t.loop_index + loff } else { t.loop_index },
                reversed: t.reversed,
                trim_type: t.trim_type,
            });
        }
        for lp in &sub_b.m_loops {
            result.m_loops.push(BRepLoop {
                trim_indices: lp.trim_indices.iter().map(|&ti| ti + toff).collect(),
                face_index: if lp.face_index >= 0 { lp.face_index + foff } else { lp.face_index },
                loop_type: lp.loop_type,
            });
        }
        for f in &sub_b.m_faces {
            result.m_faces.push(BRepFace {
                surface_index: if f.surface_index >= 0 { f.surface_index + soff } else { f.surface_index },
                loop_indices: f.loop_indices.iter().map(|&li| li + loff).collect(),
                reversed: f.reversed,
                facecolor: f.facecolor.clone(),
            });
        }
        for e in &mut result.m_topology_edges { e.trim_indices.clear(); }
        for ti in 0..result.m_trims.len() {
            let ei = result.m_trims[ti].edge_index;
            if ei >= 0 && (ei as usize) < result.m_topology_edges.len() {
                result.m_topology_edges[ei as usize].trim_indices.push(ti as i32);
            }
        }

        result.imprint_edges(0.0);
        // Co-refine the A<->B section: where one operand imprinted the shared curve as a single closed
        // circle and the other as 2+ arcs (or partially-overlapping arcs), split the longer at the
        // shorter's endpoints so they mate 1:1. Strictly coincidence-gated; each solid's own edges are
        // untouched. After it, sew merges segments that are arc-for-arc identical.
        if std::env::var("SESSION_NO_COREFINE").is_err() { result.co_refine_coincident_edges(0.0); }
        result.sew_coincident_edges(0.0);
        result
    }

    pub fn boolean_union(&self, other: &BRep, tolerance: Option<f64>) -> BRep { self.boolean(other, "union", tolerance) }
    pub fn boolean_difference(&self, other: &BRep, tolerance: Option<f64>) -> BRep { self.boolean(other, "difference", tolerance) }
    pub fn boolean_intersection(&self, other: &BRep, tolerance: Option<f64>) -> BRep { self.boolean(other, "intersection", tolerance) }

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

        // Phase 2: Mesh direct faces, extract boundary 3D points for shared edges
        let mut fmesh: Vec<Mesh> = (0..nf).map(|_| Mesh::new()).collect();
        let mut edge_bnd: std::collections::HashMap<i32, Vec<Point>> = std::collections::HashMap::new();
        for fi in 0..nf {
            if !face_direct[fi] { continue; }
            let face = &self.m_faces[fi];
            let srf = &self.m_surfaces[face.surface_index as usize];
            fmesh[fi] = srf.mesh();
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
                    // Only extract if shared with a CDT (non-direct) face.
                    let mut shared = false;
                    for &oti in &self.m_topology_edges[eidx as usize].trim_indices {
                        if oti == ti || oti < 0 || oti as usize >= self.m_trims.len() { continue; }
                        let oli = self.m_trims[oti as usize].loop_index;
                        if oli < 0 || oli as usize >= self.m_loops.len() { continue; }
                        let ofi = self.m_loops[oli as usize].face_index;
                        if ofi >= 0 && (ofi as usize) < nf && !face_direct[ofi as usize] { shared = true; break; }
                    }
                    if !shared { continue; }
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

        // Phase 3: Mesh CDT faces, using matched boundary points for shared edges
        for fi in 0..nf {
            if face_direct[fi] { continue; }
            let face = &self.m_faces[fi];
            if face.surface_index < 0 || face.surface_index as usize >= self.m_surfaces.len() { continue; }
            let srf = &self.m_surfaces[face.surface_index as usize];
            // Bilinear 3D->UV projection frame (valid for planar cap surfaces).
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
            let (u0c, u1c) = srf.domain(0).unwrap_or((0.0, 1.0));
            let (v0c, v1c) = srf.domain(1).unwrap_or((0.0, 1.0));
            let deg_u = srf.degree(0); let deg_v = srf.degree(1);
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
                            // A UV-straight edge can still map to a CURVED 3D path when it spans
                            // the surface's curved parametric direction (e.g. a cylinder rim along
                            // angular u). Sampling by its 2 CVs lets CDT chord-cut straight across
                            // the interior (a spurious membrane); densify along the curved span.
                            let sp = crv.get_cv(0); let ep = crv.get_cv(crv.cv_count().saturating_sub(1));
                            let mut span: f64 = 0.0;
                            if let (Some(sp), Some(ep)) = (&sp, &ep) {
                                if deg_u > 1 && (u1c - u0c) > 0.0 {
                                    span = span.max((ep[0]-sp[0]).abs() / (u1c - u0c));
                                }
                                if deg_v > 1 && (v1c - v0c) > 0.0 {
                                    span = span.max((ep[1]-sp[1]).abs() / (v1c - v0c));
                                }
                            }
                            if span > 1e-9 {
                                let n = ((span * 48.0).round() as i64).max(8) as usize;
                                let (pts, _) = crv.divide_by_count(n, true);
                                for k in 0..pts.len().saturating_sub(1) { loop_pts.push(pts[k].clone()); }
                            } else {
                                for k in 0..crv.cv_count().saturating_sub(1) {
                                    if let Some(p) = crv.get_cv(k) { loop_pts.push(p); }
                                }
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

        // Phase 4: Combine
        let mut all_polygons: Vec<Vec<Point>> = Vec::new();
        for fi in 0..nf {
            let face = &self.m_faces[fi];
            let fm = &fmesh[fi];
            if fm.face.is_empty() { continue; }
            // Reversed faces must have their triangle winding flipped so the facet
            // orientation matches the face's outward normal.
            for (_fk, fverts) in &fm.face {
                let mut poly: Vec<Point> = fverts.iter()
                    .filter_map(|vi| fm.vertex.get(vi).map(|v| Point::new(v.x, v.y, v.z)))
                    .collect();
                if face.reversed { poly.reverse(); }
                all_polygons.push(poly);
            }
        }
        Mesh::from_polylines(all_polygons, Some(1e-6))
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

    pub fn transform(&mut self, xf: &Xform) {
        for srf in &mut self.m_surfaces {
            srf.transform(xf);
        }
        for crv in &mut self.m_curves_3d {
            crv.transform(xf);
        }
        for pt in &mut self.m_vertices {
            let x = xf.m[0] * pt[0] + xf.m[4] * pt[1] + xf.m[8] * pt[2] + xf.m[12];
            let y = xf.m[1] * pt[0] + xf.m[5] * pt[1] + xf.m[9] * pt[2] + xf.m[13];
            let z = xf.m[2] * pt[0] + xf.m[6] * pt[1] + xf.m[10] * pt[2] + xf.m[14];
            *pt = Point::new(x, y, z);
        }
    }

    pub fn transformed(&self, xf: &Xform) -> Self {
        let mut b = self.clone();
        b.transform(xf);
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
        self.to_proto().encode_to_vec()
    }

    /// The proto struct itself — pb_dumps encodes it; Session embeds it directly.
    pub fn to_proto(&self) -> crate::proto::BRep {
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
                pointcolor: None })
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

        crate::proto::BRep {
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
        }
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        Self::from_proto(crate::proto::BRep::decode(data)?)
    }

    /// Build from an already-decoded proto — pb_loads decodes then calls this.
    pub fn from_proto(proto: crate::proto::BRep) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
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

// ---- Analytic primitive recognition for O(1) boolean classification ----
// A recognized solid lets boolean() classify fragment sample points without tessellating +
// ray-casting the operand. recognize_solid returns None for anything it can't prove is a
// box/cylinder/sphere, so GENERAL / freeform solids fall back to the mesh ray-cast and stay
// correct; recognition self-verifies so a wrong guess yields None, never a wrong answer.
struct PrimSolid {
    kind: i32, // 1 convex polyhedron, 2 cylinder, 3 sphere
    tol: f64,
    hs: Vec<[f64; 4]>,            // half-spaces: inside iff n.p <= d
    ca: Point, cd: Vector, ch: f64, cr: f64, // cylinder axis/dir/length/radius
    sc: Point, sr: f64,          // sphere centre/radius
}

fn srf_is_planar(s: &NurbsSurface) -> bool {
    let (u0, u1) = s.domain(0).unwrap_or((0.0, 1.0));
    let (v0, v1) = s.domain(1).unwrap_or((0.0, 1.0));
    let n0 = s.normal_at(0.5 * (u0 + u1), 0.5 * (v0 + v1));
    for &(uu, vv) in &[(0.25, 0.3), (0.75, 0.8)] {
        let n = s.normal_at(u0 + (u1 - u0) * uu, v0 + (v1 - v0) * vv);
        let cx = n0[1] * n[2] - n0[2] * n[1];
        let cy = n0[2] * n[0] - n0[0] * n[2];
        let cz = n0[0] * n[1] - n0[1] * n[0];
        if (cx * cx + cy * cy + cz * cz).sqrt() > 1e-7 { return false; }
    }
    true
}

fn recognize_solid(x: &BRep) -> Option<PrimSolid> {
    if x.m_faces.is_empty() || x.m_surfaces.is_empty() { return None; }
    // Geometry from SURFACE-sampled points, NOT m_vertices (stale vs surfaces after transformed()).
    let mut spts: Vec<Point> = Vec::new();
    for f in &x.m_faces {
        if f.surface_index < 0 || f.surface_index as usize >= x.m_surfaces.len() { return None; }
        let s = &x.m_surfaces[f.surface_index as usize];
        let (u0, u1) = s.domain(0).unwrap_or((0.0, 1.0));
        let (v0, v1) = s.domain(1).unwrap_or((0.0, 1.0));
        for &a in &[0.0, 0.5, 1.0] {
            for &b in &[0.0, 0.5, 1.0] {
                spts.push(s.point_at(u0 + (u1 - u0) * a, v0 + (v1 - v0) * b).unwrap_or(Point::new(0.0, 0.0, 0.0)));
            }
        }
    }
    if spts.is_empty() { return None; }
    let (mut xmn, mut ymn, mut zmn) = (1e300_f64, 1e300_f64, 1e300_f64);
    let (mut xmx, mut ymx, mut zmx) = (-1e300_f64, -1e300_f64, -1e300_f64);
    let (mut cx, mut cy, mut cz) = (0.0, 0.0, 0.0);
    for p in &spts {
        xmn = xmn.min(p[0]); ymn = ymn.min(p[1]); zmn = zmn.min(p[2]);
        xmx = xmx.max(p[0]); ymx = ymx.max(p[1]); zmx = zmx.max(p[2]);
        cx += p[0]; cy += p[1]; cz += p[2];
    }
    let nv = spts.len() as f64;
    let c = [cx / nv, cy / nv, cz / nv];
    let diag = ((xmx - xmn).powi(2) + (ymx - ymn).powi(2) + (zmx - zmn).powi(2)).sqrt();
    if diag < 1e-12 { return None; }
    let tol = diag * 1e-6;

    let all_planar = x.m_faces.iter().all(|f| srf_is_planar(&x.m_surfaces[f.surface_index as usize]));
    if all_planar {
        let mut hs: Vec<[f64; 4]> = Vec::new();
        for f in &x.m_faces {
            let s = &x.m_surfaces[f.surface_index as usize];
            let (u0, u1) = s.domain(0).unwrap_or((0.0, 1.0));
            let (v0, v1) = s.domain(1).unwrap_or((0.0, 1.0));
            let q = s.point_at(0.5 * (u0 + u1), 0.5 * (v0 + v1)).unwrap_or(Point::new(0.0, 0.0, 0.0));
            let mut n = s.normal_at(0.5 * (u0 + u1), 0.5 * (v0 + v1));
            if n[0] * (c[0] - q[0]) + n[1] * (c[1] - q[1]) + n[2] * (c[2] - q[2]) > 0.0 {
                n = Vector::new(-n[0], -n[1], -n[2]);
            }
            hs.push([n[0], n[1], n[2], n[0] * q[0] + n[1] * q[1] + n[2] * q[2]]);
        }
        for vtx in &spts {
            for h in &hs {
                if h[0] * vtx[0] + h[1] * vtx[1] + h[2] * vtx[2] > h[3] + tol * 50.0 { return None; }
            }
        }
        return Some(PrimSolid { kind: 1, tol, hs, ca: Point::new(0.0, 0.0, 0.0), cd: Vector::new(0.0, 0.0, 1.0), ch: 0.0, cr: 0.0, sc: Point::new(0.0, 0.0, 0.0), sr: 0.0 });
    }

    // Sphere: a single (non-planar) face. Least-squares centre fit, then verify equidistant.
    if x.m_faces.len() == 1 {
        let s = &x.m_surfaces[x.m_faces[0].surface_index as usize];
        let (u0, u1) = s.domain(0).unwrap_or((0.0, 1.0));
        let (v0, v1) = s.domain(1).unwrap_or((0.0, 1.0));
        let mut sp: Vec<Point> = Vec::new();
        for i in 1..6 {
            for j in 1..6 {
                sp.push(s.point_at(u0 + (u1 - u0) * i as f64 / 6.0, v0 + (v1 - v0) * j as f64 / 6.0).unwrap_or(Point::new(0.0, 0.0, 0.0)));
            }
        }
        let mut a = [[0.0_f64; 3]; 3];
        let mut bb = [0.0_f64; 3];
        let p0 = &sp[0];
        let p0d = p0[0] * p0[0] + p0[1] * p0[1] + p0[2] * p0[2];
        for i in 1..sp.len() {
            let row = [2.0 * (sp[i][0] - p0[0]), 2.0 * (sp[i][1] - p0[1]), 2.0 * (sp[i][2] - p0[2])];
            let rhs = sp[i][0] * sp[i][0] + sp[i][1] * sp[i][1] + sp[i][2] * sp[i][2] - p0d;
            for r in 0..3 {
                for cc in 0..3 { a[r][cc] += row[r] * row[cc]; }
                bb[r] += row[r] * rhs;
            }
        }
        let det3 = |m: &[[f64; 3]; 3]| -> f64 {
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        };
        let d = det3(&a);
        if d.abs() > 1e-12 {
            let mut cc = [0.0_f64; 3];
            for k in 0..3 {
                let mut m = a;
                for r in 0..3 { m[r][k] = bb[r]; }
                cc[k] = det3(&m) / d;
            }
            let center = Point::new(cc[0], cc[1], cc[2]);
            let rs: f64 = sp.iter().map(|p| ((p[0] - center[0]).powi(2) + (p[1] - center[1]).powi(2) + (p[2] - center[2]).powi(2)).sqrt()).sum();
            let radius = rs / sp.len() as f64;
            let mut ok = radius > tol;
            for p in &sp {
                let dd = ((p[0] - center[0]).powi(2) + (p[1] - center[1]).powi(2) + (p[2] - center[2]).powi(2)).sqrt();
                if (dd - radius).abs() > diag * 1e-3 { ok = false; break; }
            }
            if ok {
                return Some(PrimSolid { kind: 3, tol, hs: Vec::new(), ca: Point::new(0.0, 0.0, 0.0), cd: Vector::new(0.0, 0.0, 1.0), ch: 0.0, cr: 0.0, sc: center, sr: radius });
            }
        }
    }

    // Cylinder: exactly 2 planar caps + 1 curved lateral.
    let mut planar: Vec<usize> = Vec::new();
    let mut curved: Vec<usize> = Vec::new();
    for fi in 0..x.m_faces.len() {
        if srf_is_planar(&x.m_surfaces[x.m_faces[fi].surface_index as usize]) { planar.push(fi); } else { curved.push(fi); }
    }
    if planar.len() == 2 && curved.len() == 1 {
        let cap_circle = |fi: usize| -> Option<(Point, f64)> {
            let face = &x.m_faces[fi];
            let s = &x.m_surfaces[face.surface_index as usize];
            let mut bpts: Vec<Point> = Vec::new();
            for &li in &face.loop_indices {
                if li < 0 || li as usize >= x.m_loops.len() { continue; }
                if x.m_loops[li as usize].loop_type != BRepLoopType::Outer { continue; }
                for &ti in &x.m_loops[li as usize].trim_indices {
                    if ti < 0 || ti as usize >= x.m_trims.len() { continue; }
                    let c2 = x.m_trims[ti as usize].curve_2d_index;
                    if c2 < 0 || c2 as usize >= x.m_curves_2d.len() { continue; }
                    let pc = &x.m_curves_2d[c2 as usize];
                    let (d0, d1) = pc.domain();
                    for k in 0..16 {
                        let uv = pc.point_at(d0 + (d1 - d0) * k as f64 / 16.0);
                        bpts.push(s.point_at(uv[0], uv[1]).unwrap_or(Point::new(0.0, 0.0, 0.0)));
                    }
                }
            }
            if bpts.len() < 6 { return None; }
            let n = bpts.len() as f64;
            let mx = bpts.iter().map(|p| p[0]).sum::<f64>() / n;
            let my = bpts.iter().map(|p| p[1]).sum::<f64>() / n;
            let mz = bpts.iter().map(|p| p[2]).sum::<f64>() / n;
            let rs: f64 = bpts.iter().map(|p| ((p[0] - mx).powi(2) + (p[1] - my).powi(2) + (p[2] - mz).powi(2)).sqrt()).sum();
            let radius = rs / n;
            for p in &bpts {
                if (((p[0] - mx).powi(2) + (p[1] - my).powi(2) + (p[2] - mz).powi(2)).sqrt() - radius).abs() > diag * 1e-3 { return None; }
            }
            Some((Point::new(mx, my, mz), radius))
        };
        if let (Some((a0, r0)), Some((a1, r1))) = (cap_circle(planar[0]), cap_circle(planar[1])) {
            if (r0 - r1).abs() < diag * 1e-3 {
                let h = ((a1[0] - a0[0]).powi(2) + (a1[1] - a0[1]).powi(2) + (a1[2] - a0[2]).powi(2)).sqrt();
                if h > tol {
                    let d = Vector::new((a1[0] - a0[0]) / h, (a1[1] - a0[1]) / h, (a1[2] - a0[2]) / h);
                    let l = &x.m_surfaces[x.m_faces[curved[0]].surface_index as usize];
                    let (u0, u1) = l.domain(0).unwrap_or((0.0, 1.0));
                    let (v0, v1) = l.domain(1).unwrap_or((0.0, 1.0));
                    let mut ok = true;
                    let mut brk = false;
                    for i in 0..4 {
                        if brk { break; }
                        for j in 0..4 {
                            let p = l.point_at(u0 + (u1 - u0) * i as f64 / 3.0, v0 + (v1 - v0) * j as f64 / 3.0).unwrap_or(Point::new(0.0, 0.0, 0.0));
                            let w = [p[0] - a0[0], p[1] - a0[1], p[2] - a0[2]];
                            let t = w[0] * d[0] + w[1] * d[1] + w[2] * d[2];
                            let rad = (0.0_f64).max(w[0] * w[0] + w[1] * w[1] + w[2] * w[2] - t * t).sqrt();
                            if (rad - r0).abs() > diag * 1e-3 || t < -diag * 1e-3 || t > h + diag * 1e-3 { ok = false; brk = true; break; }
                        }
                    }
                    if ok {
                        return Some(PrimSolid { kind: 2, tol, hs: Vec::new(), ca: a0, cd: d, ch: h, cr: r0, sc: Point::new(0.0, 0.0, 0.0), sr: 0.0 });
                    }
                }
            }
        }
    }
    None
}

fn inside_prim(prim: &PrimSolid, p: &Point) -> bool {
    let tol = prim.tol;
    match prim.kind {
        1 => {
            for h in &prim.hs {
                if h[0] * p[0] + h[1] * p[1] + h[2] * p[2] > h[3] + tol { return false; }
            }
            true
        }
        2 => {
            let w = [p[0] - prim.ca[0], p[1] - prim.ca[1], p[2] - prim.ca[2]];
            let t = w[0] * prim.cd[0] + w[1] * prim.cd[1] + w[2] * prim.cd[2];
            if t < -tol || t > prim.ch + tol { return false; }
            let rad = (0.0_f64).max(w[0] * w[0] + w[1] * w[1] + w[2] * w[2] - t * t).sqrt();
            rad <= prim.cr + tol
        }
        3 => {
            let d = ((p[0] - prim.sc[0]).powi(2) + (p[1] - prim.sc[1]).powi(2) + (p[2] - prim.sc[2]).powi(2)).sqrt();
            d <= prim.sr + tol
        }
        _ => false,
    }
}
