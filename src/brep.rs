use crate::color::Color;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
use crate::vector::Vector;
use crate::xform::Xform;
use serde::ser::SerializeMap;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// TopAbs_Orientation: carried by the parent -> child reference, never by the shape.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BRepOrientation {
    Forward = 0,
    Reversed = 1,
    Internal = 2,
    External = 3,
}

impl BRepOrientation {
    pub fn to_str(self) -> &'static str {
        match self {
            BRepOrientation::Forward => "forward",
            BRepOrientation::Reversed => "reversed",
            BRepOrientation::Internal => "internal",
            BRepOrientation::External => "external",
        }
    }

    pub fn from_str(s: &str) -> Self {
        match s {
            "reversed" => BRepOrientation::Reversed,
            "internal" => BRepOrientation::Internal,
            "external" => BRepOrientation::External,
            _ => BRepOrientation::Forward,
        }
    }

    pub fn from_i32(v: i32) -> Self {
        match v {
            1 => BRepOrientation::Reversed,
            2 => BRepOrientation::Internal,
            3 => BRepOrientation::External,
            _ => BRepOrientation::Forward,
        }
    }
}

/// TopAbs::Reverse.
pub fn brep_reverse(o: BRepOrientation) -> BRepOrientation {
    match o {
        BRepOrientation::Forward => BRepOrientation::Reversed,
        BRepOrientation::Reversed => BRepOrientation::Forward,
        other => other,
    }
}

/// TopAbs::Compose: the orientation of a sub-shape reached through a parent with orientation `a`.
pub fn brep_compose(a: BRepOrientation, b: BRepOrientation) -> BRepOrientation {
    match a {
        BRepOrientation::Internal | BRepOrientation::External => a,
        BRepOrientation::Forward => b,
        BRepOrientation::Reversed => brep_reverse(b),
    }
}

/// TopoDS_Shape: an oriented reference to a sub-shape (index into the owning table).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BRepRef {
    pub index: i32,
    pub orientation: BRepOrientation,
}

impl BRepRef {
    pub fn new(index: i32, orientation: BRepOrientation) -> Self {
        BRepRef { index, orientation }
    }
}

/// BRep_TVertex.
#[derive(Debug, Clone, PartialEq)]
pub struct BRepVertex {
    pub point: Point,
    pub tolerance: f64,
}

/// BRep_CurveOnSurface / BRep_CurveOnClosedSurface. curve_2d_index_2 is the pcurve of the
/// REVERSED use of the edge on a closed surface (seam); -1 otherwise. Pcurves run in the
/// edge's own direction (OCCT SameParameter convention).
#[derive(Debug, Clone, PartialEq)]
pub struct BRepCurveOnSurface {
    pub surface_index: i32,
    pub curve_2d_index: i32,
    pub curve_2d_index_2: i32,
}

/// BRep_TEdge. curve_3d_index is -1 for a degenerated edge (sphere pole, cone apex).
#[derive(Debug, Clone, PartialEq)]
pub struct BRepEdge {
    pub curve_3d_index: i32,
    pub start_vertex: i32,
    pub end_vertex: i32,
    pub tolerance: f64,
    pub degenerated: bool,
    pub pcurves: Vec<BRepCurveOnSurface>,
}

/// TopoDS_TWire.
#[derive(Debug, Clone, PartialEq)]
pub struct BRepWire {
    pub edges: Vec<BRepRef>,
}

/// BRep_TFace. The first wire is the outer boundary.
#[derive(Debug, Clone, PartialEq)]
pub struct BRepFace {
    pub surface_index: i32,
    pub wires: Vec<BRepRef>,
    pub tolerance: f64,
    pub facecolor: Option<Color>,
}

/// TopoDS_TShell.
#[derive(Debug, Clone, PartialEq)]
pub struct BRepShell {
    pub faces: Vec<BRepRef>,
}

/// TopoDS_TSolid.
#[derive(Debug, Clone, PartialEq)]
pub struct BRepSolid {
    pub shells: Vec<BRepRef>,
}

const F: BRepOrientation = BRepOrientation::Forward;
const R: BRepOrientation = BRepOrientation::Reversed;

/// Bilinear planar patch: u runs p00 -> p10, v runs p00 -> p01, natural normal = u x v.
fn bilinear_patch(p00: &Point, p10: &Point, p01: &Point, p11: &Point) -> NurbsSurface {
    let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
    srf.set_cv(0, 0, p00);
    srf.set_cv(1, 0, p10);
    srf.set_cv(0, 1, p01);
    srf.set_cv(1, 1, p11);
    srf
}

/// Straight pcurve from (u0, v0) to (u1, v1).
fn uv_line(u0: f64, v0: f64, u1: f64, v1: f64) -> NurbsCurve {
    NurbsCurve::create(false, 1, &[Point::new(u0, v0, 0.0), Point::new(u1, v1, 0.0)])
}

/// Exact pcurve of a 3D curve lying on a bilinear planar patch: the affine image of its CVs.
fn project_to_patch(crv: &NurbsCurve, srf: &NurbsSurface) -> NurbsCurve {
    let p00 = srf.get_cv(0, 0).unwrap();
    let p10 = srf.get_cv(1, 0).unwrap();
    let p01 = srf.get_cv(0, 1).unwrap();
    let eu = [p10[0] - p00[0], p10[1] - p00[1], p10[2] - p00[2]];
    let ev = [p01[0] - p00[0], p01[1] - p00[1], p01[2] - p00[2]];
    let eu2 = eu[0] * eu[0] + eu[1] * eu[1] + eu[2] * eu[2];
    let ev2 = ev[0] * ev[0] + ev[1] * ev[1] + ev[2] * ev[2];
    let mut c2 = NurbsCurve::new(3, crv.is_rational(), crv.order(), crv.cv_count());
    for i in 0..crv.nurbsknot_count() {
        c2.set_nurbsknot(i, crv.nurbsknot(i).unwrap_or(0.0));
    }
    for i in 0..crv.cv_count() {
        let (wx, wy, wz, w) = crv.get_cv_4d(i).unwrap();
        let dx = wx / w - p00[0];
        let dy = wy / w - p00[1];
        let dz = wz / w - p00[2];
        let u = (dx * eu[0] + dy * eu[1] + dz * eu[2]) / eu2;
        let v = (dx * ev[0] + dy * ev[1] + dz * ev[2]) / ev2;
        if crv.is_rational() {
            c2.set_cv_4d(i, u * w, v * w, 0.0, w);
        } else {
            c2.set_cv(i, &Point::new(u, v, 0.0));
        }
    }
    c2
}

/// Signed area of a closed pcurve's sampled polygon (positive = counter-clockwise).
fn uv_signed_area(c2d: &NurbsCurve) -> f64 {
    let (pts, _) = c2d.divide_by_count((c2d.cv_count() * 4).max(16), true);
    let mut a = 0.0;
    for i in 0..pts.len().saturating_sub(1) {
        a += pts[i][0] * pts[i + 1][1] - pts[i + 1][0] * pts[i][1];
    }
    0.5 * a
}

fn polygon_signed_area(pts: &[Point]) -> f64 {
    let n = pts.len();
    let mut a = 0.0;
    for i in 0..n {
        let p = &pts[i];
        let q = &pts[(i + 1) % n];
        a += p[0] * q[1] - q[0] * p[1];
    }
    0.5 * a
}

/// Shared builder for planar polygon faces whose vertices, edges (lo -> hi vertex) and
/// surfaces are made from a point table. The face order lists vertices counter-clockwise
/// seen from the outside, so the natural normal of the patch points outward (Forward face).
struct PolyFaceBuilder {
    edge_map: std::collections::HashMap<(usize, usize), usize>,
}

impl PolyFaceBuilder {
    fn new() -> Self {
        PolyFaceBuilder { edge_map: std::collections::HashMap::new() }
    }

    fn edge(&mut self, b: &mut BRep, v0: usize, v1: usize) -> usize {
        let (lo, hi) = (v0.min(v1), v0.max(v1));
        if let Some(&ei) = self.edge_map.get(&(lo, hi)) {
            return ei;
        }
        let line = NurbsCurve::create(false, 1, &[b.m_vertices[lo].point.clone(), b.m_vertices[hi].point.clone()]);
        let ci = b.add_curve_3d(&line);
        let ei = b.add_edge(ci as i32, lo as i32, hi as i32);
        self.edge_map.insert((lo, hi), ei);
        ei
    }

    fn wire_refs(&mut self, b: &mut BRep, si: usize, vi: &[usize]) -> Vec<BRepRef> {
        let n = vi.len();
        let mut refs = Vec::new();
        for i in 0..n {
            let (va, vb) = (vi[i], vi[(i + 1) % n]);
            let ei = self.edge(b, va, vb);
            let c2d = project_to_patch(&b.m_curves_3d[b.m_edges[ei].curve_3d_index as usize], &b.m_surfaces[si]);
            let ci = b.add_curve_2d(&c2d);
            b.add_pcurve(ei, si, ci as i32, -1);
            refs.push(BRepRef::new(ei as i32, if b.m_edges[ei].start_vertex == va as i32 { F } else { R }));
        }
        refs
    }

    fn face(&mut self, b: &mut BRep, srf: &NurbsSurface, vi: &[usize]) -> usize {
        let si = b.add_surface(srf);
        let refs = self.wire_refs(b, si, vi);
        let wi = b.add_wire(&refs);
        b.add_face(si as i32, &[BRepRef::new(wi as i32, F)], 0.0)
    }
}

const BOX_FACES: [[usize; 4]; 6] = [
    [0, 3, 2, 1], // bottom (z=-hz), normal -Z
    [4, 5, 6, 7], // top (z=+hz), normal +Z
    [0, 1, 5, 4], // front (y=-hy), normal -Y
    [1, 2, 6, 5], // right (x=+hx), normal +X
    [2, 3, 7, 6], // back (y=+hy), normal +Y
    [3, 0, 4, 7], // left (x=-hx), normal -X
];

/// Bilinear patch spanned by four vertex indices in face order (p00, p10, p11, p01).
fn quad_patch(b: &BRep, fv: &[usize; 4]) -> NurbsSurface {
    bilinear_patch(&b.m_vertices[fv[0]].point, &b.m_vertices[fv[1]].point,
                   &b.m_vertices[fv[3]].point, &b.m_vertices[fv[2]].point)
}

fn box_corners(b: &mut BRep, sx: f64, sy: f64, sz: f64) {
    let (hx, hy, hz) = (sx * 0.5, sy * 0.5, sz * 0.5);
    b.add_vertex(&Point::new(-hx, -hy, -hz), 0.0);
    b.add_vertex(&Point::new(hx, -hy, -hz), 0.0);
    b.add_vertex(&Point::new(hx, hy, -hz), 0.0);
    b.add_vertex(&Point::new(-hx, hy, -hz), 0.0);
    b.add_vertex(&Point::new(-hx, -hy, hz), 0.0);
    b.add_vertex(&Point::new(hx, -hy, hz), 0.0);
    b.add_vertex(&Point::new(hx, hy, hz), 0.0);
    b.add_vertex(&Point::new(-hx, hy, hz), 0.0);
}

/// Planar cap at height z with natural normal +Z (up) or -Z (down), spanning [-r, r]^2.
fn cap_patch(r: f64, z: f64, up: bool) -> NurbsSurface {
    if up {
        bilinear_patch(&Point::new(-r, -r, z), &Point::new(r, -r, z), &Point::new(-r, r, z), &Point::new(r, r, z))
    } else {
        bilinear_patch(&Point::new(-r, -r, z), &Point::new(-r, r, z), &Point::new(r, -r, z), &Point::new(r, r, z))
    }
}

/// Cap face bounded by one closed edge: outer wire counter-clockwise in the patch's UV.
fn cap_face(b: &mut BRep, cap: &NurbsSurface, edge: usize) -> usize {
    let si = b.add_surface(cap);
    let c2d = project_to_patch(&b.m_curves_3d[b.m_edges[edge].curve_3d_index as usize], cap);
    let o = if uv_signed_area(&c2d) > 0.0 { F } else { R };
    let ci = b.add_curve_2d(&c2d);
    b.add_pcurve(edge, si, ci as i32, -1);
    let wi = b.add_wire(&[BRepRef::new(edge as i32, o)]);
    b.add_face(si as i32, &[BRepRef::new(wi as i32, F)], 0.0)
}

/// Periodic body face (cylinder / cone / bore): seam from v0 to v1 at u0 == u1, bottom ring
/// forward at v0, top ring (or degenerated apex) reversed at v1.
fn body_face(b: &mut BRep, si: usize, e_bot: usize, e_seam: usize, e_top: usize) -> usize {
    let (u0, u1) = b.m_surfaces[si].domain(0).unwrap();
    let (v0, v1) = b.m_surfaces[si].domain(1).unwrap();
    let c_bot = b.add_curve_2d(&uv_line(u0, v0, u1, v0));
    b.add_pcurve(e_bot, si, c_bot as i32, -1);
    let c_top = b.add_curve_2d(&uv_line(u0, v1, u1, v1));
    b.add_pcurve(e_top, si, c_top as i32, -1);
    let c_right = b.add_curve_2d(&uv_line(u1, v0, u1, v1));
    let c_left = b.add_curve_2d(&uv_line(u0, v0, u0, v1));
    b.add_pcurve(e_seam, si, c_right as i32, c_left as i32);
    let wi = b.add_wire(&[BRepRef::new(e_bot as i32, F), BRepRef::new(e_seam as i32, F),
                          BRepRef::new(e_top as i32, R), BRepRef::new(e_seam as i32, R)]);
    b.add_face(si as i32, &[BRepRef::new(wi as i32, F)], 0.0)
}

/// Padded bilinear patch through `pts` in the plane (org, xa, ya).
fn planar_patch_through(pts: &[Point], org: &Point, xa: &Vector, ya: &Vector) -> NurbsSurface {
    let (mut umin, mut umax, mut vmin, mut vmax): (f64, f64, f64, f64) = (1e30, -1e30, 1e30, -1e30);
    for p in pts {
        let (dx, dy, dz) = (p[0] - org[0], p[1] - org[1], p[2] - org[2]);
        let u = dx * xa[0] + dy * xa[1] + dz * xa[2];
        let v = dx * ya[0] + dy * ya[1] + dz * ya[2];
        umin = umin.min(u); umax = umax.max(u);
        vmin = vmin.min(v); vmax = vmax.max(v);
    }
    let pad = (umax - umin).max(vmax - vmin) * 0.01;
    umin -= pad; umax += pad; vmin -= pad; vmax += pad;
    let pt3d = |u: f64, v: f64| Point::new(org[0] + u * xa[0] + v * ya[0], org[1] + u * xa[1] + v * ya[1], org[2] + u * xa[2] + v * ya[2]);
    bilinear_patch(&pt3d(umin, vmin), &pt3d(umax, vmin), &pt3d(umin, vmax), &pt3d(umax, vmax))
}

fn find_or_add_vertex(b: &mut BRep, p: &Point, tol: f64) -> usize {
    for (i, v) in b.m_vertices.iter().enumerate() {
        if v.point.distance(p, None) < tol { return i; }
    }
    b.add_vertex(p, 0.0)
}

fn cv_points(c: &NurbsCurve) -> Vec<Point> {
    let mut pts = Vec::new();
    for k in 0..c.cv_count() {
        if let Some((wx, wy, wz, w)) = c.get_cv_4d(k) {
            if w != 0.0 { pts.push(Point::new(wx / w, wy / w, wz / w)); }
        }
    }
    pts
}

/// Signed volume of face meshes (positive when the windings point outward).
fn signed_volume(meshes: &[Mesh]) -> f64 {
    let mut total = 0.0;
    for fm in meshes {
        for fverts in fm.face.values() {
            let a = fm.vertex[&fverts[0]].position();
            for k in 1..fverts.len().saturating_sub(1) {
                let b = fm.vertex[&fverts[k]].position();
                let c = fm.vertex[&fverts[k + 1]].position();
                total += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
            }
        }
    }
    total / 6.0
}

/// BRepBuilderAPI_Sewing + MakeSolid for free faces: when every edge is shared by exactly two
/// face uses, orient the faces consistently across shared edges, one shell per connected
/// component wound outward, and one solid per shell.
fn close_free_faces(b: &mut BRep) {
    let nf = b.face_count();
    if nf == 0 { return; }
    let mut uses: Vec<Vec<(usize, BRepOrientation)>> = vec![Vec::new(); b.m_edges.len()];
    for fi in 0..nf {
        for wr in &b.m_faces[fi].wires {
            for er in b.wire_edges(wr) { uses[er.index as usize].push((fi, er.orientation)); }
        }
    }
    if uses.iter().any(|u| u.len() != 2) { return; }
    let mut fo = vec![F; nf];
    let mut seen = vec![false; nf];
    let mut components: Vec<Vec<usize>> = Vec::new();
    for seed in 0..nf {
        if seen[seed] { continue; }
        let mut comp = Vec::new();
        let mut stack = vec![seed];
        seen[seed] = true;
        while let Some(fi) = stack.pop() {
            comp.push(fi);
            for wr in b.m_faces[fi].wires.clone() {
                for er in b.wire_edges(&wr) {
                    for &(g, og) in &uses[er.index as usize] {
                        if g == fi || seen[g] { continue; }
                        fo[g] = if og == er.orientation { brep_reverse(fo[fi]) } else { fo[fi] };
                        seen[g] = true;
                        stack.push(g);
                    }
                }
            }
        }
        components.push(comp);
    }
    let mut shells = Vec::new();
    for comp in &components {
        let refs: Vec<BRepRef> = comp.iter().map(|&fi| BRepRef::new(fi as i32, fo[fi])).collect();
        shells.push(BRepRef::new(b.add_shell(&refs) as i32, F));
    }
    let fm = b.face_meshes();
    for sr in shells {
        let part: Vec<Mesh> = b.m_shells[sr.index as usize].faces.iter().map(|fr| fm[fr.index as usize].clone()).collect();
        if signed_volume(&part) < 0.0 {
            for fr in &mut b.m_shells[sr.index as usize].faces { fr.orientation = brep_reverse(fr.orientation); }
        }
        b.add_solid(&[sr]);
    }
}

/// Boundary representation after OCCT's TopoDS/BRep model, with indexed tables.
///
/// Geometry pools (surfaces, 3D curves, 2D pcurves) and shape tables (vertices, edges, wires,
/// faces, shells, solids). Every parent -> child link is a BRepRef carrying the orientation.
/// The BRep itself is the compound: its free shapes are those no parent references.
#[derive(Debug, Clone)]
pub struct BRep {
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub width: f64,
    pub surfacecolor: Color,
    pub m_surfaces: Vec<NurbsSurface>,
    pub m_curves_3d: Vec<NurbsCurve>,
    pub m_curves_2d: Vec<NurbsCurve>,
    pub m_vertices: Vec<BRepVertex>,
    pub m_edges: Vec<BRepEdge>,
    pub m_wires: Vec<BRepWire>,
    pub m_faces: Vec<BRepFace>,
    pub m_shells: Vec<BRepShell>,
    pub m_solids: Vec<BRepSolid>,
}

impl PartialEq for BRep {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && self.width == other.width
            && self.surfacecolor == other.surfacecolor
            && self.m_surfaces.len() == other.m_surfaces.len()
            && self.m_vertices.len() == other.m_vertices.len()
            && self.m_edges.len() == other.m_edges.len()
            && self.m_wires.len() == other.m_wires.len()
            && self.m_faces.len() == other.m_faces.len()
            && self.m_shells.len() == other.m_shells.len()
            && self.m_solids.len() == other.m_solids.len()
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
            m_edges: Vec::new(),
            m_wires: Vec::new(),
            m_faces: Vec::new(),
            m_shells: Vec::new(),
            m_solids: Vec::new(),
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

    /// Clear the guid so a FRESH one mints lazily on next read.
    pub fn refresh_guid(&mut self) {
        self.guid = std::sync::OnceLock::new();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Static Factory Methods
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Axis-aligned box centered at the origin: 6 faces, 12 edges, 8 vertices, one solid.
    pub fn create_box(sx: f64, sy: f64, sz: f64) -> Self {
        let mut b = BRep::new();
        b.name = "box".to_string();
        box_corners(&mut b, sx, sy, sz);
        let mut pb = PolyFaceBuilder::new();
        let mut faces = Vec::new();
        for fv in &BOX_FACES {
            let srf = quad_patch(&b, fv);
            faces.push(BRepRef::new(pb.face(&mut b, &srf, fv) as i32, F));
        }
        let sh = b.add_shell(&faces);
        b.add_solid(&[BRepRef::new(sh as i32, F)]);
        b
    }

    /// Cylinder along +Z: one periodic body face (seam edge) and two planar caps.
    pub fn create_cylinder(radius: f64, height: f64) -> Self {
        use crate::primitives::Primitives;
        let mut b = BRep::new();
        b.name = "cylinder".to_string();
        let body = Primitives::cylinder_surface(0.0, 0.0, 0.0, radius, height);
        let p_bot = body.point_at_corner(0, 0).unwrap();
        let p_top = body.point_at_corner(0, 1).unwrap();
        let v_bot = b.add_vertex(&p_bot, 0.0) as i32;
        let v_top = b.add_vertex(&p_top, 0.0) as i32;
        let c_bot = b.add_curve_3d(&Primitives::circle(0.0, 0.0, 0.0, radius)) as i32;
        let e_bot = b.add_edge(c_bot, v_bot, v_bot);
        let c_top = b.add_curve_3d(&Primitives::circle(0.0, 0.0, height, radius)) as i32;
        let e_top = b.add_edge(c_top, v_top, v_top);
        let c_seam = b.add_curve_3d(&NurbsCurve::create(false, 1, &[p_bot, p_top])) as i32;
        let e_seam = b.add_edge(c_seam, v_bot, v_top);
        let si = b.add_surface(&body);
        let f_body = body_face(&mut b, si, e_bot, e_seam, e_top);
        let f_bot = cap_face(&mut b, &cap_patch(radius, 0.0, false), e_bot);
        let f_top = cap_face(&mut b, &cap_patch(radius, height, true), e_top);
        let sh = b.add_shell(&[BRepRef::new(f_body as i32, F), BRepRef::new(f_bot as i32, F), BRepRef::new(f_top as i32, F)]);
        b.add_solid(&[BRepRef::new(sh as i32, F)]);
        b
    }

    /// Sphere centered at the origin: one face, a seam meridian and two degenerated pole edges.
    pub fn create_sphere(radius: f64) -> Self {
        use crate::primitives::Primitives;
        let mut b = BRep::new();
        b.name = "sphere".to_string();
        let srf = Primitives::sphere_surface(0.0, 0.0, 0.0, radius);
        let (u0, u1) = srf.domain(0).unwrap();
        let (v0, v1) = srf.domain(1).unwrap();
        let v_s = b.add_vertex(&Point::new(0.0, 0.0, -radius), 0.0) as i32;
        let v_n = b.add_vertex(&Point::new(0.0, 0.0, radius), 0.0) as i32;
        let c_seam = b.add_curve_3d(&srf.iso_curve(1, u0).unwrap()) as i32;
        let e_seam = b.add_edge(c_seam, v_s, v_n);
        let e_south = b.add_edge(-1, v_s, v_s);
        let e_north = b.add_edge(-1, v_n, v_n);
        let si = b.add_surface(&srf);
        let c_south = b.add_curve_2d(&uv_line(u0, v0, u1, v0)) as i32;
        b.add_pcurve(e_south, si, c_south, -1);
        let c_north = b.add_curve_2d(&uv_line(u0, v1, u1, v1)) as i32;
        b.add_pcurve(e_north, si, c_north, -1);
        let c_right = b.add_curve_2d(&uv_line(u1, v0, u1, v1)) as i32;
        let c_left = b.add_curve_2d(&uv_line(u0, v0, u0, v1)) as i32;
        b.add_pcurve(e_seam, si, c_right, c_left);
        let wi = b.add_wire(&[BRepRef::new(e_south as i32, F), BRepRef::new(e_seam as i32, F),
                              BRepRef::new(e_north as i32, R), BRepRef::new(e_seam as i32, R)]);
        let fi = b.add_face(si as i32, &[BRepRef::new(wi as i32, F)], 0.0);
        let sh = b.add_shell(&[BRepRef::new(fi as i32, F)]);
        b.add_solid(&[BRepRef::new(sh as i32, F)]);
        b
    }

    /// Cone along +Z: base circle at z=0, apex at z=height (degenerated apex edge), planar base.
    pub fn create_cone(radius: f64, height: f64) -> Self {
        use crate::primitives::Primitives;
        let mut b = BRep::new();
        b.name = "cone".to_string();
        let body = Primitives::cone_surface(0.0, 0.0, 0.0, radius, height);
        let p_base = body.point_at_corner(0, 0).unwrap();
        let p_apex = Point::new(0.0, 0.0, height);
        let v_base = b.add_vertex(&p_base, 0.0) as i32;
        let v_apex = b.add_vertex(&p_apex, 0.0) as i32;
        let c_base = b.add_curve_3d(&Primitives::circle(0.0, 0.0, 0.0, radius)) as i32;
        let e_base = b.add_edge(c_base, v_base, v_base);
        let c_seam = b.add_curve_3d(&NurbsCurve::create(false, 1, &[p_base, p_apex])) as i32;
        let e_seam = b.add_edge(c_seam, v_base, v_apex);
        let e_apex = b.add_edge(-1, v_apex, v_apex);
        let si = b.add_surface(&body);
        let f_body = body_face(&mut b, si, e_base, e_seam, e_apex);
        let f_base = cap_face(&mut b, &cap_patch(radius, 0.0, false), e_base);
        let sh = b.add_shell(&[BRepRef::new(f_body as i32, F), BRepRef::new(f_base as i32, F)]);
        b.add_solid(&[BRepRef::new(sh as i32, F)]);
        b
    }

    /// Square pyramid: base edge `base` centered at the origin in z=0, apex at (0,0,height).
    pub fn create_pyramid(base: f64, height: f64) -> Self {
        let mut b = BRep::new();
        b.name = "pyramid".to_string();
        let h = base * 0.5;
        b.add_vertex(&Point::new(-h, -h, 0.0), 0.0);
        b.add_vertex(&Point::new(h, -h, 0.0), 0.0);
        b.add_vertex(&Point::new(h, h, 0.0), 0.0);
        b.add_vertex(&Point::new(-h, h, 0.0), 0.0);
        let v_apex = b.add_vertex(&Point::new(0.0, 0.0, height), 0.0);
        let mut pb = PolyFaceBuilder::new();
        let fv = [0usize, 3, 2, 1];
        let base_srf = quad_patch(&b, &fv);
        let mut faces = vec![BRepRef::new(pb.face(&mut b, &base_srf, &fv) as i32, F)];
        for i in 0..4usize {
            let (a, c) = (i, (i + 1) % 4);
            let srf = bilinear_patch(&b.m_vertices[a].point, &b.m_vertices[c].point, &b.m_vertices[v_apex].point, &b.m_vertices[v_apex].point);
            let si = b.add_surface(&srf);
            let e_ac = pb.edge(&mut b, a, c);
            let e_c = pb.edge(&mut b, c, v_apex);
            let e_a = pb.edge(&mut b, a, v_apex);
            let e_deg = b.add_edge(-1, v_apex as i32, v_apex as i32);
            let ac_fwd = b.m_edges[e_ac].start_vertex == a as i32;
            let c_ac = b.add_curve_2d(&if ac_fwd { uv_line(0.0, 0.0, 1.0, 0.0) } else { uv_line(1.0, 0.0, 0.0, 0.0) }) as i32;
            b.add_pcurve(e_ac, si, c_ac, -1);
            let c_c = b.add_curve_2d(&uv_line(1.0, 0.0, 1.0, 1.0)) as i32;
            b.add_pcurve(e_c, si, c_c, -1);
            let c_deg = b.add_curve_2d(&uv_line(1.0, 1.0, 0.0, 1.0)) as i32;
            b.add_pcurve(e_deg, si, c_deg, -1);
            let c_a = b.add_curve_2d(&uv_line(0.0, 0.0, 0.0, 1.0)) as i32;
            b.add_pcurve(e_a, si, c_a, -1);
            let wi = b.add_wire(&[BRepRef::new(e_ac as i32, if ac_fwd { F } else { R }), BRepRef::new(e_c as i32, F),
                                  BRepRef::new(e_deg as i32, F), BRepRef::new(e_a as i32, R)]);
            faces.push(BRepRef::new(b.add_face(si as i32, &[BRepRef::new(wi as i32, F)], 0.0) as i32, F));
        }
        let sh = b.add_shell(&faces);
        b.add_solid(&[BRepRef::new(sh as i32, F)]);
        b
    }

    /// Torus in the XY plane: one face closed in both directions, two seam edges, one vertex.
    pub fn create_torus(major_radius: f64, minor_radius: f64) -> Self {
        use crate::primitives::Primitives;
        let mut b = BRep::new();
        b.name = "torus".to_string();
        let srf = Primitives::torus_surface(0.0, 0.0, 0.0, major_radius, minor_radius);
        let (u0, u1) = srf.domain(0).unwrap();
        let (v0, v1) = srf.domain(1).unwrap();
        let v = b.add_vertex(&srf.point_at_corner(0, 0).unwrap(), 0.0) as i32;
        let c_u = b.add_curve_3d(&srf.iso_curve(1, u0).unwrap()) as i32;   // minor circle at u0
        let e_u = b.add_edge(c_u, v, v);
        let c_v = b.add_curve_3d(&srf.iso_curve(0, v0).unwrap()) as i32;   // major circle at v0
        let e_v = b.add_edge(c_v, v, v);
        let si = b.add_surface(&srf);
        let c_bottom = b.add_curve_2d(&uv_line(u0, v0, u1, v0)) as i32;
        let c_top = b.add_curve_2d(&uv_line(u0, v1, u1, v1)) as i32;
        b.add_pcurve(e_v, si, c_bottom, c_top);
        let c_right = b.add_curve_2d(&uv_line(u1, v0, u1, v1)) as i32;
        let c_left = b.add_curve_2d(&uv_line(u0, v0, u0, v1)) as i32;
        b.add_pcurve(e_u, si, c_right, c_left);
        let wi = b.add_wire(&[BRepRef::new(e_v as i32, F), BRepRef::new(e_u as i32, F),
                              BRepRef::new(e_v as i32, R), BRepRef::new(e_u as i32, R)]);
        let fi = b.add_face(si as i32, &[BRepRef::new(wi as i32, F)], 0.0);
        let sh = b.add_shell(&[BRepRef::new(fi as i32, F)]);
        b.add_solid(&[BRepRef::new(sh as i32, F)]);
        b
    }

    /// Axis-aligned box with a cylindrical through-hole along Z.
    pub fn create_block_with_hole(sx: f64, sy: f64, sz: f64, hole_radius: f64) -> Self {
        use crate::primitives::Primitives;
        let mut b = BRep::new();
        b.name = "block_with_hole".to_string();
        let hz = sz * 0.5;
        box_corners(&mut b, sx, sy, sz);
        let mut pb = PolyFaceBuilder::new();
        let mut faces = Vec::new();
        for fv in &BOX_FACES[2..] {
            let srf = quad_patch(&b, fv);
            faces.push(BRepRef::new(pb.face(&mut b, &srf, fv) as i32, F));
        }
        let p_bot = Point::new(hole_radius, 0.0, -hz);
        let p_top = Point::new(hole_radius, 0.0, hz);
        let v_bot = b.add_vertex(&p_bot, 0.0) as i32;
        let v_top = b.add_vertex(&p_top, 0.0) as i32;
        let c_bot = b.add_curve_3d(&Primitives::circle(0.0, 0.0, -hz, hole_radius)) as i32;
        let e_bot = b.add_edge(c_bot, v_bot, v_bot);
        let c_top = b.add_curve_3d(&Primitives::circle(0.0, 0.0, hz, hole_radius)) as i32;
        let e_top = b.add_edge(c_top, v_top, v_top);
        let c_seam = b.add_curve_3d(&NurbsCurve::create(false, 1, &[p_bot, p_top])) as i32;
        let e_seam = b.add_edge(c_seam, v_bot, v_top);
        let bore = Primitives::cylinder_surface(0.0, 0.0, -hz, hole_radius, sz);
        let si_bore = b.add_surface(&bore);
        faces.push(BRepRef::new(body_face(&mut b, si_bore, e_bot, e_seam, e_top) as i32, R));
        for fi in 0..2usize {
            let fv = &BOX_FACES[fi];
            let cap = quad_patch(&b, fv);
            let si = b.add_surface(&cap);
            let outer = pb.wire_refs(&mut b, si, fv);
            let e_hole = if fi == 0 { e_bot } else { e_top };
            let c2d = project_to_patch(&b.m_curves_3d[b.m_edges[e_hole].curve_3d_index as usize], &cap);
            let o = if uv_signed_area(&c2d) < 0.0 { F } else { R };
            let ci = b.add_curve_2d(&c2d) as i32;
            b.add_pcurve(e_hole, si, ci, -1);
            let w_outer = b.add_wire(&outer);
            let w_inner = b.add_wire(&[BRepRef::new(e_hole as i32, o)]);
            faces.push(BRepRef::new(b.add_face(si as i32, &[BRepRef::new(w_outer as i32, F), BRepRef::new(w_inner as i32, F)], 0.0) as i32, F));
        }
        let sh = b.add_shell(&faces);
        b.add_solid(&[BRepRef::new(sh as i32, F)]);
        b
    }

    /// One planar face per closed polyline; coincident vertices and edges are shared.
    pub fn from_polylines(polylines: &[crate::polyline::Polyline]) -> Self {
        let mut b = BRep::new();
        b.name = "polysurface".to_string();
        let tol = 1e-6;
        let mut pb = PolyFaceBuilder::new();
        for pl in polylines {
            let pts = pl.get_points();
            let n = if pl.is_closed() { pts.len().saturating_sub(1) } else { pts.len() };
            if n < 3 { continue; }
            let (org, plane) = pl.get_fast_plane();
            if !plane.is_valid() { continue; }
            let vi: Vec<usize> = (0..n).map(|i| find_or_add_vertex(&mut b, &pts[i], tol)).collect();
            let srf = planar_patch_through(&pts[..n], &org, &plane.x_axis(), &plane.y_axis());
            pb.face(&mut b, &srf, &vi);
        }
        close_free_faces(&mut b);
        b
    }

    /// One planar face per closed curve with optional hole curves (inner wires).
    pub fn from_nurbscurves(curves: &[NurbsCurve], holes: &[Vec<NurbsCurve>]) -> Self {
        use crate::polyline::Polyline;
        let mut b = BRep::new();
        b.name = "polysurface".to_string();
        let tol = 1e-6;
        fn curve_wire(b: &mut BRep, crv: &NurbsCurve, si: usize, tol: f64) -> usize {
            let sp = crv.point_at(crv.domain().0);
            let ep = crv.point_at(crv.domain().1);
            let vs = find_or_add_vertex(b, &sp, tol);
            let ve = if crv.is_closed() { vs } else { find_or_add_vertex(b, &ep, tol) };
            let ci = b.add_curve_3d(crv) as i32;
            let ei = b.add_edge(ci, vs as i32, ve as i32);
            let c2d = project_to_patch(crv, &b.m_surfaces[si]);
            let c2 = b.add_curve_2d(&c2d) as i32;
            b.add_pcurve(ei, si, c2, -1);
            b.add_wire(&[BRepRef::new(ei as i32, F)])
        }
        for (ci, crv) in curves.iter().enumerate() {
            let mut pts = cv_points(crv);
            if pts.len() >= 2 && pts[0].distance(&pts[pts.len() - 1], None) < tol { pts.pop(); }
            if pts.len() < 3 { continue; }
            let (org, plane) = Polyline::new(pts.clone()).get_fast_plane();
            if !plane.is_valid() { continue; }
            if ci < holes.len() {
                for h in &holes[ci] { pts.extend(cv_points(h)); }
            }
            let si = b.add_surface(&planar_patch_through(&pts, &org, &plane.x_axis(), &plane.y_axis()));
            let mut wires = vec![BRepRef::new(curve_wire(&mut b, crv, si, tol) as i32, F)];
            if ci < holes.len() {
                for h in &holes[ci] { wires.push(BRepRef::new(curve_wire(&mut b, h, si, tol) as i32, F)); }
            }
            b.add_face(si as i32, &wires, 0.0);
        }
        close_free_faces(&mut b);
        b
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Accessors
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn vertex_count(&self) -> usize { self.m_vertices.len() }
    pub fn edge_count(&self) -> usize { self.m_edges.len() }
    pub fn wire_count(&self) -> usize { self.m_wires.len() }
    pub fn face_count(&self) -> usize { self.m_faces.len() }
    pub fn shell_count(&self) -> usize { self.m_shells.len() }
    pub fn solid_count(&self) -> usize { self.m_solids.len() }

    /// Every reference resolves into its table, every face has a surface and an outer wire,
    /// every edge has two vertices and (unless degenerated) a 3D curve.
    pub fn is_valid(&self) -> bool {
        if self.m_faces.is_empty() { return false; }
        let ok = |i: i32, n: usize| i >= 0 && (i as usize) < n;
        for e in &self.m_edges {
            if !ok(e.start_vertex, self.m_vertices.len()) || !ok(e.end_vertex, self.m_vertices.len()) { return false; }
            if !e.degenerated && !ok(e.curve_3d_index, self.m_curves_3d.len()) { return false; }
            for pc in &e.pcurves {
                if !ok(pc.surface_index, self.m_surfaces.len()) || !ok(pc.curve_2d_index, self.m_curves_2d.len()) { return false; }
                if pc.curve_2d_index_2 >= 0 && !ok(pc.curve_2d_index_2, self.m_curves_2d.len()) { return false; }
            }
        }
        for w in &self.m_wires {
            if w.edges.is_empty() { return false; }
            if w.edges.iter().any(|r| !ok(r.index, self.m_edges.len())) { return false; }
        }
        for f in &self.m_faces {
            if !ok(f.surface_index, self.m_surfaces.len()) || f.wires.is_empty() { return false; }
            if f.wires.iter().any(|r| !ok(r.index, self.m_wires.len())) { return false; }
        }
        for s in &self.m_shells {
            if s.faces.iter().any(|r| !ok(r.index, self.m_faces.len())) { return false; }
        }
        for s in &self.m_solids {
            if s.shells.iter().any(|r| !ok(r.index, self.m_shells.len())) { return false; }
        }
        true
    }

    /// BRep_Tool::IsClosed(shell): every non-degenerated edge is used exactly twice by the
    /// shell's faces (a seam counts twice through its two pcurves).
    pub fn is_closed(&self, shell_index: usize) -> bool {
        if shell_index >= self.m_shells.len() { return false; }
        let mut uses = vec![0usize; self.m_edges.len()];
        for fr in &self.m_shells[shell_index].faces {
            for wr in &self.m_faces[fr.index as usize].wires {
                for er in self.wire_edges(wr) { uses[er.index as usize] += 1; }
            }
        }
        for (i, e) in self.m_edges.iter().enumerate() {
            if !e.degenerated && uses[i] != 0 && uses[i] != 2 { return false; }
        }
        !self.m_shells[shell_index].faces.is_empty()
    }

    /// At least one solid, and every shell of every solid is closed.
    pub fn is_solid(&self) -> bool {
        if self.m_solids.is_empty() { return false; }
        self.m_solids.iter().all(|s| s.shells.iter().all(|r| self.is_closed(r.index as usize)))
    }

    /// Orientation of a face inside its first parent shell; Forward for a free face.
    pub fn face_orientation(&self, face_index: usize) -> BRepOrientation {
        for s in &self.m_shells {
            for r in &s.faces {
                if r.index as usize == face_index { return r.orientation; }
            }
        }
        BRepOrientation::Forward
    }

    /// BRep_Tool::CurveOnSurface(E, F): the pcurve index of an edge on a face's surface for the
    /// given use orientation (the REVERSED pcurve on a seam); -1 if none.
    pub fn pcurve_index(&self, edge_index: usize, face_index: usize, orientation: BRepOrientation) -> i32 {
        if edge_index >= self.m_edges.len() || face_index >= self.m_faces.len() { return -1; }
        let si = self.m_faces[face_index].surface_index;
        for pc in &self.m_edges[edge_index].pcurves {
            if pc.surface_index == si {
                return if orientation == BRepOrientation::Reversed && pc.curve_2d_index_2 >= 0 { pc.curve_2d_index_2 } else { pc.curve_2d_index };
            }
        }
        -1
    }

    /// The edges of a wire composed with the wire's own orientation (a Reversed wire is
    /// traversed backwards with every edge reversed).
    pub fn wire_edges(&self, wire: &BRepRef) -> Vec<BRepRef> {
        if wire.index < 0 || wire.index as usize >= self.m_wires.len() { return Vec::new(); }
        let mut out: Vec<BRepRef> = self.m_wires[wire.index as usize].edges.iter()
            .map(|r| BRepRef::new(r.index, brep_compose(wire.orientation, r.orientation))).collect();
        if wire.orientation == BRepOrientation::Reversed { out.reverse(); }
        out
    }

    /// Faces sharing an edge, each with the orientation of that edge use.
    pub fn edge_faces(&self, edge_index: usize) -> Vec<BRepRef> {
        let mut out = Vec::new();
        for (fi, f) in self.m_faces.iter().enumerate() {
            let fo = self.face_orientation(fi);
            for wr in &f.wires {
                for er in self.wire_edges(wr) {
                    if er.index as usize == edge_index { out.push(BRepRef::new(fi as i32, brep_compose(fo, er.orientation))); }
                }
            }
        }
        out
    }

    /// Vertex positions, in vertex order.
    pub fn vertex_points(&self) -> Vec<Point> {
        self.m_vertices.iter().map(|v| v.point.clone()).collect()
    }

    /// BRepLib::UpdateTolerances: raise every edge tolerance to the worst distance between its
    /// curve ends (3D curve and each pcurve lifted through its surface) and its vertices, and
    /// every vertex tolerance to the worst incident edge end. Returns the largest tolerance.
    pub fn update_tolerances(&mut self) -> f64 {
        let mut worst: f64 = 0.0;
        for ei in 0..self.m_edges.len() {
            let (vs_i, ve_i) = (self.m_edges[ei].start_vertex as usize, self.m_edges[ei].end_vertex as usize);
            let vs = self.m_vertices[vs_i].point.clone();
            let ve = self.m_vertices[ve_i].point.clone();
            let mut tol: f64 = self.m_edges[ei].tolerance;
            if self.m_edges[ei].curve_3d_index >= 0 {
                let c = &self.m_curves_3d[self.m_edges[ei].curve_3d_index as usize];
                tol = tol.max(c.point_at(c.domain().0).distance(&vs, None));
                tol = tol.max(c.point_at(c.domain().1).distance(&ve, None));
            }
            for pc in &self.m_edges[ei].pcurves {
                let srf = &self.m_surfaces[pc.surface_index as usize];
                for ci in [pc.curve_2d_index, pc.curve_2d_index_2] {
                    if ci < 0 { continue; }
                    let c2 = &self.m_curves_2d[ci as usize];
                    let a = c2.point_at(c2.domain().0);
                    let z = c2.point_at(c2.domain().1);
                    if let Some(p) = srf.point_at(a[0], a[1]) { tol = tol.max(p.distance(&vs, None)); }
                    if let Some(p) = srf.point_at(z[0], z[1]) { tol = tol.max(p.distance(&ve, None)); }
                }
            }
            self.m_edges[ei].tolerance = tol;
            self.m_vertices[vs_i].tolerance = self.m_vertices[vs_i].tolerance.max(tol);
            self.m_vertices[ve_i].tolerance = self.m_vertices[ve_i].tolerance.max(tol);
            worst = worst.max(tol);
        }
        worst
    }

    /// Volume of the tessellated boundary (divergence theorem); meaningful for solids only.
    pub fn volume(&self) -> f64 {
        self.mesh().volume()
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Building (BRep_Builder)
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

    /// MakeVertex.
    pub fn add_vertex(&mut self, pt: &Point, tolerance: f64) -> usize {
        self.m_vertices.push(BRepVertex { point: pt.clone(), tolerance });
        self.m_vertices.len() - 1
    }

    /// MakeEdge: curve_3d_index -1 makes a degenerated edge (start == end vertex).
    pub fn add_edge(&mut self, curve_3d_index: i32, start_vertex: i32, end_vertex: i32) -> usize {
        self.m_edges.push(BRepEdge {
            curve_3d_index,
            start_vertex,
            end_vertex,
            tolerance: 0.0,
            degenerated: curve_3d_index < 0,
            pcurves: Vec::new(),
        });
        self.m_edges.len() - 1
    }

    /// UpdateEdge(E, pcurve, S): attach a pcurve on a surface; curve_2d_index_2 for the
    /// reversed use on a closed surface. Replaces an existing record for the same surface.
    pub fn add_pcurve(&mut self, edge_index: usize, surface_index: usize, curve_2d_index: i32, curve_2d_index_2: i32) {
        for pc in &mut self.m_edges[edge_index].pcurves {
            if pc.surface_index == surface_index as i32 {
                pc.curve_2d_index = curve_2d_index;
                pc.curve_2d_index_2 = curve_2d_index_2;
                return;
            }
        }
        self.m_edges[edge_index].pcurves.push(BRepCurveOnSurface { surface_index: surface_index as i32, curve_2d_index, curve_2d_index_2 });
    }

    /// MakeWire + Add(edges).
    pub fn add_wire(&mut self, edges: &[BRepRef]) -> usize {
        self.m_wires.push(BRepWire { edges: edges.to_vec() });
        self.m_wires.len() - 1
    }

    /// MakeFace(S) + Add(wires); the first wire is the outer boundary.
    pub fn add_face(&mut self, surface_index: i32, wires: &[BRepRef], tolerance: f64) -> usize {
        self.m_faces.push(BRepFace { surface_index, wires: wires.to_vec(), tolerance, facecolor: None });
        self.m_faces.len() - 1
    }

    /// MakeShell + Add(faces).
    pub fn add_shell(&mut self, faces: &[BRepRef]) -> usize {
        self.m_shells.push(BRepShell { faces: faces.to_vec() });
        self.m_shells.len() - 1
    }

    /// MakeSolid + Add(shells).
    pub fn add_solid(&mut self, shells: &[BRepRef]) -> usize {
        self.m_solids.push(BRepSolid { shells: shells.to_vec() });
        self.m_solids.len() - 1
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Meshing
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// UV polygon of one wire of a face (pcurves sampled in traversal order).
    fn wire_uv_points(&self, face_index: usize, wire: &BRepRef) -> Vec<Point> {
        let mut pts = Vec::new();
        for er in self.wire_edges(wire) {
            let ci = self.pcurve_index(er.index as usize, face_index, er.orientation);
            if ci < 0 { continue; }
            let crv = &self.m_curves_2d[ci as usize];
            let mut seg: Vec<Point> = if crv.degree() <= 1 && !crv.is_rational() {
                (0..crv.cv_count()).filter_map(|k| crv.get_cv(k)).collect()
            } else {
                crv.divide_by_count((crv.cv_count() * 4).max(16), true).0
            };
            if er.orientation == BRepOrientation::Reversed { seg.reverse(); }
            for k in 0..seg.len().saturating_sub(1) { pts.push(seg[k].clone()); }
        }
        pts
    }

    /// One welded triangle mesh of every face, wound to the face's outward orientation.
    pub fn mesh(&self) -> Mesh {
        let mut polygons: Vec<Vec<Point>> = Vec::new();
        for fm in self.face_meshes() {
            for fverts in fm.face.values() {
                polygons.push(fverts.iter().map(|vi| fm.vertex[vi].position()).collect());
            }
        }
        Mesh::from_polylines(polygons, Some(1e-6))
    }

    /// One mesh per face, in face order (vertices not shared across faces).
    pub fn face_meshes(&self) -> Vec<Mesh> {
        self.face_meshes_q(None)
    }

    /// As face_meshes with a tessellation-quality `(max_angle_deg, chord_factor)` override for
    /// the grid-meshed faces.
    pub fn face_meshes_q(&self, quality: Option<(f64, f64)>) -> Vec<Mesh> {
        use crate::nurbssurface_trimmed::NurbsSurfaceTrimmed;
        let nf = self.m_faces.len();

        // Phase 1: a face whose outer wire is the full UV rectangle (straight pcurves enclosing the
        // whole domain area, no holes) is meshed directly on the surface grid; everything else goes
        // through the trimmed CDT.
        let mut face_direct = vec![false; nf];
        for fi in 0..nf {
            let face = &self.m_faces[fi];
            let srf = &self.m_surfaces[face.surface_index as usize];
            if face.wires.len() != 1 { continue; }
            let mut all_linear = true;
            for er in self.wire_edges(&face.wires[0]) {
                let ci = self.pcurve_index(er.index as usize, fi, er.orientation);
                if ci < 0 { continue; }
                let c = &self.m_curves_2d[ci as usize];
                if c.degree() > 1 || c.is_rational() { all_linear = false; }
            }
            if !all_linear { continue; }
            let outer = self.wire_uv_points(fi, &face.wires[0]);
            if outer.len() < 3 { continue; }
            let (u0, u1) = srf.domain(0).unwrap_or((0.0, 1.0));
            let (v0, v1) = srf.domain(1).unwrap_or((0.0, 1.0));
            let domain_area = (u1 - u0) * (v1 - v0);
            face_direct[fi] = (polygon_signed_area(&outer).abs() - domain_area).abs() < 1e-3 * domain_area;
        }

        // Phase 2: direct faces. Record the 3D boundary discretisation along every edge shared
        // with a CDT face so both sides tessellate the seam with the same points.
        let mut fmesh: Vec<Mesh> = (0..nf).map(|_| Mesh::new()).collect();
        let mut edge_bnd: std::collections::HashMap<usize, Vec<Point>> = std::collections::HashMap::new();
        for fi in 0..nf {
            if !face_direct[fi] { continue; }
            let face = &self.m_faces[fi];
            let srf = &self.m_surfaces[face.surface_index as usize];
            fmesh[fi] = match quality {
                Some((a, c)) => crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid::from_u_v_q(srf.clone(), 0, 0, a, c),
                None => srf.mesh(),
            };
            let (u0, u1) = srf.domain(0).unwrap_or((0.0, 1.0));
            let (v0, v1) = srf.domain(1).unwrap_or((0.0, 1.0));
            let utol = (u1 - u0) * 0.001;
            let vtol = (v1 - v0) * 0.001;
            for er in self.wire_edges(&face.wires[0]) {
                let eidx = er.index as usize;
                if edge_bnd.contains_key(&eidx) { continue; }
                let shared = self.edge_faces(eidx).iter().any(|fr| fr.index as usize != fi && !face_direct[fr.index as usize]);
                if !shared { continue; }
                let ci = self.pcurve_index(eidx, fi, er.orientation);
                if ci < 0 { continue; }
                let c2d = &self.m_curves_2d[ci as usize];
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

        // Phase 3: CDT faces. Shared edges reuse the direct face's boundary points projected into
        // this face's planar patch; every other edge samples its own pcurve.
        for fi in 0..nf {
            if face_direct[fi] { continue; }
            let face = &self.m_faces[fi];
            let srf = &self.m_surfaces[face.surface_index as usize];
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
            for (wi, wr) in face.wires.iter().enumerate() {
                let mut loop_pts: Vec<Point> = Vec::new();
                for er in self.wire_edges(wr) {
                    let ci = self.pcurve_index(er.index as usize, fi, er.orientation);
                    if ci < 0 { continue; }
                    let crv = &self.m_curves_2d[ci as usize];
                    let mut seg: Vec<Point>;
                    if let (Some((a, eu, ev, eu2, ev2)), Some(bnd)) = (proj.as_ref(), edge_bnd.get(&(er.index as usize))) {
                        seg = bnd.iter().map(|pt| {
                            let d = [pt[0]-a[0], pt[1]-a[1], pt[2]-a[2]];
                            Point::new((d[0]*eu[0] + d[1]*eu[1] + d[2]*eu[2]) / *eu2, (d[0]*ev[0] + d[1]*ev[1] + d[2]*ev[2]) / *ev2, 0.0)
                        }).collect();
                        let start = crv.point_at(if er.orientation == BRepOrientation::Reversed { crv.domain().1 } else { crv.domain().0 });
                        if seg[0].distance(&start, None) > seg[seg.len() - 1].distance(&start, None) { seg.reverse(); }
                    } else {
                        seg = if crv.degree() <= 1 && !crv.is_rational() {
                            (0..crv.cv_count()).filter_map(|k| crv.get_cv(k)).collect()
                        } else {
                            crv.divide_by_count((crv.cv_count() * 4).max(16), true).0
                        };
                        if er.orientation == BRepOrientation::Reversed { seg.reverse(); }
                    }
                    for k in 0..seg.len().saturating_sub(1) { loop_pts.push(seg[k].clone()); }
                }
                if loop_pts.len() < 3 { continue; }
                let loop_crv = NurbsCurve::create(true, 1, &loop_pts);
                if wi == 0 { ts.m_outer_loop = Some(loop_crv); } else { ts.m_inner_loops.push(loop_crv); }
            }
            fmesh[fi] = ts.mesh();
        }

        // A Reversed face has its outward normal opposite to the surface normal: flip winding
        // and stored normals together so shading agrees with the geometry.
        for fi in 0..nf {
            if self.face_orientation(fi) != BRepOrientation::Reversed { continue; }
            fmesh[fi].flip();
            for (_, vd) in fmesh[fi].vertex.iter_mut() {
                if let Some(n) = vd.normal() { vd.set_normal(-n[0], -n[1], -n[2]); }
            }
        }
        fmesh
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Evaluation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Surface point of a face at (u, v).
    pub fn point_at(&self, face_index: usize, u: f64, v: f64) -> Point {
        if face_index >= self.m_faces.len() { return Point::new(0.0, 0.0, 0.0); }
        self.m_surfaces[self.m_faces[face_index].surface_index as usize].point_at(u, v).unwrap_or_else(|| Point::new(0.0, 0.0, 0.0))
    }

    /// Surface normal of a face at (u, v), flipped when the face is Reversed in its shell.
    pub fn normal_at(&self, face_index: usize, u: f64, v: f64) -> Vector {
        if face_index >= self.m_faces.len() { return Vector::new(0.0, 0.0, 0.0); }
        let n = self.m_surfaces[self.m_faces[face_index].surface_index as usize].normal_at(u, v);
        if self.face_orientation(face_index) == BRepOrientation::Reversed { return Vector::new(-n[0], -n[1], -n[2]); }
        n
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Transform surfaces, 3D curves and vertices in place (pcurves are parametric, untouched).
    pub fn transform(&mut self, xf: &Xform) {
        for srf in &mut self.m_surfaces { srf.transform(xf); }
        for crv in &mut self.m_curves_3d { crv.transform(xf); }
        for v in &mut self.m_vertices { v.point = xf.transform_point(&v.point); }
    }

    /// Return a transformed copy.
    pub fn transformed(&self, xf: &Xform) -> Self {
        let mut b = self.duplicate();
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

    fn refs_to_proto(refs: &[BRepRef]) -> Vec<crate::proto::BRepRef> {
        refs.iter().map(|r| crate::proto::BRepRef { index: r.index, orientation: r.orientation as i32 }).collect()
    }

    fn refs_from_proto(refs: &[crate::proto::BRepRef]) -> Vec<BRepRef> {
        refs.iter().map(|r| BRepRef::new(r.index, BRepOrientation::from_i32(r.orientation))).collect()
    }

    /// The proto struct itself — pb_dumps encodes it; Session embeds it directly.
    pub fn to_proto(&self) -> crate::proto::BRep {
        use prost::Message;
        crate::proto::BRep {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            curves_2d: self.m_curves_2d.iter().map(|c| crate::proto::NurbsCurve::decode(c.pb_dumps().as_slice()).unwrap()).collect(),
            curves_3d: self.m_curves_3d.iter().map(|c| crate::proto::NurbsCurve::decode(c.pb_dumps().as_slice()).unwrap()).collect(),
            surfaces: self.m_surfaces.iter().map(|s| crate::proto::NurbsSurface::decode(s.pb_dumps().as_slice()).unwrap()).collect(),
            vertices: self.m_vertices.iter().map(|v| crate::proto::BRepVertex {
                point: Some(crate::proto::Point { guid: String::new(), name: String::new(), x: v.point[0], y: v.point[1], z: v.point[2], width: 0.0, pointcolor: None }),
                tolerance: v.tolerance,
            }).collect(),
            edges: self.m_edges.iter().map(|e| crate::proto::BRepEdge {
                curve_3d_index: e.curve_3d_index,
                start_vertex: e.start_vertex,
                end_vertex: e.end_vertex,
                tolerance: e.tolerance,
                degenerated: e.degenerated,
                pcurves: e.pcurves.iter().map(|pc| crate::proto::BRepCurveOnSurface {
                    surface_index: pc.surface_index, curve_2d_index: pc.curve_2d_index, curve_2d_index_2: pc.curve_2d_index_2,
                }).collect(),
            }).collect(),
            wires: self.m_wires.iter().map(|w| crate::proto::BRepWire { edges: Self::refs_to_proto(&w.edges) }).collect(),
            faces: self.m_faces.iter().map(|f| crate::proto::BRepFace {
                surface_index: f.surface_index,
                wires: Self::refs_to_proto(&f.wires),
                tolerance: f.tolerance,
                facecolor: f.facecolor.as_ref().map(|c| crate::proto::Color { guid: String::new(), name: String::new(), r: c.r, g: c.g, b: c.b, a: c.a }),
            }).collect(),
            shells: self.m_shells.iter().map(|s| crate::proto::BRepShell { faces: Self::refs_to_proto(&s.faces) }).collect(),
            solids: self.m_solids.iter().map(|s| crate::proto::BRepSolid { shells: Self::refs_to_proto(&s.shells) }).collect(),
            width: self.width,
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
        b.width = proto.width;
        for c in &proto.curves_2d { b.m_curves_2d.push(NurbsCurve::pb_loads(&c.encode_to_vec())?); }
        for c in &proto.curves_3d { b.m_curves_3d.push(NurbsCurve::pb_loads(&c.encode_to_vec())?); }
        for s in &proto.surfaces { b.m_surfaces.push(NurbsSurface::pb_loads(&s.encode_to_vec())?); }
        for v in &proto.vertices {
            let p = v.point.as_ref().map(|p| Point::new(p.x, p.y, p.z)).unwrap_or_else(|| Point::new(0.0, 0.0, 0.0));
            b.m_vertices.push(BRepVertex { point: p, tolerance: v.tolerance });
        }
        for e in &proto.edges {
            b.m_edges.push(BRepEdge {
                curve_3d_index: e.curve_3d_index,
                start_vertex: e.start_vertex,
                end_vertex: e.end_vertex,
                tolerance: e.tolerance,
                degenerated: e.degenerated,
                pcurves: e.pcurves.iter().map(|pc| BRepCurveOnSurface {
                    surface_index: pc.surface_index, curve_2d_index: pc.curve_2d_index, curve_2d_index_2: pc.curve_2d_index_2,
                }).collect(),
            });
        }
        for w in &proto.wires { b.m_wires.push(BRepWire { edges: Self::refs_from_proto(&w.edges) }); }
        for f in &proto.faces {
            b.m_faces.push(BRepFace {
                surface_index: f.surface_index,
                wires: Self::refs_from_proto(&f.wires),
                tolerance: f.tolerance,
                facecolor: f.facecolor.as_ref().map(|c| Color::new(c.r, c.g, c.b, c.a)),
            });
        }
        for s in &proto.shells { b.m_shells.push(BRepShell { faces: Self::refs_from_proto(&s.faces) }); }
        for s in &proto.solids { b.m_solids.push(BRepSolid { shells: Self::refs_from_proto(&s.shells) }); }
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

#[derive(Serialize, Deserialize)]
struct RefJson {
    index: i32,
    orientation: String,
}

fn refs_json(refs: &[BRepRef]) -> Vec<RefJson> {
    refs.iter().map(|r| RefJson { index: r.index, orientation: r.orientation.to_str().to_string() }).collect()
}

fn refs_from_json(refs: &[RefJson]) -> Vec<BRepRef> {
    refs.iter().map(|r| BRepRef::new(r.index, BRepOrientation::from_str(&r.orientation))).collect()
}

#[derive(Serialize, Deserialize)]
struct PCurveJson {
    curve_2d_index: i32,
    curve_2d_index_2: i32,
    surface_index: i32,
}

#[derive(Serialize, Deserialize)]
struct EdgeJson {
    curve_3d_index: i32,
    degenerated: bool,
    end_vertex: i32,
    pcurves: Vec<PCurveJson>,
    start_vertex: i32,
    tolerance: f64,
}

#[derive(Serialize, Deserialize)]
struct FaceJson {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(default)]
    facecolor: Option<Color>,
    surface_index: i32,
    tolerance: f64,
    wires: Vec<RefJson>,
}

#[derive(Serialize, Deserialize)]
struct ShellJson {
    faces: Vec<RefJson>,
}

#[derive(Serialize, Deserialize)]
struct SolidJson {
    shells: Vec<RefJson>,
}

#[derive(Serialize, Deserialize)]
struct VertexJson {
    point: [f64; 3],
    tolerance: f64,
}

#[derive(Serialize, Deserialize)]
struct WireJson {
    edges: Vec<RefJson>,
}

impl Serialize for BRep {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where S: Serializer {
        let mut map = serializer.serialize_map(None)?;
        map.serialize_entry("curves_2d", &self.m_curves_2d)?;
        map.serialize_entry("curves_3d", &self.m_curves_3d)?;
        let edges: Vec<EdgeJson> = self.m_edges.iter().map(|e| EdgeJson {
            curve_3d_index: e.curve_3d_index,
            degenerated: e.degenerated,
            end_vertex: e.end_vertex,
            pcurves: e.pcurves.iter().map(|pc| PCurveJson { curve_2d_index: pc.curve_2d_index, curve_2d_index_2: pc.curve_2d_index_2, surface_index: pc.surface_index }).collect(),
            start_vertex: e.start_vertex,
            tolerance: e.tolerance,
        }).collect();
        map.serialize_entry("edges", &edges)?;
        let faces: Vec<FaceJson> = self.m_faces.iter().map(|f| FaceJson {
            facecolor: f.facecolor.clone(), surface_index: f.surface_index, tolerance: f.tolerance, wires: refs_json(&f.wires),
        }).collect();
        map.serialize_entry("faces", &faces)?;
        map.serialize_entry("guid", &self.guid())?;
        map.serialize_entry("name", &self.name)?;
        let shells: Vec<ShellJson> = self.m_shells.iter().map(|s| ShellJson { faces: refs_json(&s.faces) }).collect();
        map.serialize_entry("shells", &shells)?;
        let solids: Vec<SolidJson> = self.m_solids.iter().map(|s| SolidJson { shells: refs_json(&s.shells) }).collect();
        map.serialize_entry("solids", &solids)?;
        map.serialize_entry("surfacecolor", &self.surfacecolor)?;
        map.serialize_entry("surfaces", &self.m_surfaces)?;
        map.serialize_entry("type", "BRep")?;
        let vertices: Vec<VertexJson> = self.m_vertices.iter().map(|v| VertexJson { point: [v.point[0], v.point[1], v.point[2]], tolerance: v.tolerance }).collect();
        map.serialize_entry("vertices", &vertices)?;
        map.serialize_entry("width", &self.width)?;
        let wires: Vec<WireJson> = self.m_wires.iter().map(|w| WireJson { edges: refs_json(&w.edges) }).collect();
        map.serialize_entry("wires", &wires)?;
        map.end()
    }
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
            curves_2d: Vec<NurbsCurve>,
            #[serde(default)]
            curves_3d: Vec<NurbsCurve>,
            #[serde(default)]
            surfaces: Vec<NurbsSurface>,
            #[serde(default)]
            vertices: Vec<VertexJson>,
            #[serde(default)]
            edges: Vec<EdgeJson>,
            #[serde(default)]
            wires: Vec<WireJson>,
            #[serde(default)]
            faces: Vec<FaceJson>,
            #[serde(default)]
            shells: Vec<ShellJson>,
            #[serde(default)]
            solids: Vec<SolidJson>,
        }

        let data = BRepData::deserialize(deserializer)?;
        let mut b = BRep::new();
        if let Some(g) = data.guid { b.set_guid(g); }
        if let Some(n) = data.name { b.name = n; }
        if let Some(w) = data.width { b.width = w; }
        if let Some(c) = data.surfacecolor { b.surfacecolor = c; }
        b.m_curves_2d = data.curves_2d;
        b.m_curves_3d = data.curves_3d;
        b.m_surfaces = data.surfaces;
        b.m_vertices = data.vertices.iter().map(|v| BRepVertex { point: Point::new(v.point[0], v.point[1], v.point[2]), tolerance: v.tolerance }).collect();
        b.m_edges = data.edges.iter().map(|e| BRepEdge {
            curve_3d_index: e.curve_3d_index,
            start_vertex: e.start_vertex,
            end_vertex: e.end_vertex,
            tolerance: e.tolerance,
            degenerated: e.degenerated,
            pcurves: e.pcurves.iter().map(|pc| BRepCurveOnSurface { surface_index: pc.surface_index, curve_2d_index: pc.curve_2d_index, curve_2d_index_2: pc.curve_2d_index_2 }).collect(),
        }).collect();
        b.m_wires = data.wires.iter().map(|w| BRepWire { edges: refs_from_json(&w.edges) }).collect();
        b.m_faces = data.faces.into_iter().map(|f| BRepFace { surface_index: f.surface_index, wires: refs_from_json(&f.wires), tolerance: f.tolerance, facecolor: f.facecolor }).collect();
        b.m_shells = data.shells.iter().map(|s| BRepShell { faces: refs_from_json(&s.faces) }).collect();
        b.m_solids = data.solids.iter().map(|s| BRepSolid { shells: refs_from_json(&s.shells) }).collect();
        Ok(b)
    }
}

impl std::fmt::Display for BRep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.str())
    }
}
