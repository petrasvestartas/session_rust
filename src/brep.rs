use crate::color::Color;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::nurbssurface_trimmed::NurbsSurfaceTrimmed;
use crate::nurbssurface_trimmed::TrimLoops;
use crate::plane::Plane;
use crate::point::Point;
use crate::polyline::Polyline;
use crate::primitives::Primitives;
use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
use crate::tolerance::{Tolerance, PI};
use crate::vector::Vector;
use crate::xform::Xform;
use serde::ser::SerializeMap;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::collections::BTreeMap;
use std::collections::HashMap;

// ═══════════════════════════════════════════════════════════════════════════
// Orientation
// ═══════════════════════════════════════════════════════════════════════════

/// TopAbs_Orientation: carried by the parent -> child reference, never by the shape
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BRepOrientation {
    Forward = 0,
    Reversed = 1,
    Internal = 2,
    External = 3,
}

/// TopAbs::Reverse
pub fn brep_reverse(o: BRepOrientation) -> BRepOrientation {
    if o == BRepOrientation::Forward {
        return BRepOrientation::Reversed;
    }
    if o == BRepOrientation::Reversed {
        return BRepOrientation::Forward;
    }
    o
}

/// TopAbs::Compose: the orientation of a sub-shape reached through a parent with orientation `a`
pub fn brep_compose(a: BRepOrientation, b: BRepOrientation) -> BRepOrientation {
    if a == BRepOrientation::Internal || a == BRepOrientation::External {
        return a;
    }
    if a == BRepOrientation::Forward {
        return b;
    }
    brep_reverse(b)
}

const F: BRepOrientation = BRepOrientation::Forward;
const R: BRepOrientation = BRepOrientation::Reversed;

fn orientation_to_str(o: BRepOrientation) -> &'static str {
    if o == BRepOrientation::Reversed {
        return "reversed";
    }
    if o == BRepOrientation::Internal {
        return "internal";
    }
    if o == BRepOrientation::External {
        return "external";
    }
    "forward"
}

fn orientation_from_str(s: &str) -> BRepOrientation {
    if s == "reversed" {
        return BRepOrientation::Reversed;
    }
    if s == "internal" {
        return BRepOrientation::Internal;
    }
    if s == "external" {
        return BRepOrientation::External;
    }
    BRepOrientation::Forward
}

fn orientation_from_i32(v: i32) -> BRepOrientation {
    if v == 1 {
        return BRepOrientation::Reversed;
    }
    if v == 2 {
        return BRepOrientation::Internal;
    }
    if v == 3 {
        return BRepOrientation::External;
    }
    BRepOrientation::Forward
}

// ═══════════════════════════════════════════════════════════════════════════
// Shapes
// ═══════════════════════════════════════════════════════════════════════════

/// TopoDS_Shape: an oriented reference to a sub-shape (index into the owning table)
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

/// BRep_TVertex
#[derive(Debug, Clone, PartialEq)]
pub struct BRepVertex {
    pub point: Point,
    pub tolerance: f64,
}

/// BRep_CurveOnSurface: curve_2d_index_2 is the pcurve of the REVERSED use on a closed surface (seam), -1 otherwise; pcurves run in the edge's own direction
#[derive(Debug, Clone, PartialEq)]
pub struct BRepCurveOnSurface {
    pub surface_index: i32,
    pub curve_2d_index: i32,
    pub curve_2d_index_2: i32,
}

/// BRep_TEdge: curve_3d_index is -1 for a degenerated edge (sphere pole, cone apex)
#[derive(Debug, Clone, PartialEq)]
pub struct BRepEdge {
    pub curve_3d_index: i32,
    pub start_vertex: i32,
    pub end_vertex: i32,
    pub tolerance: f64,
    pub degenerated: bool,
    pub pcurves: Vec<BRepCurveOnSurface>,
}

/// TopoDS_TWire
#[derive(Debug, Clone, PartialEq)]
pub struct BRepWire {
    pub edges: Vec<BRepRef>,
}

/// BRep_TFace: the first wire is the outer boundary; facecolor None means unset
#[derive(Debug, Clone, PartialEq)]
pub struct BRepFace {
    pub surface_index: i32,
    pub wires: Vec<BRepRef>,
    pub tolerance: f64,
    pub facecolor: Option<Color>,
}

/// TopoDS_TShell
#[derive(Debug, Clone, PartialEq)]
pub struct BRepShell {
    pub faces: Vec<BRepRef>,
}

/// TopoDS_TSolid
#[derive(Debug, Clone, PartialEq)]
pub struct BRepSolid {
    pub shells: Vec<BRepRef>,
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Bilinear planar patch: u runs p00 -> p10, v runs p00 -> p01, natural normal = u x v
fn bilinear_patch(p00: &Point, p10: &Point, p01: &Point, p11: &Point) -> NurbsSurface {
    let mut srf = NurbsSurface::new(3, false, 2, 2, 2, 2);
    srf.set_cv(0, 0, p00);
    srf.set_cv(1, 0, p10);
    srf.set_cv(0, 1, p01);
    srf.set_cv(1, 1, p11);
    srf
}

/// Straight pcurve from (u0, v0) to (u1, v1)
fn uv_line(u0: f64, v0: f64, u1: f64, v1: f64) -> NurbsCurve {
    NurbsCurve::create(
        false,
        1,
        &[Point::new(u0, v0, 0.0), Point::new(u1, v1, 0.0)],
    )
}

/// Exact pcurve of a 3D curve lying on a bilinear planar patch: the affine image of its CVs
fn project_to_patch(crv: &NurbsCurve, srf: &NurbsSurface) -> NurbsCurve {
    let p00 = srf.get_cv(0, 0).unwrap_or_default();
    let p10 = srf.get_cv(1, 0).unwrap_or_default();
    let p01 = srf.get_cv(0, 1).unwrap_or_default();
    let eu = [p10[0] - p00[0], p10[1] - p00[1], p10[2] - p00[2]];
    let ev = [p01[0] - p00[0], p01[1] - p00[1], p01[2] - p00[2]];
    let eu2 = eu[0] * eu[0] + eu[1] * eu[1] + eu[2] * eu[2];
    let ev2 = ev[0] * ev[0] + ev[1] * ev[1] + ev[2] * ev[2];
    let mut c2 = NurbsCurve::new(3, crv.is_rational(), crv.order(), crv.cv_count());
    for i in 0..crv.nurbsknot_count() {
        c2.set_nurbsknot(i, crv.nurbsknot(i).unwrap_or(0.0));
    }
    for i in 0..crv.cv_count() {
        let (wx, wy, wz, w) = crv.get_cv_4d(i).unwrap_or((0.0, 0.0, 0.0, 1.0));
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

/// Signed area of a closed pcurve's sampled polygon (positive = counter-clockwise)
fn uv_signed_area(c2d: &NurbsCurve) -> f64 {
    let pts = c2d.divide_by_count((c2d.cv_count() * 4).max(16), true).0;
    let mut a = 0.0;
    for i in 0..pts.len().saturating_sub(1) {
        a += pts[i][0] * pts[i + 1][1] - pts[i + 1][0] * pts[i][1];
    }
    0.5 * a
}

/// Signed area of a closed UV polygon (positive = counter-clockwise)
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

/// Diagonal of the control point bounding box
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
    hi.distance(&lo, None)
}

// ═══════════════════════════════════════════════════════════════════════════
// Factory helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Planar polygon faces from a vertex table: edges run lo -> hi vertex and are shared, a face lists its vertices counter-clockwise seen from outside so the patch normal points outward
struct PolyFaceBuilder {
    edge_map: HashMap<(usize, usize), usize>,
}

impl PolyFaceBuilder {
    fn new() -> Self {
        PolyFaceBuilder {
            edge_map: HashMap::new(),
        }
    }

    fn edge(&mut self, b: &mut BRep, v0: usize, v1: usize) -> usize {
        let lo = v0.min(v1);
        let hi = v0.max(v1);
        if let Some(&ei) = self.edge_map.get(&(lo, hi)) {
            return ei;
        }
        let line = NurbsCurve::create(
            false,
            1,
            &[
                b.m_vertices[lo].point.clone(),
                b.m_vertices[hi].point.clone(),
            ],
        );
        let ci = b.add_curve_3d(&line);
        let ei = b.add_edge(ci as i32, lo as i32, hi as i32);
        self.edge_map.insert((lo, hi), ei);
        ei
    }

    /// Oriented edge references of the vertex cycle `vi`, each with its pcurve on surface `si`
    fn wire_refs(&mut self, b: &mut BRep, si: usize, vi: &[usize]) -> Vec<BRepRef> {
        let n = vi.len();
        let mut refs = Vec::new();
        for i in 0..n {
            let va = vi[i];
            let vb = vi[(i + 1) % n];
            let ei = self.edge(b, va, vb);
            let c2d = project_to_patch(
                &b.m_curves_3d[b.m_edges[ei].curve_3d_index as usize],
                &b.m_surfaces[si],
            );
            let ci = b.add_curve_2d(&c2d);
            b.add_pcurve(ei, si, ci as i32, -1);
            let o = if b.m_edges[ei].start_vertex == va as i32 {
                F
            } else {
                R
            };
            refs.push(BRepRef::new(ei as i32, o));
        }
        refs
    }

    /// Face on `srf` bounded by the vertex cycle `vi`; returns the face index
    fn face(&mut self, b: &mut BRep, srf: &NurbsSurface, vi: &[usize]) -> usize {
        self.face_with_holes(b, srf, vi, &[])
    }

    /// Face on `srf` bounded by the vertex cycle `vi`, with one inner wire per hole cycle; returns the face index
    fn face_with_holes(&mut self, b: &mut BRep, srf: &NurbsSurface, vi: &[usize], holes: &[Vec<usize>]) -> usize {
        let si = b.add_surface(srf);
        let refs = self.wire_refs(b, si, vi);
        let mut wires = vec![BRepRef::new(b.add_wire(&refs) as i32, F)];
        for hole in holes {
            let hole_refs = self.wire_refs(b, si, hole);
            wires.push(BRepRef::new(b.add_wire(&hole_refs) as i32, F));
        }
        b.add_face(si as i32, &wires, 0.0)
    }
}

const BOX_FACES: [[usize; 4]; 6] = [
    [0, 3, 2, 1],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [1, 2, 6, 5],
    [2, 3, 7, 6],
    [3, 0, 4, 7],
];

/// Bilinear patch spanned by four vertex indices in face order (p00, p10, p11, p01)
fn quad_patch(b: &BRep, fv: &[usize; 4]) -> NurbsSurface {
    bilinear_patch(
        &b.m_vertices[fv[0]].point,
        &b.m_vertices[fv[1]].point,
        &b.m_vertices[fv[3]].point,
        &b.m_vertices[fv[2]].point,
    )
}

fn box_corners(b: &mut BRep, sx: f64, sy: f64, sz: f64) {
    let hx = sx * 0.5;
    let hy = sy * 0.5;
    let hz = sz * 0.5;
    b.add_vertex(&Point::new(-hx, -hy, -hz), 0.0);
    b.add_vertex(&Point::new(hx, -hy, -hz), 0.0);
    b.add_vertex(&Point::new(hx, hy, -hz), 0.0);
    b.add_vertex(&Point::new(-hx, hy, -hz), 0.0);
    b.add_vertex(&Point::new(-hx, -hy, hz), 0.0);
    b.add_vertex(&Point::new(hx, -hy, hz), 0.0);
    b.add_vertex(&Point::new(hx, hy, hz), 0.0);
    b.add_vertex(&Point::new(-hx, hy, hz), 0.0);
}

/// Planar cap at height z with natural normal +Z (up) or -Z (down), spanning [-r, r]^2
fn cap_patch(r: f64, z: f64, up: bool) -> NurbsSurface {
    if up {
        return bilinear_patch(
            &Point::new(-r, -r, z),
            &Point::new(r, -r, z),
            &Point::new(-r, r, z),
            &Point::new(r, r, z),
        );
    }
    bilinear_patch(
        &Point::new(-r, -r, z),
        &Point::new(-r, r, z),
        &Point::new(r, -r, z),
        &Point::new(r, r, z),
    )
}

/// Cap face bounded by one closed edge: outer wire counter-clockwise in the patch's UV
fn cap_face(b: &mut BRep, cap: &NurbsSurface, edge: usize) -> usize {
    let si = b.add_surface(cap);
    let c2d = project_to_patch(&b.m_curves_3d[b.m_edges[edge].curve_3d_index as usize], cap);
    let o = if uv_signed_area(&c2d) > 0.0 { F } else { R };
    let ci = b.add_curve_2d(&c2d);
    b.add_pcurve(edge, si, ci as i32, -1);
    let wi = b.add_wire(&[BRepRef::new(edge as i32, o)]);
    b.add_face(si as i32, &[BRepRef::new(wi as i32, F)], 0.0)
}

/// Periodic body face (cylinder / cone / bore): seam from v0 to v1 at u0 == u1, bottom ring forward at v0, top ring (or degenerated apex) reversed at v1
fn body_face(b: &mut BRep, si: usize, e_bot: usize, e_seam: usize, e_top: usize) -> usize {
    let (u0, u1) = b.m_surfaces[si].domain(0).unwrap_or((0.0, 1.0));
    let (v0, v1) = b.m_surfaces[si].domain(1).unwrap_or((0.0, 1.0));
    let c_bot = b.add_curve_2d(&uv_line(u0, v0, u1, v0));
    b.add_pcurve(e_bot, si, c_bot as i32, -1);
    let c_top = b.add_curve_2d(&uv_line(u0, v1, u1, v1));
    b.add_pcurve(e_top, si, c_top as i32, -1);
    let c_right = b.add_curve_2d(&uv_line(u1, v0, u1, v1));
    let c_left = b.add_curve_2d(&uv_line(u0, v0, u0, v1));
    b.add_pcurve(e_seam, si, c_right as i32, c_left as i32);
    let wi = b.add_wire(&[
        BRepRef::new(e_bot as i32, F),
        BRepRef::new(e_seam as i32, F),
        BRepRef::new(e_top as i32, R),
        BRepRef::new(e_seam as i32, R),
    ]);
    b.add_face(si as i32, &[BRepRef::new(wi as i32, F)], 0.0)
}

/// Point of the plane (org, xa, ya) at (u, v)
fn plane_point(org: &Point, xa: &Vector, ya: &Vector, u: f64, v: f64) -> Point {
    Point::new(
        org[0] + u * xa[0] + v * ya[0],
        org[1] + u * xa[1] + v * ya[1],
        org[2] + u * xa[2] + v * ya[2],
    )
}

/// Padded bilinear patch through `pts` in the plane (org, xa, ya)
fn planar_patch_through(pts: &[Point], org: &Point, xa: &Vector, ya: &Vector) -> NurbsSurface {
    let mut umin: f64 = 1e30;
    let mut umax: f64 = -1e30;
    let mut vmin: f64 = 1e30;
    let mut vmax: f64 = -1e30;
    for p in pts {
        let dx = p[0] - org[0];
        let dy = p[1] - org[1];
        let dz = p[2] - org[2];
        let u = dx * xa[0] + dy * xa[1] + dz * xa[2];
        let v = dx * ya[0] + dy * ya[1] + dz * ya[2];
        umin = umin.min(u);
        umax = umax.max(u);
        vmin = vmin.min(v);
        vmax = vmax.max(v);
    }
    let pad = (umax - umin).max(vmax - vmin) * 0.01;
    umin -= pad;
    umax += pad;
    vmin -= pad;
    vmax += pad;
    bilinear_patch(
        &plane_point(org, xa, ya, umin, vmin),
        &plane_point(org, xa, ya, umax, vmin),
        &plane_point(org, xa, ya, umin, vmax),
        &plane_point(org, xa, ya, umax, vmax),
    )
}

/// Signed area of a closed cycle of points seen in the plane (org, xa, ya): positive when it runs counter-clockwise
fn signed_area_in_plane(pts: &[Point], org: &Point, xa: &Vector, ya: &Vector) -> f64 {
    let mut area = 0.0;
    let n = pts.len();
    for i in 0..n {
        let a = &pts[i];
        let b = &pts[(i + 1) % n];
        let au = (a[0] - org[0]) * xa[0] + (a[1] - org[1]) * xa[1] + (a[2] - org[2]) * xa[2];
        let av = (a[0] - org[0]) * ya[0] + (a[1] - org[1]) * ya[1] + (a[2] - org[2]) * ya[2];
        let bu = (b[0] - org[0]) * xa[0] + (b[1] - org[1]) * xa[1] + (b[2] - org[2]) * xa[2];
        let bv = (b[0] - org[0]) * ya[0] + (b[1] - org[1]) * ya[1] + (b[2] - org[2]) * ya[2];
        area += au * bv - bu * av;
    }
    area * 0.5
}

/// The vertices of a polyline without the closing duplicate
fn open_points(pl: &Polyline) -> Vec<Point> {
    let pts = pl.get_points();
    let n = if pl.is_closed() { pts.len().saturating_sub(1) } else { pts.len() };
    pts[..n].to_vec()
}

fn find_or_add_vertex(b: &mut BRep, p: &Point, tol: f64) -> usize {
    for i in 0..b.m_vertices.len() {
        if b.m_vertices[i].point.distance(p, None) < tol {
            return i;
        }
    }
    b.add_vertex(p, 0.0)
}

fn cv_points(c: &NurbsCurve) -> Vec<Point> {
    let mut pts = Vec::new();
    for k in 0..c.cv_count() {
        let (wx, wy, wz, w) = c.get_cv_4d(k).unwrap_or((0.0, 0.0, 0.0, 0.0));
        if w != 0.0 {
            pts.push(Point::new(wx / w, wy / w, wz / w));
        }
    }
    pts
}

/// One-edge wire of a closed or open curve on planar surface `si`, sharing vertices within `tol`
fn curve_wire(b: &mut BRep, crv: &NurbsCurve, si: usize, tol: f64) -> usize {
    let sp = crv.point_at(crv.domain().0);
    let ep = crv.point_at(crv.domain().1);
    let vs = find_or_add_vertex(b, &sp, tol);
    let ve = if crv.is_closed() {
        vs
    } else {
        find_or_add_vertex(b, &ep, tol)
    };
    let ci = b.add_curve_3d(crv);
    let ei = b.add_edge(ci as i32, vs as i32, ve as i32);
    let c2 = b.add_curve_2d(&project_to_patch(crv, &b.m_surfaces[si]));
    b.add_pcurve(ei, si, c2 as i32, -1);
    b.add_wire(&[BRepRef::new(ei as i32, F)])
}

// ═══════════════════════════════════════════════════════════════════════════
// Sewing helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Signed volume of face meshes (positive when the windings point outward)
fn signed_volume(meshes: &[Mesh]) -> f64 {
    let mut total = 0.0;
    for fm in meshes {
        for fverts in fm.face.values() {
            for k in 1..fverts.len().saturating_sub(1) {
                let a = fm.vertex[&fverts[0]].position();
                let b = fm.vertex[&fverts[k]].position();
                let c = fm.vertex[&fverts[k + 1]].position();
                total += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                    + a[2] * (b[0] * c[1] - b[1] * c[0]);
            }
        }
    }
    total / 6.0
}

/// Face uses of every edge as (face, composed orientation); empty when some edge is not used exactly twice
fn edge_uses(b: &BRep) -> Vec<Vec<(usize, BRepOrientation)>> {
    let mut uses: Vec<Vec<(usize, BRepOrientation)>> = vec![Vec::new(); b.m_edges.len()];
    for fi in 0..b.face_count() {
        for wr in &b.m_faces[fi].wires {
            for er in b.wire_edges(wr) {
                uses[er.index as usize].push((fi, er.orientation));
            }
        }
    }
    for u in &uses {
        if u.len() != 2 {
            return Vec::new();
        }
    }
    uses
}

/// Connected components of faces, each face oriented consistently with the neighbour it was reached from
fn face_components(
    b: &BRep,
    uses: &[Vec<(usize, BRepOrientation)>],
    fo: &mut [BRepOrientation],
) -> Vec<Vec<usize>> {
    let nf = b.face_count();
    let mut seen = vec![false; nf];
    let mut components: Vec<Vec<usize>> = Vec::new();
    for seed in 0..nf {
        if seen[seed] {
            continue;
        }
        let mut comp = Vec::new();
        let mut stack = vec![seed];
        seen[seed] = true;
        while let Some(fi) = stack.pop() {
            comp.push(fi);
            for wr in &b.m_faces[fi].wires {
                for er in b.wire_edges(wr) {
                    for &(g, og) in &uses[er.index as usize] {
                        if g == fi || seen[g] {
                            continue;
                        }
                        fo[g] = if og == er.orientation {
                            brep_reverse(fo[fi])
                        } else {
                            fo[fi]
                        };
                        seen[g] = true;
                        stack.push(g);
                    }
                }
            }
        }
        components.push(comp);
    }
    components
}

/// BRepBuilderAPI_Sewing + MakeSolid for free faces: when every edge is shared by exactly two face uses, one shell per connected component wound outward and one solid per shell
fn close_free_faces(b: &mut BRep) {
    let nf = b.face_count();
    if nf == 0 {
        return;
    }
    let uses = edge_uses(b);
    if uses.is_empty() {
        return;
    }
    let mut fo = vec![F; nf];
    let mut shells = Vec::new();
    for comp in face_components(b, &uses, &mut fo) {
        let mut refs = Vec::new();
        for fi in comp {
            refs.push(BRepRef::new(fi as i32, fo[fi]));
        }
        shells.push(BRepRef::new(b.add_shell(&refs) as i32, F));
    }
    let fm = b.face_meshes();
    for sr in shells {
        let mut part = Vec::new();
        for fr in &b.m_shells[sr.index as usize].faces {
            part.push(fm[fr.index as usize].clone());
        }
        if signed_volume(&part) < 0.0 {
            for fr in &mut b.m_shells[sr.index as usize].faces {
                fr.orientation = brep_reverse(fr.orientation);
            }
        }
        b.add_solid(&[sr]);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Planar face helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Samples per curved edge of a planar face (a rounded corner, a circular boss)
const CURVED_EDGE_SAMPLES: usize = 16;

/// Open outline of a face's outer wire in wire order: vertices of straight edges, samples of curved ones
fn face_outline(b: &BRep, fi: usize) -> Vec<Point> {
    let mut points: Vec<Point> = Vec::new();
    for er in b.wire_edges(&b.m_faces[fi].wires[0]) {
        if er.index < 0 || er.index as usize >= b.m_edges.len() {
            continue;
        }
        let edge = &b.m_edges[er.index as usize];
        if edge.degenerated {
            continue;
        }
        let reversed = er.orientation == BRepOrientation::Reversed;
        let curved =
            edge.curve_3d_index >= 0 && b.m_curves_3d[edge.curve_3d_index as usize].degree() > 1;
        if curved {
            let c = &b.m_curves_3d[edge.curve_3d_index as usize];
            let (d0, d1) = c.domain();
            for s in 0..CURVED_EDGE_SAMPLES {
                let u = s as f64 / CURVED_EDGE_SAMPLES as f64;
                let t = if reversed {
                    d1 + (d0 - d1) * u
                } else {
                    d0 + (d1 - d0) * u
                };
                points.push(c.point_at(t));
            }
        } else {
            let start = if reversed {
                edge.end_vertex
            } else {
                edge.start_vertex
            };
            if start >= 0 && (start as usize) < b.m_vertices.len() {
                points.push(b.m_vertices[start as usize].point.clone());
            }
        }
    }
    points
}

/// Signed volume enclosed by closed outlines (tetrahedra fans from the origin), positive when wound outward
fn outline_volume(polylines: &[Polyline]) -> f64 {
    let mut total = 0.0;
    for pl in polylines {
        let pts = pl.get_points();
        if pts.len() < 3 {
            continue;
        }
        let p0 = &pts[0];
        for k in 1..pts.len() - 1 {
            let p1 = &pts[k];
            let p2 = &pts[k + 1];
            total += p0[0] * (p1[1] * p2[2] - p1[2] * p2[1])
                + p0[1] * (p1[2] * p2[0] - p1[0] * p2[2])
                + p0[2] * (p1[0] * p2[1] - p1[1] * p2[0]);
        }
    }
    total / 6.0
}

/// Outer polyline and outward plane of every planar face in one walk, so the two stay index-aligned
fn planar_faces(b: &BRep) -> (Vec<Polyline>, Vec<Plane>) {
    let mut polylines = Vec::new();
    let mut planes = Vec::new();
    for fi in 0..b.face_count() {
        let face = &b.m_faces[fi];
        if face.surface_index < 0 || face.wires.is_empty() {
            continue;
        }
        if !b.m_surfaces[face.surface_index as usize].is_planar(None, Tolerance::ZERO_TOLERANCE) {
            continue;
        }
        let mut points = face_outline(b, fi);
        if points.len() < 3 {
            continue;
        }
        let origin = Point::centroid(&points);
        let mut normal = Vector::average_normal(&points);
        if b.face_orientation(fi) == BRepOrientation::Reversed {
            normal.reverse();
        }
        points.push(points[0].clone());
        polylines.push(Polyline::new(points));
        planes.push(Plane::from_point_normal(origin, normal, None));
    }
    if b.is_solid() && outline_volume(&polylines) < 0.0 {
        for pl in planes.iter_mut() {
            let mut n = pl.z_axis();
            n.reverse();
            *pl = Plane::from_point_normal(pl.origin(), n, None);
        }
    }
    (polylines, planes)
}

// ═══════════════════════════════════════════════════════════════════════════
// Meshing helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Boundary sample: pcurve parameter, uv point, model point
type Sample = (f64, Point, Point);

/// Edge use of a trim loop: edge index, loop index, first sample index in the loop, sample count
type EdgeUse = (usize, usize, usize, usize);

/// Canonical boundary of every shared edge: model points, the (face, pcurve, parameters) that produced them, and refined (t, uv) samples
#[derive(Default)]
struct EdgeBoundary {
    points: HashMap<usize, Vec<Point>>,
    basis: BTreeMap<usize, (usize, usize, Vec<f64>)>,
    samples: HashMap<usize, Vec<(f64, Point)>>,
}

/// UV polygon of one wire of a face (pcurves sampled in traversal order)
fn wire_uv_points(b: &BRep, face_index: usize, wire: &BRepRef) -> Vec<Point> {
    let mut pts = Vec::new();
    for er in b.wire_edges(wire) {
        let ci = b.pcurve_index(er.index as usize, face_index, er.orientation);
        if ci < 0 {
            continue;
        }
        let crv = &b.m_curves_2d[ci as usize];
        let mut seg: Vec<Point> = Vec::new();
        if crv.degree() <= 1 && !crv.is_rational() {
            for k in 0..crv.cv_count() {
                seg.push(crv.get_cv(k).unwrap_or_default());
            }
        } else {
            seg = crv.divide_by_count((crv.cv_count() * 4).max(16), true).0;
        }
        if er.orientation == BRepOrientation::Reversed {
            seg.reverse();
        }
        for uv in seg.iter().take(seg.len().saturating_sub(1)) {
            pts.push(uv.clone());
        }
    }
    pts
}

/// Distance from `point` to the surface point the pcurve reaches at t
fn lifted_distance(surface: &NurbsSurface, curve: &NurbsCurve, point: &Point, t: f64) -> f64 {
    let uv = curve.point_at(t);
    surface
        .point_at(uv[0], uv[1])
        .map_or(f64::INFINITY, |p| p.distance(point, None))
}

/// Parameter of the lifted pcurve closest to `point`: a coarse scan then 64 golden-section steps in the best cell
fn boundary_parameter(surface: &NurbsSurface, curve: &NurbsCurve, point: &Point) -> f64 {
    let (start, end) = curve.domain();
    let count = (curve.cv_count() * 4).clamp(32, 4096);
    let step = (end - start) / count as f64;
    let mut best = start;
    let mut error = lifted_distance(surface, curve, point, start);
    for index in 1..=count {
        let t = if index == count {
            end
        } else {
            start + index as f64 * step
        };
        let candidate = lifted_distance(surface, curve, point, t);
        if candidate < error {
            best = t;
            error = candidate;
        }
    }
    let mut left = (best - step).max(start);
    let mut right = (best + step).min(end);
    let ratio = (5.0f64.sqrt() - 1.0) * 0.5;
    let mut a = right - ratio * (right - left);
    let mut b = left + ratio * (right - left);
    let mut da = lifted_distance(surface, curve, point, a);
    let mut db = lifted_distance(surface, curve, point, b);
    for _ in 0..64 {
        if da < db {
            right = b;
            b = a;
            db = da;
            a = right - ratio * (right - left);
            da = lifted_distance(surface, curve, point, a);
        } else {
            left = a;
            a = b;
            da = db;
            b = left + ratio * (right - left);
            db = lifted_distance(surface, curve, point, b);
        }
    }
    if da < error {
        best = a;
        error = da;
    }
    if db < error {
        best = b;
    }
    best
}

/// Unit normal on a boundary, taking the one-sided limit toward `toward` at a singular endpoint
fn boundary_normal(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    t: f64,
    toward: f64,
) -> Option<Vector> {
    for at in [t, t + (toward - t) * 1e-6] {
        let uv = curve.point_at(at);
        let derivatives = surface.evaluate(uv[0], uv[1], 1);
        if derivatives.len() < 3 {
            continue;
        }
        let mut n = derivatives[1].cross(&derivatives[2]);
        let scale = n[0].abs().max(n[1].abs()).max(n[2].abs());
        if !scale.is_finite() || scale == 0.0 {
            continue;
        }
        n /= scale;
        let length = n.magnitude();
        if length.is_finite() && length > 0.0 {
            return Some(n / length);
        }
    }
    None
}

/// True when the normals at ta, t and tb turn more than the angle whose cosine is given
fn boundary_turns(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    ta: f64,
    t: f64,
    tb: f64,
    cosine: f64,
) -> bool {
    let normals = [
        boundary_normal(surface, curve, ta, tb),
        boundary_normal(surface, curve, t, ta),
        boundary_normal(surface, curve, tb, ta),
    ];
    for j in 0..3 {
        for k in j + 1..3 {
            let (Some(n), Some(m)) = (&normals[j], &normals[k]) else {
                continue;
            };
            if n[0] * m[0] + n[1] * m[1] + n[2] * m[2] < cosine {
                return true;
            }
        }
    }
    false
}

/// Refine samples of a lifted pcurve until chord and angle hold; existing samples stay exact, eight split levels and 4096 added points per edge bound the work
fn refine_surface_boundary(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    samples: &[Sample],
    angle: f64,
    chord: f64,
) -> Vec<Sample> {
    if samples.len() < 2 {
        return samples.to_vec();
    }
    let tolerance = bbox_diagonal(surface) * chord;
    let cosine = (angle.clamp(0.1, 179.0) * PI / 180.0).cos();
    let mut result = Vec::new();
    let mut added = 0;
    for i in 1..samples.len() {
        let mut stack = vec![(samples[i - 1].clone(), samples[i].clone(), 0)];
        while let Some((a, b, depth)) = stack.pop() {
            let t = (a.0 + b.0) * 0.5;
            let uv = curve.point_at(t);
            let Some(point) = surface.point_at(uv[0], uv[1]) else {
                result.push(a);
                continue;
            };
            let pa = &a.2;
            let pb = &b.2;
            let center = Point::new(
                (pa[0] + pb[0]) * 0.5,
                (pa[1] + pb[1]) * 0.5,
                (pa[2] + pb[2]) * 0.5,
            );
            let split = (point.distance(&center, None) > tolerance
                || boundary_turns(surface, curve, a.0, t, b.0, cosine))
                && depth < 8
                && added < 4096;
            if !split {
                result.push(a);
                continue;
            }
            added += 1;
            let middle = (t, uv, point);
            stack.push((middle.clone(), b, depth + 1));
            stack.push((a, middle, depth + 1));
        }
    }
    result.push(samples[samples.len() - 1].clone());
    result
}

/// Compare canonical boundary positions exactly, without tolerance
fn same_boundary_point(a: &Point, b: &Point) -> bool {
    a[0] == b[0] && a[1] == b[1] && a[2] == b[2]
}

/// Phase 1: the outer wire is the full UV rectangle (straight pcurves enclosing the whole domain, no holes), so the face meshes directly on the surface grid
fn direct_face(b: &BRep, fi: usize) -> bool {
    let face = &b.m_faces[fi];
    if face.wires.len() != 1 {
        return false;
    }
    for er in b.wire_edges(&face.wires[0]) {
        let ci = b.pcurve_index(er.index as usize, fi, er.orientation);
        if ci < 0 {
            continue;
        }
        if b.m_curves_2d[ci as usize].degree() > 1 || b.m_curves_2d[ci as usize].is_rational() {
            return false;
        }
    }
    let outer = wire_uv_points(b, fi, &face.wires[0]);
    if outer.len() < 3 {
        return false;
    }
    let srf = &b.m_surfaces[face.surface_index as usize];
    let (u0, u1) = srf.domain(0).unwrap_or((0.0, 1.0));
    let (v0, v1) = srf.domain(1).unwrap_or((0.0, 1.0));
    for er in b.wire_edges(&face.wires[0]) {
        let ci = b.pcurve_index(er.index as usize, fi, er.orientation);
        if ci < 0 {
            continue;
        }
        let curve = &b.m_curves_2d[ci as usize];
        for k in [0, curve.cv_count().saturating_sub(1)] {
            let p = curve.get_cv(k).unwrap_or_default();
            let corner_u = (p[0] - u0).abs().min((p[0] - u1).abs()) <= (u1 - u0) * 1e-9;
            let corner_v = (p[1] - v0).abs().min((p[1] - v1).abs()) <= (v1 - v0) * 1e-9;
            if !corner_u || !corner_v {
                return false;
            }
        }
    }
    let domain_area = (u1 - u0) * (v1 - v0);
    (polygon_signed_area(&outer).abs() - domain_area).abs() < 1e-3 * domain_area
}

/// Phase 2: grid vertices of a direct face along a shared edge that runs on a domain side, as (pcurve parameter, model point) sorted along the edge; empty elsewhere
fn grid_edge_samples(b: &BRep, fi: usize, grid: &Mesh, er: &BRepRef) -> Vec<(f64, Point)> {
    let mut samples: Vec<(f64, Point)> = Vec::new();
    let mut shared = false;
    for fr in b.edge_faces(er.index as usize) {
        if fr.index as usize != fi {
            shared = true;
        }
    }
    if !shared {
        return samples;
    }
    let ci = b.pcurve_index(er.index as usize, fi, er.orientation);
    if ci < 0 {
        return samples;
    }
    let srf = &b.m_surfaces[b.m_faces[fi].surface_index as usize];
    let (u0, u1) = srf.domain(0).unwrap_or((0.0, 1.0));
    let (v0, v1) = srf.domain(1).unwrap_or((0.0, 1.0));
    let utol = (u1 - u0) * 0.001;
    let vtol = (v1 - v0) * 0.001;
    let c2d = &b.m_curves_2d[ci as usize];
    let (Some(sp), Some(ep)) = (c2d.get_cv(0), c2d.get_cv(c2d.cv_count().saturating_sub(1))) else {
        return samples;
    };
    let at_v0 = (sp[1] - v0).abs() < vtol && (ep[1] - v0).abs() < vtol;
    let at_v1 = (sp[1] - v1).abs() < vtol && (ep[1] - v1).abs() < vtol;
    let at_u0 = (sp[0] - u0).abs() < utol && (ep[0] - u0).abs() < utol;
    let at_u1 = (sp[0] - u1).abs() < utol && (ep[0] - u1).abs() < utol;
    if !at_v0 && !at_v1 && !at_u0 && !at_u1 {
        return samples;
    }
    let mut pts: Vec<(f64, Point)> = Vec::new();
    for vd in grid.vertex.values() {
        let (Some(&iu), Some(&iv)) = (vd.attributes.get("u"), vd.attributes.get("v")) else {
            continue;
        };
        if (at_v0 && (iv - v0).abs() < vtol * 0.1) || (at_v1 && (iv - v1).abs() < vtol * 0.1) {
            pts.push((iu, vd.position()));
        } else if (at_u0 && (iu - u0).abs() < utol * 0.1) || (at_u1 && (iu - u1).abs() < utol * 0.1)
        {
            pts.push((iv, vd.position()));
        }
    }
    pts.sort_by(|x, y| x.0.total_cmp(&y.0));
    pts.dedup_by(|x, y| x.0 == y.0);
    if pts.len() < 2 {
        return samples;
    }
    let varying = if at_v0 || at_v1 { 0 } else { 1 };
    let (t0, t1) = c2d.domain();
    for (p, pt) in pts {
        samples.push((
            t0 + (p - sp[varying]) / (ep[varying] - sp[varying]) * (t1 - t0),
            pt,
        ));
    }
    samples
}

/// Phase 2: the first incident grid supplies the canonical polygon of every shared edge; true when this face's grid disagrees with an earlier one and must be rebuilt
fn grid_boundaries(b: &BRep, fi: usize, grid: &Mesh, boundary: &mut EdgeBoundary) -> bool {
    let mut rebuild = false;
    for er in b.wire_edges(&b.m_faces[fi].wires[0]) {
        let eidx = er.index as usize;
        let samples = grid_edge_samples(b, fi, grid, &er);
        if samples.is_empty() {
            continue;
        }
        let mut parameters = Vec::new();
        let mut bnd = Vec::new();
        for (t, pt) in samples {
            parameters.push(t);
            bnd.push(pt);
        }
        let Some(canonical) = boundary.points.get(&eidx) else {
            boundary.points.insert(eidx, bnd);
            let ci = b.pcurve_index(eidx, fi, er.orientation) as usize;
            boundary.basis.insert(eidx, (fi, ci, parameters));
            continue;
        };
        let mut matches = canonical.len() == bnd.len();
        let mut forward = true;
        let mut backward = true;
        for k in 0..canonical.len().min(bnd.len()) {
            forward = forward && same_boundary_point(&canonical[k], &bnd[k]);
            backward = backward && same_boundary_point(&canonical[k], &bnd[bnd.len() - 1 - k]);
        }
        matches = matches && (forward || backward);
        rebuild = rebuild || !matches;
    }
    rebuild
}

/// Refine the canonical polygon of every edge shared with a curved CDT face, then mark every incident face for rebuild with the same refined polygon
fn refine_shared_boundaries(
    b: &BRep,
    face_direct: &[bool],
    rebuild_grid: &mut [bool],
    boundary: &mut EdgeBoundary,
    angle: f64,
    chord: f64,
) {
    for (&edge, (face, pcurve, parameters)) in &boundary.basis {
        let mut curved_cdt = false;
        for incident in b.edge_faces(edge) {
            let fi = incident.index as usize;
            let cdt = !face_direct[fi] || rebuild_grid[fi];
            curved_cdt = curved_cdt
                || (cdt
                    && !b.m_surfaces[b.m_faces[fi].surface_index as usize].is_planar(None, 0.0));
        }
        if !curved_cdt {
            continue;
        }
        let surface = &b.m_surfaces[b.m_faces[*face].surface_index as usize];
        let curve = &b.m_curves_2d[*pcurve];
        let points = &boundary.points[&edge];
        let mut samples: Vec<Sample> = Vec::new();
        for i in 0..parameters.len() {
            samples.push((
                parameters[i],
                curve.point_at(parameters[i]),
                points[i].clone(),
            ));
        }
        samples.sort_by(|x, y| x.0.total_cmp(&y.0));
        if b.m_edges[edge].start_vertex == b.m_edges[edge].end_vertex {
            let end = curve.domain().1;
            if !samples.is_empty() && samples[samples.len() - 1].0 < end {
                samples.push((end, curve.point_at(end), samples[0].2.clone()));
            }
        }
        let refined = refine_surface_boundary(surface, curve, &samples, angle, chord);
        if refined.len() <= samples.len() {
            continue;
        }
        let mut refined_points = Vec::new();
        let mut refined_samples = Vec::new();
        for (t, uv, p) in refined {
            refined_points.push(p);
            refined_samples.push((t, uv));
        }
        boundary.points.insert(edge, refined_points);
        boundary.samples.insert(edge, refined_samples);
        for incident in b.edge_faces(edge) {
            rebuild_grid[incident.index as usize] = true;
        }
    }
}

/// Interior UV seeds of a rebuilt face: its grid vertices strictly inside the domain
fn grid_interior_uv(srf: &NurbsSurface, grid: &Mesh) -> Vec<Point> {
    let (u0, u1) = srf.domain(0).unwrap_or((0.0, 1.0));
    let (v0, v1) = srf.domain(1).unwrap_or((0.0, 1.0));
    let mut seeds = Vec::new();
    for vertex in grid.vertex.values() {
        let (Some(&u), Some(&v)) = (vertex.attributes.get("u"), vertex.attributes.get("v")) else {
            continue;
        };
        if u > u0 && u < u1 && v > v0 && v < v1 {
            seeds.push(Point::new(u, v, 0.0));
        }
    }
    seeds.sort_by(|x, y| x[0].total_cmp(&y[0]).then(x[1].total_cmp(&y[1])));
    seeds
}

/// Phase 3: map the canonical points of edge `ei` onto pcurve `ci` of face `fi`, checked in model space; false when a point cannot be lifted
/// Planarity tolerance for a surface of any size: 1e-9 of its control-point bounding box diagonal, never below the zero tolerance
fn planar_patch_tolerance(srf: &NurbsSurface) -> f64 {
    let mut lo: [f64; 3] = [1e300; 3];
    let mut hi: [f64; 3] = [-1e300; 3];
    for i in 0..srf.cv_count(0) {
        for j in 0..srf.cv_count(1) {
            let p = srf.get_cv(i, j).unwrap_or_default();
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
    }
    let diagonal = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    (1e-9 * diagonal).max(Tolerance::ZERO_TOLERANCE)
}

/// True for a surface flat within planar_patch_tolerance, whatever its coordinates
fn is_planar_patch(srf: &NurbsSurface) -> bool {
    srf.is_planar(None, planar_patch_tolerance(srf))
}

/// Surface parameters of a point on a degree-1 parallelogram patch by two dot products; None when the patch is not that shape
fn planar_patch_uv(srf: &NurbsSurface, p: &Point) -> Option<(f64, f64)> {
    if srf.degree(0) != 1 || srf.degree(1) != 1 || srf.cv_count(0) != 2 || srf.cv_count(1) != 2 {
        return None;
    }
    let p00 = srf.get_cv(0, 0)?;
    let p10 = srf.get_cv(1, 0)?;
    let p01 = srf.get_cv(0, 1)?;
    let p11 = srf.get_cv(1, 1)?;
    let eu = p10.clone() - p00.clone();
    let ev = p01.clone() - p00.clone();
    let skew = (p11 - p10) - ev.clone();
    if skew.magnitude() > planar_patch_tolerance(srf) {
        return None;
    }
    let eu2 = eu.dot(&eu);
    let ev2 = ev.dot(&ev);
    if eu2 <= 0.0 || ev2 <= 0.0 {
        return None;
    }
    let d = p.clone() - p00;
    let (u0, u1) = srf.domain(0)?;
    let (v0, v1) = srf.domain(1)?;
    Some((u0 + d.dot(&eu) / eu2 * (u1 - u0), v0 + d.dot(&ev) / ev2 * (v1 - v0)))
}

/// Parameter of the closest point on a two-point degree-1 pcurve by one projection; None for any other curve
fn linear_pcurve_parameter(crv: &NurbsCurve, uv: (f64, f64)) -> Option<f64> {
    if crv.degree() != 1 || crv.is_rational() || crv.cv_count() != 2 {
        return None;
    }
    let c0 = crv.get_cv(0)?;
    let c1 = crv.get_cv(1)?;
    let dx = c1[0] - c0[0];
    let dy = c1[1] - c0[1];
    let length_squared = dx * dx + dy * dy;
    if length_squared <= 0.0 {
        return None;
    }
    let fraction = (((uv.0 - c0[0]) * dx + (uv.1 - c0[1]) * dy) / length_squared).clamp(0.0, 1.0);
    let (t0, t1) = crv.domain();
    Some(t0 + fraction * (t1 - t0))
}

fn lift_canonical(
    b: &BRep,
    fi: usize,
    ei: usize,
    ci: usize,
    boundary: &EdgeBoundary,
    samples: &mut Vec<Sample>,
) -> bool {
    let face = &b.m_faces[fi];
    let edge = &b.m_edges[ei];
    let srf = &b.m_surfaces[face.surface_index as usize];
    let crv = &b.m_curves_2d[ci];
    let planar = is_planar_patch(srf);
    let cached = boundary
        .basis
        .get(&ei)
        .is_some_and(|basis| basis.0 == fi && basis.1 == ci)
        && boundary.samples.contains_key(&ei);
    let points = &boundary.points[&ei];
    for (index, p) in points.iter().enumerate() {
        let (mut t, mut q) = if cached {
            boundary.samples[&ei][index].clone()
        } else {
            let patch_uv = if planar { planar_patch_uv(srf, p) } else { None };
            let (u, v) = match patch_uv {
                Some(uv) => uv,
                None => srf.closest_parameters(p),
            };
            let t = match patch_uv.and_then(|uv| linear_pcurve_parameter(crv, uv)) {
                Some(t) => t,
                None => crv.closest_parameter(&Point::new(u, v, 0.0)),
            };
            (t, crv.point_at(t))
        };
        let scale = p[0].abs().max(p[1].abs()).max(p[2].abs()).max(1.0);
        let tolerance = edge
            .tolerance
            .max(face.tolerance)
            .max(f64::EPSILON.sqrt() * scale);
        if srf
            .point_at(q[0], q[1])
            .is_none_or(|lifted| lifted.distance(p, None) > tolerance)
        {
            t = boundary_parameter(srf, crv, p);
            q = crv.point_at(t);
            if srf
                .point_at(q[0], q[1])
                .is_none_or(|lifted| lifted.distance(p, None) > tolerance)
            {
                return false;
            }
        }
        samples.push((t, q, p.clone()));
    }
    samples.sort_by(|x, y| x.0.total_cmp(&y.0));
    samples.dedup_by(|x, y| x.0 == y.0);
    true
}

/// Phase 3: fresh samples of a pcurve nobody has sampled yet, refined to the face's angle and chord; empty when the surface cannot be evaluated
fn fresh_samples(srf: &NurbsSurface, crv: &NurbsCurve, angle: f64, chord: f64) -> Vec<Sample> {
    let count = (crv.cv_count() * 4)
        .max((360.0 / angle.max(0.1)).ceil() as usize)
        .min(4096);
    let mut points = Vec::new();
    let mut parameters = Vec::new();
    if crv.degree() <= 1 && !crv.is_rational() && is_planar_patch(srf) {
        for k in 0..crv.cv_count() {
            points.push(crv.get_cv(k).unwrap_or_default());
            parameters.push(crv.greville_abcissa(k));
        }
    } else {
        (points, parameters) = crv.divide_by_count(count, true);
    }
    let mut samples: Vec<Sample> = Vec::new();
    for k in 0..points.len() {
        let q = points[k].clone();
        let Some(p) = srf.point_at(q[0], q[1]) else {
            return Vec::new();
        };
        samples.push((parameters[k], q, p));
    }
    refine_surface_boundary(srf, crv, &samples, angle, chord)
}

/// Phase 3: samples of one edge use of a CDT face in traversal direction, a closed edge repeating its first point at the end; false when the edge has no pcurve or cannot be lifted
fn edge_use_samples(
    b: &BRep,
    fi: usize,
    er: &BRepRef,
    boundary: &mut EdgeBoundary,
    angle: f64,
    chord: f64,
    samples: &mut Vec<Sample>,
) -> bool {
    let ei = er.index as usize;
    let edge = &b.m_edges[ei];
    let ci = b.pcurve_index(ei, fi, er.orientation);
    if ci < 0 {
        return false;
    }
    let crv = &b.m_curves_2d[ci as usize];
    let canonical = boundary.points.contains_key(&ei);
    if canonical {
        if !lift_canonical(b, fi, ei, ci as usize, boundary, samples) {
            return false;
        }
    } else {
        *samples = fresh_samples(
            &b.m_surfaces[b.m_faces[fi].surface_index as usize],
            crv,
            angle,
            chord,
        );
        let mut positions = Vec::new();
        for sample in samples.iter() {
            positions.push(sample.2.clone());
        }
        boundary.points.insert(ei, positions);
    }
    if edge.start_vertex == edge.end_vertex && samples.len() > 1 {
        let first = samples[0].clone();
        if !same_boundary_point(&first.2, &samples[samples.len() - 1].2) {
            samples.push((crv.domain().1, first.1, first.2));
        }
    }
    if er.orientation == BRepOrientation::Reversed {
        samples.reverse();
    }
    samples.len() >= 2
}

/// Phase 3: trim loops of a CDT face, every edge use keeping its boundary-node identities; false when some edge cannot be sampled
fn trim_loops(
    b: &BRep,
    fi: usize,
    boundary: &mut EdgeBoundary,
    angle: f64,
    chord: f64,
    loops: &mut TrimLoops,
    uses: &mut Vec<EdgeUse>,
) -> bool {
    let face = &b.m_faces[fi];
    for wi in 0..face.wires.len() {
        let mut uv = Vec::new();
        let mut xyz = Vec::new();
        for er in b.wire_edges(&face.wires[wi]) {
            let mut samples: Vec<Sample> = Vec::new();
            if !edge_use_samples(b, fi, &er, boundary, angle, chord, &mut samples) {
                return false;
            }
            uses.push((er.index as usize, wi, uv.len(), samples.len()));
            for sample in samples.iter().take(samples.len() - 1) {
                uv.push(sample.1.clone());
                xyz.push(sample.2.clone());
            }
        }
        loops.uv.push(uv);
        loops.xyz.push(xyz);
    }
    true
}

/// Tag every boundary vertex of a CDT mesh with the edge use it samples; each use keeps both ends, including the next edge's start
/// Phase 3 for a planar face: the sampled loops triangulated as one polygon with holes, wound to the surface normal, every loop vertex tagged boundary/{loop}/{sample} as mesh_loops does; no grid, no surface evaluation
fn planar_loops_mesh(srf: &NurbsSurface, loops: &TrimLoops) -> Mesh {
    use crate::remesh_cdt::{cdt_triangulate, project_2d, signed_area};
    let mut mesh = Mesh::new();
    if loops.xyz.is_empty() || loops.xyz[0].len() < 3 {
        return mesh;
    }
    let mut all_pts: Vec<Point> = Vec::new();
    for loop_pts in &loops.xyz {
        all_pts.extend(loop_pts.iter().cloned());
    }
    let (origin, xaxis, yaxis, _zaxis) = Polyline::new(all_pts).get_average_plane();
    let mut border = loops.xyz[0].clone();
    let mut border_2d = project_2d(&border, &origin, &xaxis, &yaxis);
    if signed_area(&border_2d) < 0.0 {
        border.reverse();
        border_2d.reverse();
    }
    let mut holes: Vec<Vec<Point>> = Vec::new();
    let mut holes_2d: Vec<Vec<Point>> = Vec::new();
    for loop_pts in loops.xyz.iter().skip(1) {
        if loop_pts.len() < 3 {
            continue;
        }
        let mut hole = loop_pts.clone();
        let mut hole_2d = project_2d(&hole, &origin, &xaxis, &yaxis);
        if signed_area(&hole_2d) > 0.0 {
            hole.reverse();
            hole_2d.reverse();
        }
        holes.push(hole);
        holes_2d.push(hole_2d);
    }
    let mut vkeys = Vec::new();
    for p in &border {
        vkeys.push(mesh.add_vertex(p.clone(), None));
    }
    for hole in &holes {
        for p in hole {
            vkeys.push(mesh.add_vertex(p.clone(), None));
        }
    }
    for (a, b, c) in cdt_triangulate(&border_2d, &holes_2d) {
        if a != b && b != c && c != a {
            mesh.add_face(vec![vkeys[a], vkeys[b], vkeys[c]], None);
        }
    }
    let (u0, u1) = srf.domain(0).unwrap_or((0.0, 1.0));
    let (v0, v1) = srf.domain(1).unwrap_or((0.0, 1.0));
    let normal = srf.normal_at(0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let first: Option<Vec<usize>> = mesh.face.values().next().cloned();
    if let Some(fverts) = first {
        let a = mesh.vertex[&fverts[0]].position();
        let b = mesh.vertex[&fverts[1]].position();
        let c = mesh.vertex[&fverts[2]].position();
        if (b.clone() - a.clone()).cross(&(c - a)).dot(&normal) < 0.0 {
            mesh.flip();
        }
    }
    let mut lookup: HashMap<(u64, u64, u64), (usize, usize)> = HashMap::new();
    for (li, loop_pts) in loops.xyz.iter().enumerate() {
        for (k, p) in loop_pts.iter().enumerate() {
            lookup.entry((p[0].to_bits(), p[1].to_bits(), p[2].to_bits())).or_insert((li, k));
        }
    }
    for vd in mesh.vertex.values_mut() {
        vd.set_normal(normal[0], normal[1], normal[2]);
        if let Some(&(li, k)) = lookup.get(&(vd.x.to_bits(), vd.y.to_bits(), vd.z.to_bits())) {
            vd.attributes.insert(format!("boundary/{li}/{k}"), 1.0);
        }
    }
    mesh
}

fn tag_edge_uses(mesh: &mut Mesh, loops: &TrimLoops, uses: &[EdgeUse]) {
    for (use_id, &(edge, li, start, count)) in uses.iter().enumerate() {
        let length = loops.uv[li].len();
        if length == 0 {
            continue;
        }
        for sample in 0..count {
            let key = format!("boundary/{li}/{}", (start + sample) % length);
            let tag = format!("brep_edge/{edge}/{use_id}/{sample}");
            for vd in mesh.vertex.values_mut() {
                if vd.attributes.contains_key(&key) {
                    vd.attributes.insert(tag.clone(), 1.0);
                }
            }
            if sample + 1 >= count {
                continue;
            }
            let interval = format!("boundary_interval/{li}/{}", (start + sample) % length);
            let interval_tag = format!("brep_edge_interval/{edge}/{use_id}/{sample}");
            for vd in mesh.vertex.values_mut() {
                if let Some(&t) = vd.attributes.get(&interval) {
                    vd.attributes.insert(interval_tag.clone(), t);
                }
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Serialization helpers
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Serialize, Deserialize)]
struct RefJson {
    index: i32,
    orientation: String,
}

fn refs_to_json(refs: &[BRepRef]) -> Vec<RefJson> {
    let mut arr = Vec::new();
    for r in refs {
        arr.push(RefJson {
            index: r.index,
            orientation: orientation_to_str(r.orientation).to_string(),
        });
    }
    arr
}

fn refs_from_json(arr: &[RefJson]) -> Vec<BRepRef> {
    let mut refs = Vec::new();
    for r in arr {
        refs.push(BRepRef::new(r.index, orientation_from_str(&r.orientation)));
    }
    refs
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

fn refs_to_proto(refs: &[BRepRef]) -> Vec<crate::proto::BRepRef> {
    let mut out = Vec::new();
    for r in refs {
        out.push(crate::proto::BRepRef {
            index: r.index,
            orientation: r.orientation as i32,
        });
    }
    out
}

fn refs_from_proto(refs: &[crate::proto::BRepRef]) -> Vec<BRepRef> {
    let mut out = Vec::new();
    for r in refs {
        out.push(BRepRef::new(r.index, orientation_from_i32(r.orientation)));
    }
    out
}

/// Boundary representation after OCCT's TopoDS/BRep model: geometry pools, indexed shape tables, every parent -> child link a BRepRef carrying the orientation
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
            surfacecolor: Color::lightgrey(),
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

    /// Copy (new guid, same data)
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = std::sync::OnceLock::new();
        copy
    }

    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Clear the guid so a fresh one mints lazily on next read
    pub fn refresh_guid(&mut self) {
        self.guid = std::sync::OnceLock::new();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Axis-aligned box centered at the origin: 6 faces, 12 edges, 8 vertices, one solid
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

    /// Cylinder along +Z: one periodic body face (seam edge) and two planar caps
    pub fn create_cylinder(radius: f64, height: f64) -> Self {
        let mut b = BRep::new();
        b.name = "cylinder".to_string();
        let body = Primitives::cylinder_surface(0.0, 0.0, 0.0, radius, height);
        let p_bot = body.point_at_corner(0, 0).unwrap_or_default();
        let p_top = body.point_at_corner(0, 1).unwrap_or_default();
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
        let sh = b.add_shell(&[
            BRepRef::new(f_body as i32, F),
            BRepRef::new(f_bot as i32, F),
            BRepRef::new(f_top as i32, F),
        ]);
        b.add_solid(&[BRepRef::new(sh as i32, F)]);
        b
    }

    /// Sphere centered at the origin: one face, a seam meridian and two degenerated pole edges
    pub fn create_sphere(radius: f64) -> Self {
        let mut b = BRep::new();
        b.name = "sphere".to_string();
        let srf = Primitives::sphere_surface(0.0, 0.0, 0.0, radius);
        let (u0, u1) = srf.domain(0).unwrap_or((0.0, 1.0));
        let (v0, v1) = srf.domain(1).unwrap_or((0.0, 1.0));
        let v_s = b.add_vertex(&Point::new(0.0, 0.0, -radius), 0.0) as i32;
        let v_n = b.add_vertex(&Point::new(0.0, 0.0, radius), 0.0) as i32;
        let c_seam = b.add_curve_3d(&srf.iso_curve(1, u0).unwrap_or_default()) as i32;
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
        let wi = b.add_wire(&[
            BRepRef::new(e_south as i32, F),
            BRepRef::new(e_seam as i32, F),
            BRepRef::new(e_north as i32, R),
            BRepRef::new(e_seam as i32, R),
        ]);
        let fi = b.add_face(si as i32, &[BRepRef::new(wi as i32, F)], 0.0);
        let sh = b.add_shell(&[BRepRef::new(fi as i32, F)]);
        b.add_solid(&[BRepRef::new(sh as i32, F)]);
        b
    }

    /// Cone along +Z: base circle at z=0, apex at z=height (degenerated apex edge), planar base
    pub fn create_cone(radius: f64, height: f64) -> Self {
        let mut b = BRep::new();
        b.name = "cone".to_string();
        let body = Primitives::cone_surface(0.0, 0.0, 0.0, radius, height);
        let p_base = body.point_at_corner(0, 0).unwrap_or_default();
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
        let sh = b.add_shell(&[
            BRepRef::new(f_body as i32, F),
            BRepRef::new(f_base as i32, F),
        ]);
        b.add_solid(&[BRepRef::new(sh as i32, F)]);
        b
    }

    /// Square pyramid: base edge `base` centered at the origin in z=0, apex at (0,0,height)
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
            let a = i;
            let c = (i + 1) % 4;
            let srf = bilinear_patch(
                &b.m_vertices[a].point,
                &b.m_vertices[c].point,
                &b.m_vertices[v_apex].point,
                &b.m_vertices[v_apex].point,
            );
            let si = b.add_surface(&srf);
            let e_ac = pb.edge(&mut b, a, c);
            let e_c = pb.edge(&mut b, c, v_apex);
            let e_a = pb.edge(&mut b, a, v_apex);
            let e_deg = b.add_edge(-1, v_apex as i32, v_apex as i32);
            let ac_fwd = b.m_edges[e_ac].start_vertex == a as i32;
            let c_ac = b.add_curve_2d(&if ac_fwd {
                uv_line(0.0, 0.0, 1.0, 0.0)
            } else {
                uv_line(1.0, 0.0, 0.0, 0.0)
            }) as i32;
            b.add_pcurve(e_ac, si, c_ac, -1);
            let c_c = b.add_curve_2d(&uv_line(1.0, 0.0, 1.0, 1.0)) as i32;
            b.add_pcurve(e_c, si, c_c, -1);
            let c_deg = b.add_curve_2d(&uv_line(1.0, 1.0, 0.0, 1.0)) as i32;
            b.add_pcurve(e_deg, si, c_deg, -1);
            let c_a = b.add_curve_2d(&uv_line(0.0, 0.0, 0.0, 1.0)) as i32;
            b.add_pcurve(e_a, si, c_a, -1);
            let wire = b.add_wire(&[
                BRepRef::new(e_ac as i32, if ac_fwd { F } else { R }),
                BRepRef::new(e_c as i32, F),
                BRepRef::new(e_deg as i32, F),
                BRepRef::new(e_a as i32, R),
            ]);
            faces.push(BRepRef::new(
                b.add_face(si as i32, &[BRepRef::new(wire as i32, F)], 0.0) as i32,
                F,
            ));
        }
        let sh = b.add_shell(&faces);
        b.add_solid(&[BRepRef::new(sh as i32, F)]);
        b
    }

    /// Torus in the XY plane: one face closed in both directions, two seam edges, one vertex
    pub fn create_torus(major_radius: f64, minor_radius: f64) -> Self {
        let mut b = BRep::new();
        b.name = "torus".to_string();
        let srf = Primitives::torus_surface(0.0, 0.0, 0.0, major_radius, minor_radius);
        let (u0, u1) = srf.domain(0).unwrap_or((0.0, 1.0));
        let (v0, v1) = srf.domain(1).unwrap_or((0.0, 1.0));
        let v = b.add_vertex(&srf.point_at_corner(0, 0).unwrap_or_default(), 0.0) as i32;
        let c_u = b.add_curve_3d(&srf.iso_curve(1, u0).unwrap_or_default()) as i32;
        let e_u = b.add_edge(c_u, v, v);
        let c_v = b.add_curve_3d(&srf.iso_curve(0, v0).unwrap_or_default()) as i32;
        let e_v = b.add_edge(c_v, v, v);
        let si = b.add_surface(&srf);
        let c_bottom = b.add_curve_2d(&uv_line(u0, v0, u1, v0)) as i32;
        let c_top = b.add_curve_2d(&uv_line(u0, v1, u1, v1)) as i32;
        b.add_pcurve(e_v, si, c_bottom, c_top);
        let c_right = b.add_curve_2d(&uv_line(u1, v0, u1, v1)) as i32;
        let c_left = b.add_curve_2d(&uv_line(u0, v0, u0, v1)) as i32;
        b.add_pcurve(e_u, si, c_right, c_left);
        let wi = b.add_wire(&[
            BRepRef::new(e_v as i32, F),
            BRepRef::new(e_u as i32, F),
            BRepRef::new(e_v as i32, R),
            BRepRef::new(e_u as i32, R),
        ]);
        let fi = b.add_face(si as i32, &[BRepRef::new(wi as i32, F)], 0.0);
        let sh = b.add_shell(&[BRepRef::new(fi as i32, F)]);
        b.add_solid(&[BRepRef::new(sh as i32, F)]);
        b
    }

    /// Axis-aligned box with a cylindrical through-hole along Z
    pub fn create_block_with_hole(sx: f64, sy: f64, sz: f64, hole_radius: f64) -> Self {
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
        faces.push(BRepRef::new(
            body_face(&mut b, si_bore, e_bot, e_seam, e_top) as i32,
            R,
        ));
        for (fi, fv) in BOX_FACES.iter().enumerate().take(2) {
            let cap = quad_patch(&b, fv);
            let si = b.add_surface(&cap);
            let outer = pb.wire_refs(&mut b, si, fv);
            let e_hole = if fi == 0 { e_bot } else { e_top };
            let c2d = project_to_patch(
                &b.m_curves_3d[b.m_edges[e_hole].curve_3d_index as usize],
                &cap,
            );
            let o = if uv_signed_area(&c2d) < 0.0 { F } else { R };
            let ci = b.add_curve_2d(&c2d) as i32;
            b.add_pcurve(e_hole, si, ci, -1);
            let w_outer = b.add_wire(&outer);
            let w_inner = b.add_wire(&[BRepRef::new(e_hole as i32, o)]);
            let wires = [
                BRepRef::new(w_outer as i32, F),
                BRepRef::new(w_inner as i32, F),
            ];
            faces.push(BRepRef::new(b.add_face(si as i32, &wires, 0.0) as i32, F));
        }
        let sh = b.add_shell(&faces);
        b.add_solid(&[BRepRef::new(sh as i32, F)]);
        b
    }

    /// One planar face per closed polyline, holes[i] the closed polylines bounding the holes of face i; coincident vertices and edges are shared, closed sheets become solids
    pub fn from_polylines(polylines: &[Polyline], holes: &[Vec<Polyline>]) -> Self {
        let mut b = BRep::new();
        b.name = "polysurface".to_string();
        let tol = 1e-6;
        let mut pb = PolyFaceBuilder::new();
        for (pi, pl) in polylines.iter().enumerate() {
            let pts = open_points(pl);
            if pts.len() < 3 {
                continue;
            }
            let (org, plane) = pl.get_fast_plane();
            if !plane.is_valid() {
                continue;
            }
            let xa = plane.x_axis();
            let ya = plane.y_axis();
            let outer_area = signed_area_in_plane(&pts, &org, &xa, &ya);
            let mut vi = Vec::new();
            for pt in &pts {
                vi.push(find_or_add_vertex(&mut b, pt, tol));
            }
            let mut all_pts = pts.clone();
            let mut hole_cycles: Vec<Vec<usize>> = Vec::new();
            if pi < holes.len() {
                for h in &holes[pi] {
                    let mut hp = open_points(h);
                    if hp.len() < 3 {
                        continue;
                    }
                    if signed_area_in_plane(&hp, &org, &xa, &ya) * outer_area > 0.0 {
                        hp.reverse();
                    }
                    let mut cycle = Vec::new();
                    for pt in &hp {
                        cycle.push(find_or_add_vertex(&mut b, pt, tol));
                    }
                    hole_cycles.push(cycle);
                    all_pts.extend(hp);
                }
            }
            let srf = planar_patch_through(&all_pts, &org, &xa, &ya);
            pb.face_with_holes(&mut b, &srf, &vi, &hole_cycles);
        }
        close_free_faces(&mut b);
        b
    }

    /// One planar face per closed curve with optional hole curves (inner wires); closed sheets become solids
    pub fn from_nurbscurves(curves: &[NurbsCurve], holes: &[Vec<NurbsCurve>]) -> Self {
        let mut b = BRep::new();
        b.name = "polysurface".to_string();
        let tol = 1e-6;
        for (ci, crv) in curves.iter().enumerate() {
            let mut pts = cv_points(crv);
            if pts.len() >= 2 && pts[0].distance(&pts[pts.len() - 1], None) < tol {
                pts.pop();
            }
            if pts.len() < 3 {
                continue;
            }
            let (org, plane) = Polyline::new(pts.clone()).get_fast_plane();
            if !plane.is_valid() {
                continue;
            }
            if ci < holes.len() {
                for h in &holes[ci] {
                    pts.extend(cv_points(h));
                }
            }
            let si = b.add_surface(&planar_patch_through(
                &pts,
                &org,
                &plane.x_axis(),
                &plane.y_axis(),
            ));
            let mut wires = vec![BRepRef::new(curve_wire(&mut b, crv, si, tol) as i32, F)];
            if ci < holes.len() {
                for h in &holes[ci] {
                    wires.push(BRepRef::new(curve_wire(&mut b, h, si, tol) as i32, F));
                }
            }
            b.add_face(si as i32, &wires, 0.0);
        }
        close_free_faces(&mut b);
        b
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn vertex_count(&self) -> usize {
        self.m_vertices.len()
    }

    pub fn edge_count(&self) -> usize {
        self.m_edges.len()
    }

    pub fn wire_count(&self) -> usize {
        self.m_wires.len()
    }

    pub fn face_count(&self) -> usize {
        self.m_faces.len()
    }

    pub fn shell_count(&self) -> usize {
        self.m_shells.len()
    }

    pub fn solid_count(&self) -> usize {
        self.m_solids.len()
    }

    /// Every reference resolves into its table, every face has a surface and an outer wire, every edge two vertices and (unless degenerated) a 3D curve
    pub fn is_valid(&self) -> bool {
        if self.m_faces.is_empty() {
            return false;
        }
        let ok = |i: i32, n: usize| i >= 0 && (i as usize) < n;
        for e in &self.m_edges {
            if !ok(e.start_vertex, self.m_vertices.len())
                || !ok(e.end_vertex, self.m_vertices.len())
            {
                return false;
            }
            if !e.degenerated && !ok(e.curve_3d_index, self.m_curves_3d.len()) {
                return false;
            }
            for pc in &e.pcurves {
                if !ok(pc.surface_index, self.m_surfaces.len())
                    || !ok(pc.curve_2d_index, self.m_curves_2d.len())
                {
                    return false;
                }
                if pc.curve_2d_index_2 >= 0 && !ok(pc.curve_2d_index_2, self.m_curves_2d.len()) {
                    return false;
                }
            }
        }
        for w in &self.m_wires {
            if w.edges.is_empty() {
                return false;
            }
            for r in &w.edges {
                if !ok(r.index, self.m_edges.len()) {
                    return false;
                }
            }
        }
        for f in &self.m_faces {
            if !ok(f.surface_index, self.m_surfaces.len()) || f.wires.is_empty() {
                return false;
            }
            for r in &f.wires {
                if !ok(r.index, self.m_wires.len()) {
                    return false;
                }
            }
        }
        for s in &self.m_shells {
            for r in &s.faces {
                if !ok(r.index, self.m_faces.len()) {
                    return false;
                }
            }
        }
        for s in &self.m_solids {
            for r in &s.shells {
                if !ok(r.index, self.m_shells.len()) {
                    return false;
                }
            }
        }
        true
    }

    /// BRep_Tool::IsClosed(shell): every non-degenerated edge is used exactly twice by the shell's faces (a seam counts twice through its two pcurves)
    pub fn is_closed(&self, shell_index: usize) -> bool {
        if shell_index >= self.m_shells.len() {
            return false;
        }
        let mut uses = vec![0usize; self.m_edges.len()];
        for fr in &self.m_shells[shell_index].faces {
            for wr in &self.m_faces[fr.index as usize].wires {
                for er in self.wire_edges(wr) {
                    uses[er.index as usize] += 1;
                }
            }
        }
        for (i, e) in self.m_edges.iter().enumerate() {
            if !e.degenerated && uses[i] != 0 && uses[i] != 2 {
                return false;
            }
        }
        !self.m_shells[shell_index].faces.is_empty()
    }

    /// At least one solid, and every shell of every solid is closed
    pub fn is_solid(&self) -> bool {
        if self.m_solids.is_empty() {
            return false;
        }
        for s in &self.m_solids {
            for r in &s.shells {
                if !self.is_closed(r.index as usize) {
                    return false;
                }
            }
        }
        true
    }

    /// Orientation of a face inside its first parent shell; Forward for a free face
    pub fn face_orientation(&self, face_index: usize) -> BRepOrientation {
        for s in &self.m_shells {
            for r in &s.faces {
                if r.index as usize == face_index {
                    return r.orientation;
                }
            }
        }
        BRepOrientation::Forward
    }

    /// BRep_Tool::CurveOnSurface(E, F): the pcurve index of an edge on a face's surface for the given use orientation (the REVERSED pcurve on a seam); -1 if none
    pub fn pcurve_index(
        &self,
        edge_index: usize,
        face_index: usize,
        orientation: BRepOrientation,
    ) -> i32 {
        if edge_index >= self.m_edges.len() || face_index >= self.m_faces.len() {
            return -1;
        }
        let si = self.m_faces[face_index].surface_index;
        for pc in &self.m_edges[edge_index].pcurves {
            if pc.surface_index == si {
                if orientation == BRepOrientation::Reversed && pc.curve_2d_index_2 >= 0 {
                    return pc.curve_2d_index_2;
                }
                return pc.curve_2d_index;
            }
        }
        -1
    }

    /// The edges of a wire composed with the wire's own orientation (a Reversed wire is traversed backwards with every edge reversed)
    pub fn wire_edges(&self, wire: &BRepRef) -> Vec<BRepRef> {
        let mut out = Vec::new();
        if wire.index < 0 || wire.index as usize >= self.m_wires.len() {
            return out;
        }
        for r in &self.m_wires[wire.index as usize].edges {
            out.push(BRepRef::new(
                r.index,
                brep_compose(wire.orientation, r.orientation),
            ));
        }
        if wire.orientation == BRepOrientation::Reversed {
            out.reverse();
        }
        out
    }

    /// Faces sharing an edge, each with the orientation of that edge use
    pub fn edge_faces(&self, edge_index: usize) -> Vec<BRepRef> {
        let mut out = Vec::new();
        for fi in 0..self.m_faces.len() {
            let fo = self.face_orientation(fi);
            for wr in &self.m_faces[fi].wires {
                for er in self.wire_edges(wr) {
                    if er.index as usize == edge_index {
                        out.push(BRepRef::new(fi as i32, brep_compose(fo, er.orientation)));
                    }
                }
            }
        }
        out
    }

    /// Vertex positions, in vertex order
    pub fn vertex_points(&self) -> Vec<Point> {
        let mut pts = Vec::new();
        for v in &self.m_vertices {
            pts.push(v.point.clone());
        }
        pts
    }

    /// One closed polyline per PLANAR face: the outer wire walked in wire order with its winding untouched (lofts pair loops by it), inner wires ignored; index-aligned with face_planes
    pub fn face_polylines(&self) -> Vec<Polyline> {
        planar_faces(self).0
    }

    /// The plane of every face face_polylines emits: centroid origin, Newell normal flipped for a Reversed face, and the whole set flipped when a closed solid encloses negative volume so normals point outward; free faces keep the wire's sign
    pub fn face_planes(&self) -> Vec<Plane> {
        planar_faces(self).1
    }

    /// BRepLib::UpdateTolerances: raise every edge tolerance to the worst gap between its curve ends (3D and lifted pcurves) and its vertices, every vertex to its worst edge; returns the largest
    pub fn update_tolerances(&mut self) -> f64 {
        let mut worst: f64 = 0.0;
        for ei in 0..self.m_edges.len() {
            let vs_i = self.m_edges[ei].start_vertex as usize;
            let ve_i = self.m_edges[ei].end_vertex as usize;
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
                    if ci < 0 {
                        continue;
                    }
                    let c2 = &self.m_curves_2d[ci as usize];
                    let a = c2.point_at(c2.domain().0);
                    let z = c2.point_at(c2.domain().1);
                    if let Some(p) = srf.point_at(a[0], a[1]) {
                        tol = tol.max(p.distance(&vs, None));
                    }
                    if let Some(p) = srf.point_at(z[0], z[1]) {
                        tol = tol.max(p.distance(&ve, None));
                    }
                }
            }
            self.m_edges[ei].tolerance = tol;
            self.m_vertices[vs_i].tolerance = self.m_vertices[vs_i].tolerance.max(tol);
            self.m_vertices[ve_i].tolerance = self.m_vertices[ve_i].tolerance.max(tol);
            worst = worst.max(tol);
        }
        worst
    }

    /// Volume of the tessellated boundary (divergence theorem); meaningful for solids only
    pub fn volume(&self) -> f64 {
        self.mesh().volume()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Building
    // ═══════════════════════════════════════════════════════════════════════════

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

    /// MakeVertex
    pub fn add_vertex(&mut self, pt: &Point, tolerance: f64) -> usize {
        self.m_vertices.push(BRepVertex {
            point: pt.clone(),
            tolerance,
        });
        self.m_vertices.len() - 1
    }

    /// MakeEdge: curve_3d_index -1 makes a degenerated edge (start == end vertex); tolerance starts at 0
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

    /// UpdateEdge(E, pcurve, S): attach a pcurve on a surface, curve_2d_index_2 for the reversed use on a closed surface; replaces an existing record for the same surface
    pub fn add_pcurve(
        &mut self,
        edge_index: usize,
        surface_index: usize,
        curve_2d_index: i32,
        curve_2d_index_2: i32,
    ) {
        for pc in &mut self.m_edges[edge_index].pcurves {
            if pc.surface_index == surface_index as i32 {
                pc.curve_2d_index = curve_2d_index;
                pc.curve_2d_index_2 = curve_2d_index_2;
                return;
            }
        }
        self.m_edges[edge_index].pcurves.push(BRepCurveOnSurface {
            surface_index: surface_index as i32,
            curve_2d_index,
            curve_2d_index_2,
        });
    }

    /// MakeWire + Add(edges)
    pub fn add_wire(&mut self, edges: &[BRepRef]) -> usize {
        self.m_wires.push(BRepWire {
            edges: edges.to_vec(),
        });
        self.m_wires.len() - 1
    }

    /// MakeFace(S) + Add(wires); the first wire is the outer boundary
    pub fn add_face(&mut self, surface_index: i32, wires: &[BRepRef], tolerance: f64) -> usize {
        self.m_faces.push(BRepFace {
            surface_index,
            wires: wires.to_vec(),
            tolerance,
            facecolor: None,
        });
        self.m_faces.len() - 1
    }

    /// MakeShell + Add(faces)
    pub fn add_shell(&mut self, faces: &[BRepRef]) -> usize {
        self.m_shells.push(BRepShell {
            faces: faces.to_vec(),
        });
        self.m_shells.len() - 1
    }

    /// MakeSolid + Add(shells)
    pub fn add_solid(&mut self, shells: &[BRepRef]) -> usize {
        self.m_solids.push(BRepSolid {
            shells: shells.to_vec(),
        });
        self.m_solids.len() - 1
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Meshing
    // ═══════════════════════════════════════════════════════════════════════════

    /// One welded triangle mesh of every face, wound to the face's outward orientation
    pub fn mesh(&self) -> Mesh {
        let mut polygons: Vec<Vec<Point>> = Vec::new();
        for fm in self.face_meshes() {
            for fverts in fm.face.values() {
                let mut poly = Vec::new();
                for vi in fverts {
                    poly.push(fm.vertex[vi].position());
                }
                polygons.push(poly);
            }
        }
        Mesh::from_polylines(polygons, Some(1e-6))
    }

    /// One mesh per face, in face order (vertices not shared across faces)
    pub fn face_meshes(&self) -> Vec<Mesh> {
        self.face_meshes_q(None)
    }

    /// As face_meshes with a tessellation-quality (max_angle_deg, chord_factor) override for the grid-meshed faces
    pub fn face_meshes_q(&self, quality: Option<(f64, f64)>) -> Vec<Mesh> {
        let nf = self.m_faces.len();
        let (angle, chord) = quality.unwrap_or((20.0, 0.005));
        let mut face_direct = vec![false; nf];
        for (fi, direct) in face_direct.iter_mut().enumerate() {
            *direct = direct_face(self, fi);
        }
        let mut rebuild_grid = vec![false; nf];
        let mut fmesh: Vec<Mesh> = Vec::new();
        for _ in 0..nf {
            fmesh.push(Mesh::new());
        }
        let mut boundary = EdgeBoundary::default();
        for fi in 0..nf {
            if !face_direct[fi] {
                continue;
            }
            let srf = &self.m_surfaces[self.m_faces[fi].surface_index as usize];
            fmesh[fi] = match quality {
                Some((a, c)) => RemeshNurbsSurfaceGrid::from_u_v_q(srf.clone(), 0, 0, a, c),
                None => srf.mesh(),
            };
            rebuild_grid[fi] = grid_boundaries(self, fi, &fmesh[fi], &mut boundary);
        }
        refine_shared_boundaries(
            self,
            &face_direct,
            &mut rebuild_grid,
            &mut boundary,
            angle,
            chord,
        );
        for fi in 0..nf {
            if rebuild_grid[fi] {
                face_direct[fi] = false;
            }
        }
        for fi in 0..nf {
            if face_direct[fi] {
                continue;
            }
            let srf = &self.m_surfaces[self.m_faces[fi].surface_index as usize];
            let mut loops = TrimLoops::default();
            if rebuild_grid[fi] {
                loops.interior_uv = grid_interior_uv(srf, &fmesh[fi]);
            }
            let mut uses: Vec<EdgeUse> = Vec::new();
            if !trim_loops(self, fi, &mut boundary, angle, chord, &mut loops, &mut uses) {
                continue;
            }
            if loops.interior_uv.is_empty() && is_planar_patch(srf) {
                fmesh[fi] = planar_loops_mesh(srf, &loops);
            } else {
                let mut ts = NurbsSurfaceTrimmed::new();
                ts.m_surface = srf.clone();
                fmesh[fi] = ts.mesh_loops(&loops, angle, chord);
            }
            tag_edge_uses(&mut fmesh[fi], &loops, &uses);
        }
        for (fi, fm) in fmesh.iter_mut().enumerate() {
            if self.face_orientation(fi) != BRepOrientation::Reversed {
                continue;
            }
            fm.flip();
            for vd in fm.vertex.values_mut() {
                if let Some(n) = vd.normal() {
                    vd.set_normal(-n[0], -n[1], -n[2]);
                }
            }
        }
        fmesh
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Surface point of a face at (u, v)
    pub fn point_at(&self, face_index: usize, u: f64, v: f64) -> Point {
        if face_index >= self.m_faces.len() {
            return Point::new(0.0, 0.0, 0.0);
        }
        self.m_surfaces[self.m_faces[face_index].surface_index as usize]
            .point_at(u, v)
            .unwrap_or_default()
    }

    /// Surface normal of a face at (u, v), flipped when the face is Reversed in its shell
    pub fn normal_at(&self, face_index: usize, u: f64, v: f64) -> Vector {
        if face_index >= self.m_faces.len() {
            return Vector::new(0.0, 0.0, 0.0);
        }
        let n = self.m_surfaces[self.m_faces[face_index].surface_index as usize].normal_at(u, v);
        if self.face_orientation(face_index) == BRepOrientation::Reversed {
            return Vector::new(-n[0], -n[1], -n[2]);
        }
        n
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Transform surfaces, 3D curves and vertices in place (pcurves are parametric, untouched)
    pub fn transform(&mut self, xform: &Xform) {
        for srf in &mut self.m_surfaces {
            srf.transform(xform);
        }
        for crv in &mut self.m_curves_3d {
            crv.transform(xform);
        }
        for v in &mut self.m_vertices {
            v.point = xform.transform_point(&v.point);
        }
    }

    /// Return a transformed copy
    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut b = self.duplicate();
        b.transform(xform);
        b
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_default()
    }

    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.jsondump()?)?;
        Ok(())
    }

    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        self.to_proto().encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        Self::from_proto(crate::proto::BRep::decode(data)?)
    }

    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    /// The proto message; pb_dumps encodes it and Session embeds it
    pub fn to_proto(&self) -> crate::proto::BRep {
        use prost::Message;
        let mut proto = crate::proto::BRep {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            width: self.width,
            ..Default::default()
        };
        for c in &self.m_curves_2d {
            proto.curves_2d.push(
                crate::proto::NurbsCurve::decode(c.pb_dumps().as_slice()).unwrap_or_default(),
            );
        }
        for c in &self.m_curves_3d {
            proto.curves_3d.push(
                crate::proto::NurbsCurve::decode(c.pb_dumps().as_slice()).unwrap_or_default(),
            );
        }
        for s in &self.m_surfaces {
            proto.surfaces.push(
                crate::proto::NurbsSurface::decode(s.pb_dumps().as_slice()).unwrap_or_default(),
            );
        }
        for v in &self.m_vertices {
            proto.vertices.push(crate::proto::BRepVertex {
                point: Some(crate::proto::Point {
                    x: v.point[0],
                    y: v.point[1],
                    z: v.point[2],
                    ..Default::default()
                }),
                tolerance: v.tolerance,
            });
        }
        for e in &self.m_edges {
            let mut p = crate::proto::BRepEdge {
                curve_3d_index: e.curve_3d_index,
                start_vertex: e.start_vertex,
                end_vertex: e.end_vertex,
                tolerance: e.tolerance,
                degenerated: e.degenerated,
                pcurves: Vec::new(),
            };
            for pc in &e.pcurves {
                p.pcurves.push(crate::proto::BRepCurveOnSurface {
                    surface_index: pc.surface_index,
                    curve_2d_index: pc.curve_2d_index,
                    curve_2d_index_2: pc.curve_2d_index_2,
                });
            }
            proto.edges.push(p);
        }
        for w in &self.m_wires {
            proto.wires.push(crate::proto::BRepWire {
                edges: refs_to_proto(&w.edges),
            });
        }
        for f in &self.m_faces {
            let mut p = crate::proto::BRepFace {
                surface_index: f.surface_index,
                wires: refs_to_proto(&f.wires),
                tolerance: f.tolerance,
                facecolor: None,
            };
            if let Some(fc) = &f.facecolor {
                p.facecolor = Some(crate::proto::Color {
                    r: fc.r,
                    g: fc.g,
                    b: fc.b,
                    a: fc.a,
                    ..Default::default()
                });
            }
            proto.faces.push(p);
        }
        for s in &self.m_shells {
            proto.shells.push(crate::proto::BRepShell {
                faces: refs_to_proto(&s.faces),
            });
        }
        for s in &self.m_solids {
            proto.solids.push(crate::proto::BRepSolid {
                shells: refs_to_proto(&s.shells),
            });
        }
        proto.surfacecolor = Some(crate::proto::Color {
            guid: self.surfacecolor.guid().to_string(),
            name: self.surfacecolor.name.clone(),
            r: self.surfacecolor.r,
            g: self.surfacecolor.g,
            b: self.surfacecolor.b,
            a: self.surfacecolor.a,
        });
        proto
    }

    /// BRep from a decoded proto message
    pub fn from_proto(proto: crate::proto::BRep) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let mut b = BRep::new();
        if !proto.guid.is_empty() {
            b.set_guid(proto.guid.clone());
        }
        b.name = proto.name;
        b.width = proto.width;
        for c in &proto.curves_2d {
            b.m_curves_2d
                .push(NurbsCurve::pb_loads(&c.encode_to_vec())?);
        }
        for c in &proto.curves_3d {
            b.m_curves_3d
                .push(NurbsCurve::pb_loads(&c.encode_to_vec())?);
        }
        for s in &proto.surfaces {
            b.m_surfaces
                .push(NurbsSurface::pb_loads(&s.encode_to_vec())?);
        }
        for v in &proto.vertices {
            let p = v
                .point
                .as_ref()
                .map(|p| Point::new(p.x, p.y, p.z))
                .unwrap_or_default();
            b.m_vertices.push(BRepVertex {
                point: p,
                tolerance: v.tolerance,
            });
        }
        for e in &proto.edges {
            let mut be = BRepEdge {
                curve_3d_index: e.curve_3d_index,
                start_vertex: e.start_vertex,
                end_vertex: e.end_vertex,
                tolerance: e.tolerance,
                degenerated: e.degenerated,
                pcurves: Vec::new(),
            };
            for pc in &e.pcurves {
                be.pcurves.push(BRepCurveOnSurface {
                    surface_index: pc.surface_index,
                    curve_2d_index: pc.curve_2d_index,
                    curve_2d_index_2: pc.curve_2d_index_2,
                });
            }
            b.m_edges.push(be);
        }
        for w in &proto.wires {
            b.m_wires.push(BRepWire {
                edges: refs_from_proto(&w.edges),
            });
        }
        for f in &proto.faces {
            b.m_faces.push(BRepFace {
                surface_index: f.surface_index,
                wires: refs_from_proto(&f.wires),
                tolerance: f.tolerance,
                facecolor: f.facecolor.as_ref().map(|c| Color::new(c.r, c.g, c.b, c.a)),
            });
        }
        for s in &proto.shells {
            b.m_shells.push(BRepShell {
                faces: refs_from_proto(&s.faces),
            });
        }
        for s in &proto.solids {
            b.m_solids.push(BRepSolid {
                shells: refs_from_proto(&s.shells),
            });
        }
        if let Some(c) = proto.surfacecolor {
            b.surfacecolor = Color::with_name(c.r, c.g, c.b, c.a, &c.name);
            b.surfacecolor.set_guid(c.guid);
        }
        Ok(b)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn str(&self) -> String {
        format!(
            "BRep(name={}, faces={}, edges={}, vertices={})",
            self.name,
            self.face_count(),
            self.edge_count(),
            self.vertex_count()
        )
    }

    pub fn repr(&self) -> String {
        format!(
            "BRep(\n  name={},\n  faces={},\n  edges={},\n  vertices={},\n  solid={}\n)",
            self.name,
            self.face_count(),
            self.edge_count(),
            self.vertex_count(),
            if self.is_solid() { "true" } else { "false" }
        )
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

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

impl std::fmt::Display for BRep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Serde
// ═══════════════════════════════════════════════════════════════════════════

impl Serialize for BRep {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut map = serializer.serialize_map(None)?;
        map.serialize_entry("curves_2d", &self.m_curves_2d)?;
        map.serialize_entry("curves_3d", &self.m_curves_3d)?;
        let mut edges: Vec<EdgeJson> = Vec::new();
        for e in &self.m_edges {
            let mut pcurves = Vec::new();
            for pc in &e.pcurves {
                pcurves.push(PCurveJson {
                    curve_2d_index: pc.curve_2d_index,
                    curve_2d_index_2: pc.curve_2d_index_2,
                    surface_index: pc.surface_index,
                });
            }
            edges.push(EdgeJson {
                curve_3d_index: e.curve_3d_index,
                degenerated: e.degenerated,
                end_vertex: e.end_vertex,
                pcurves,
                start_vertex: e.start_vertex,
                tolerance: e.tolerance,
            });
        }
        map.serialize_entry("edges", &edges)?;
        let mut faces: Vec<FaceJson> = Vec::new();
        for f in &self.m_faces {
            faces.push(FaceJson {
                facecolor: f.facecolor.clone(),
                surface_index: f.surface_index,
                tolerance: f.tolerance,
                wires: refs_to_json(&f.wires),
            });
        }
        map.serialize_entry("faces", &faces)?;
        map.serialize_entry("guid", &self.guid())?;
        map.serialize_entry("name", &self.name)?;
        let mut shells: Vec<ShellJson> = Vec::new();
        for s in &self.m_shells {
            shells.push(ShellJson {
                faces: refs_to_json(&s.faces),
            });
        }
        map.serialize_entry("shells", &shells)?;
        let mut solids: Vec<SolidJson> = Vec::new();
        for s in &self.m_solids {
            solids.push(SolidJson {
                shells: refs_to_json(&s.shells),
            });
        }
        map.serialize_entry("solids", &solids)?;
        map.serialize_entry("surfacecolor", &self.surfacecolor)?;
        map.serialize_entry("surfaces", &self.m_surfaces)?;
        map.serialize_entry("type", "BRep")?;
        let mut vertices: Vec<VertexJson> = Vec::new();
        for v in &self.m_vertices {
            vertices.push(VertexJson {
                point: [v.point[0], v.point[1], v.point[2]],
                tolerance: v.tolerance,
            });
        }
        map.serialize_entry("vertices", &vertices)?;
        map.serialize_entry("width", &self.width)?;
        let mut wires: Vec<WireJson> = Vec::new();
        for w in &self.m_wires {
            wires.push(WireJson {
                edges: refs_to_json(&w.edges),
            });
        }
        map.serialize_entry("wires", &wires)?;
        map.end()
    }
}

impl<'de> Deserialize<'de> for BRep {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
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
        if let Some(g) = data.guid {
            b.set_guid(g);
        }
        if let Some(n) = data.name {
            b.name = n;
        }
        if let Some(w) = data.width {
            b.width = w;
        }
        if let Some(c) = data.surfacecolor {
            b.surfacecolor = c;
        }
        b.m_curves_2d = data.curves_2d;
        b.m_curves_3d = data.curves_3d;
        b.m_surfaces = data.surfaces;
        for v in &data.vertices {
            b.m_vertices.push(BRepVertex {
                point: Point::new(v.point[0], v.point[1], v.point[2]),
                tolerance: v.tolerance,
            });
        }
        for e in &data.edges {
            let mut be = BRepEdge {
                curve_3d_index: e.curve_3d_index,
                start_vertex: e.start_vertex,
                end_vertex: e.end_vertex,
                tolerance: e.tolerance,
                degenerated: e.degenerated,
                pcurves: Vec::new(),
            };
            for pc in &e.pcurves {
                be.pcurves.push(BRepCurveOnSurface {
                    surface_index: pc.surface_index,
                    curve_2d_index: pc.curve_2d_index,
                    curve_2d_index_2: pc.curve_2d_index_2,
                });
            }
            b.m_edges.push(be);
        }
        for w in &data.wires {
            b.m_wires.push(BRepWire {
                edges: refs_from_json(&w.edges),
            });
        }
        for f in data.faces {
            b.m_faces.push(BRepFace {
                surface_index: f.surface_index,
                wires: refs_from_json(&f.wires),
                tolerance: f.tolerance,
                facecolor: f.facecolor,
            });
        }
        for s in &data.shells {
            b.m_shells.push(BRepShell {
                faces: refs_from_json(&s.faces),
            });
        }
        for s in &data.solids {
            b.m_solids.push(BRepSolid {
                shells: refs_from_json(&s.shells),
            });
        }
        Ok(b)
    }
}
