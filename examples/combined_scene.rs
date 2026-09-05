// Build ONE scene from the coloured demo geometry plus the nine architectural drawings, each
// drawing shifted into its own cell of a grid so nothing overlaps. Output:
// session_data/combined_scene.pb — the single file the viewer loads.
//
// The shift is stored ONCE per file, on that file's group node in the session tree - never baked
// into coordinates. Objects inherit it through the tree (Session::world_xform), and the viewer
// uploads that matrix as the instance row, so placement costs nothing and geometry stays as authored.
// cd /home/petras/code/code_rust/session/session_rust
// cargo run --release --example combined_scene
use std::collections::HashMap;

use session_rust::session::Geometry;
use session_rust::{Color, Mesh, Point, Polyline, Session, Xform};

// A slice, not a fixed-size array, so entries can be commented in and out without
// having to keep a length in sync.
const DRAWINGS: &[&str] = &[
    "../session_data/30700_querschnitt_gg.pb",
    "../session_data/draw_pb_haus25.pb",
    // "../session_data/draw_pc_gru_og2.pb",
    // "../session_data/draw_pd_treppenhaus04.pb",
    // "../session_data/draw_pe_schalungsbild.pb",
    // "../session_data/draw_pf_he.pb",
    // "../session_data/draw_pi_laengsschnitt.pb",
    // "../session_data/draw_pj_grundriss_og2.pb",
    // "../session_data/draw_pj_treppenhaus_a.pb",
];
const GAP: f64 = 2000.0; // mm between cells

/// Only ONE source PDF survived to this machine, so the other eight .pb files cannot be
/// re-imported with the white-knockout filter that `pdf_to_session.py` now applies. Strip the
/// same ink here instead: white-on-white is a PDF mask box, invisible on paper and pure noise
/// in 3D (~1100 phantom rectangles per sheet).
fn white_ink(g: &Geometry) -> bool {
    let white = |c: &session_rust::Color| c.r >= 0.99 && c.g >= 0.99 && c.b >= 0.99;
    match g {
        Geometry::Line(l) => white(&l.linecolor),
        Geometry::Polyline(pl) => white(&pl.linecolor),
        Geometry::NurbsCurve(c) => c.linecolors.first().map(white).unwrap_or(false),
        _ => false,
    }
}

/// World-space points of one object — the session holds its placement, so it is passed in.
/// Curved types are sampled: control points of a rational curve are weighted, so they are
/// not bounds.
fn world_points(g: &Geometry, xf: &Xform) -> Vec<Point> {
    let pts = |xf: &Xform, ps: Vec<Point>| ps.iter().map(|p| xf.transform_point(p)).collect();
    match g {
        Geometry::Point(p) => pts(xf, vec![Point::new(p[0], p[1], p[2])]),
        Geometry::Line(l) => pts(xf, vec![l.start(), l.end()]),
        Geometry::Polyline(pl) => pts(xf, pl.get_points()),
        Geometry::PointCloud(pc) => pts(xf, pc.get_points()),
        Geometry::Mesh(m) => pts(
            xf,
            m.vertex
                .values()
                .map(|v| Point::new(v.x, v.y, v.z))
                .collect(),
        ),
        Geometry::NurbsCurve(c) => {
            let (t0, t1) = c.domain();
            pts(
                xf,
                (0..=16)
                    .map(|i| c.point_at(t0 + (t1 - t0) * i as f64 / 16.0))
                    .collect(),
            )
        }
        Geometry::NurbsSurface(s) => {
            let (u0, u1) = s.domain(0).unwrap_or((0.0, 1.0));
            let (v0, v1) = s.domain(1).unwrap_or((0.0, 1.0));
            let mut ps = Vec::new();
            for i in 0..=8 {
                for j in 0..=8 {
                    let u = u0 + (u1 - u0) * i as f64 / 8.0;
                    let v = v0 + (v1 - v0) * j as f64 / 8.0;
                    if let Some(p) = s.point_at(u, v) {
                        ps.push(p);
                    }
                }
            }
            pts(xf, ps)
        }
        Geometry::BRep(b) => {
            let m = b.mesh();
            pts(
                xf,
                m.vertex
                    .values()
                    .map(|v| Point::new(v.x, v.y, v.z))
                    .collect(),
            )
        }
        Geometry::Plane(p) => pts(xf, vec![p.origin()]),
        Geometry::OBB(o) => pts(xf, vec![o.min_point(), o.max_point()]),
        Geometry::Element(_) => Vec::new(),
    }
}

/// Axis-aligned bounds of a whole session, or None if it holds nothing measurable.
fn bounds(s: &Session) -> Option<([f64; 3], [f64; 3])> {
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for guid in s.order() {
        if let Some(g) = s.get_object(&guid).filter(|g| !white_ink(g)) {
            for p in world_points(g, &s.world_xform(&guid)) {
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
        }
    }
    if lo[0] > hi[0] {
        None
    } else {
        Some((lo, hi))
    }
}

/// guid -> layer name, read back out of a file's tree. `add_*(obj, parent)` files the object as a
/// leaf node NAMED WITH ITS GUID under the layer group (session.rs), so the tree is the only place
/// the PDF's 33 CAD layers survive - and merging with `parent: None` would silently drop them.
fn layer_of(src: &Session) -> HashMap<String, String> {
    let mut map = HashMap::new();
    let Some(root) = src.tree.root() else {
        return map;
    };
    for grp in root.borrow().children() {
        let name = grp.borrow().name.clone();
        for leaf in grp.borrow().children() {
            map.insert(leaf.borrow().name.clone(), name.clone());
        }
    }
    map
}

/// Copy every object of `src` into `dst` with `shift` PREPENDED to its own placement, re-creating
/// the source's layer groups under a per-file prefix so nine sheets' worth of "030 Decken" stay
/// distinct.
fn place(dst: &mut Session, src: &Session, shift: &Xform, file: &str) -> usize {
    let layers = layer_of(src);
    // ONE node per file carries the whole placement. Layer groups hang under it and objects under
    // those, so the shift reaches every object by tree composition - one stored matrix per file
    // instead of one per object, and the geometry stays exactly as authored.
    let file_node = dst.add_group(file);
    let file_key = file_node.borrow().name.clone();
    dst.set_xform(&file_key, shift.clone());

    let mut groups: HashMap<String, std::rc::Rc<std::cell::RefCell<session_rust::tree::TreeNode>>> =
        HashMap::new();
    let mut n = 0;
    for guid in src.order() {
        let Some(g) = src.get_object(&guid) else {
            continue;
        };
        if white_ink(g) {
            continue;
        }
        let parent = layers
            .get(&guid)
            .map(|l| {
                groups
                    .entry(l.clone())
                    .or_insert_with(|| {
                        let node = session_rust::tree::TreeNode::new(&format!("{file} / {l}"));
                        dst.add(&node, &file_node);
                        node
                    })
                    .clone()
            })
            .unwrap_or_else(|| file_node.clone());
        let parent = Some(&parent);
        macro_rules! moved {
            ($rc:expr) => {{
                (**$rc).clone()
            }};
        }
        match g {
            Geometry::Point(p) => {
                dst.add_point(moved!(p), parent);
            }
            Geometry::Line(l) => {
                dst.add_line(moved!(l), parent);
            }
            Geometry::Polyline(pl) => {
                dst.add_polyline(moved!(pl), parent);
            }
            Geometry::PointCloud(pc) => {
                dst.add_pointcloud(moved!(pc), parent);
            }
            Geometry::Mesh(m) => {
                dst.add_mesh(moved!(m), parent);
            }
            Geometry::NurbsCurve(c) => {
                dst.add_nurbscurve(moved!(c), parent);
            }
            Geometry::NurbsSurface(s) => {
                dst.add_nurbssurface(moved!(s), parent);
            }
            Geometry::BRep(b) => {
                dst.add_brep(moved!(b), parent);
            }
            Geometry::Plane(p) => {
                dst.add_plane(moved!(p), parent);
            }
            Geometry::OBB(o) => {
                dst.add_obb(moved!(o));
            }
            Geometry::Element(_) => continue, // elements carry their own sub-geometry, skip
        }
        n += 1;
    }
    n
}

/// The coloured demo geometry of `colors_widths.rs`, as its own session so it lays out like a file.
fn demo() -> Session {
    let mut s = Session::new("colors_widths");
    let palette = Color::palette();

    let mut m1 = Mesh::create_box(400.0, 400.0, 400.0); // FACECOLORS - 6 faces
    m1.set_facecolors((0..6).map(|i| palette[i * 2].clone()).collect());
    let m1_guid = m1.guid().to_string();

    let mut m2 = Mesh::create_box(400.0, 400.0, 400.0); // POINTCOLORS gradient - 8 vertices
    let n = m2.number_of_vertices();
    m2.set_pointcolors(
        (0..n)
            .map(|i| Color::new(i as f32 / n as f32, 0.2, 1.0 - i as f32 / n as f32, 1.0))
            .collect(),
    );

    let mut m3 = Mesh::create_box(400.0, 400.0, 400.0); // LINECOLORS - fat red wireframe
    let m3_guid = m3.guid().to_string();
    let n = m3.edges_with_colors().len();
    m3.set_linecolors(vec![Color::red(); n], vec![10.0; n]);

    let mut pl = Polyline::new(vec![
        Point::new(-600.0, -600.0, 0.0),
        Point::new(600.0, -600.0, 0.0),
        Point::new(1400.0, 600.0, 200.0),
    ]);
    pl.linecolor = Color::red();
    pl.width = 30.0;

    let mut p = Point::new(0.0, -800.0, 0.0);
    p.width = 20.0;

    s.add_mesh(m1, None);
    s.add_mesh(m2, None);
    s.add_mesh(m3, None);
    s.set_xform(&m1_guid, Xform::translation(-600.0, 0.0, 0.0));
    s.set_xform(&m3_guid, Xform::translation(600.0, 0.0, 0.0));
    s.add_polyline(pl, None);
    s.add_point(p, None);
    s
}

fn main() {
    // Load everything first: cell size is the WIDEST file, so one grid step fits them all.
    let mut files: Vec<(String, Session)> = vec![("colors_widths".into(), demo())];
    for path in DRAWINGS {
        let s = Session::pb_load(path);
        if s.lookup.is_empty() {
            println!("skip {path} - empty or missing");
            continue;
        }
        files.push((s.name.clone(), s));
    }

    let boxes: Vec<Option<([f64; 3], [f64; 3])>> = files.iter().map(|(_, s)| bounds(s)).collect();
    let cell_x = boxes
        .iter()
        .flatten()
        .map(|(lo, hi)| hi[0] - lo[0])
        .fold(0.0, f64::max)
        + GAP;
    let cell_y = boxes
        .iter()
        .flatten()
        .map(|(lo, hi)| hi[1] - lo[1])
        .fold(0.0, f64::max)
        + GAP;
    let cols = (files.len() as f64).sqrt().ceil() as usize;

    let mut out = Session::new("combined_scene");
    for (i, (name, src)) in files.iter().enumerate() {
        // Cell corner minus the file's own min corner: each file starts flush at its cell,
        // wherever its coordinates happen to live (drawings are far from the origin).
        let (lo, _) = boxes[i].unwrap_or(([0.0; 3], [0.0; 3]));
        let shift = Xform::translation(
            (i % cols) as f64 * cell_x - lo[0],
            (i / cols) as f64 * cell_y - lo[1],
            -lo[2],
        );
        let n = place(&mut out, src, &shift, name);
        println!("{name}: {n} objects → cell ({}, {})", i % cols, i / cols);
    }

    let groups = out
        .tree
        .root()
        .map(|r| r.borrow().children().len())
        .unwrap_or(0);
    println!(
        "combined: {} objects, {} layer groups, cell {:.0} x {:.0} mm, {} cols",
        out.lookup.len(),
        groups,
        cell_x,
        cell_y,
        cols
    );
    out.pb_dump("../session_viewer/assets/pb/combined_scene.pb");
}
