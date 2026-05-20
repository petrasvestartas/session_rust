//! Sample Session builders for the viewer / quickstart use. Each function
//! returns a populated `Session` ready to pass to `GpuSession::rebuild_from`.
//!
//! These are intentionally tiny so they tessellate quickly and fit inside
//! the default arena capacities without growth.

use crate::{
    Color, Line, Mesh, NurbsCurve, Plane, Point, PointCloud, Polyline, Session, Vector, OBB,
};

/// One of every renderable kind: a point, a line, a polyline, a small
/// pointcloud, a triangle mesh, an XY-plane, an OBB cube wireframe, and a
/// simple NURBS curve. Useful as a "hello, GPU" scene.
pub fn make_demo_session() -> Session {
    let mut s = Session::new("demo");

    // Point at the origin
    let mut p = Point::new(0.0, 0.0, 0.0);
    p.pointcolor = Color::new(255, 80, 80, 255);
    let _ = s.add_point(p, None);

    // Line — bright green
    let mut l = Line::new(-2.0, -2.0, 0.0, 2.0, -2.0, 0.0);
    l.linecolor = Color::new(80, 220, 80, 255);
    let _ = s.add_line(l, None);

    // Polyline — magenta zigzag
    let mut pl = Polyline::from_coords(vec![
        -2.0, 2.0, 0.0,
        -1.0, 2.5, 0.0,
         0.0, 2.0, 0.0,
         1.0, 2.5, 0.0,
         2.0, 2.0, 0.0,
    ]);
    pl.linecolor = Color::new(220, 80, 220, 255);
    let _ = s.add_polyline(pl, None);

    // PointCloud: 9 colored points on a 3×3 grid
    let mut pc_pts: Vec<Point> = Vec::new();
    let mut pc_cols: Vec<Color> = Vec::new();
    for i in 0..3 {
        for j in 0..3 {
            pc_pts.push(Point::new(i as f32 - 1.0, 4.0 + j as f32, 0.0));
            pc_cols.push(Color::new(
                (i as u8) * 100,
                (j as u8) * 100,
                255,
                255,
            ));
        }
    }
    let pc = PointCloud::new(pc_pts, Vec::new(), pc_cols);
    let _ = s.add_pointcloud(pc, None);

    // Mesh: a single XY-plane triangle, blue-ish
    let mut m = Mesh::new();
    let a = m.add_vertex(Point::new(-2.0, -4.0, 0.0), None);
    let b = m.add_vertex(Point::new(2.0, -4.0, 0.0), None);
    let c = m.add_vertex(Point::new(0.0, -2.5, 0.0), None);
    m.add_face(vec![a, b, c], None);
    m.set_objectcolor(Color::new(80, 120, 220, 255));
    let _ = s.add_mesh(m, None);

    // Plane: tessellated as a quad at +Z=2
    let pl = Plane::from_point_normal(Point::new(0.0, 0.0, 2.0), Vector::new(0.0, 0.0, 1.0));
    let _ = s.add_plane(pl, None);

    // OBB cube wireframe at the origin
    let bb = OBB::new(
        Point::new(0.0, 0.0, -2.0),
        Vector::new(1.0, 0.0, 0.0),
        Vector::new(0.0, 1.0, 0.0),
        Vector::new(0.0, 0.0, 1.0),
        Vector::new(0.5, 0.5, 0.5),
    );
    let _ = s.add_obb(bb);

    // NurbsCurve — a quadratic Bezier-ish curve
    let nc = NurbsCurve::create(false, 3, &[
        Point::new(-3.0, 0.0, 0.0),
        Point::new(-1.5, 1.5, 0.0),
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.5, -1.5, 0.0),
        Point::new(3.0, 0.0, 0.0),
    ]);
    s.objects.nurbscurves.push(nc);

    s
}
