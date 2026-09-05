use session_rust::{Color, Mesh, Point, Polyline, Session, Xform};

fn main() {
    let mut s = Session::new("colors_widths");
    let palette = Color::palette(); // 12 spectral colors

    let mut m1 = Mesh::create_box(400.0, 400.0, 400.0); // Facecolors - 6 faces
    m1.set_facecolors((0..6).map(|i| palette[i * 2].clone()).collect());
    let m1_guid = m1.guid().to_string();

    let mut m2 = Mesh::create_box(400.0, 400.0, 400.0); // POINTCOLORS gradient - 8 vertices
    let n = m2.number_of_vertices();
    m2.set_pointcolors(
        (0..n)
            .map(|i| Color::new(i as f32 / n as f32, 0.2, 1.0 - i as f32 / n as f32, 1.0))
            .collect(),
    );

    let mut m3 = Mesh::create_box(400.0, 400.0, 400.0); // control - unchanged look
    let m3_guid = m3.guid().to_string();
    let n = m3.edges_with_colors().len();
    m3.set_linecolors(vec![Color::red(); n], vec![10.0; n]); // 3× LineUniform.thickness

    let mut pl = Polyline::new(vec![
        Point::new(-600.0, -600.0, 0.0),
        Point::new(600.0, -600.0, 0.0),
        Point::new(1400.0, 600.0, 200.0), // clear of box 3 (x 400..800) - was running inside it
    ]);
    pl.linecolor = Color::red();
    pl.width = 30.0;

    let mut p = Point::new(0.0, -800.0, 0.0);
    p.width = 20.0; // fat dot, 4x the global px

    s.add_mesh(m1, None);
    s.add_mesh(m2, None);
    s.add_mesh(m3, None);
    s.set_xform(&m1_guid, Xform::translation(-600.0, 0.0, 0.0));
    s.set_xform(&m3_guid, Xform::translation(600.0, 0.0, 0.0));
    s.add_polyline(pl, None);
    s.add_point(p, None);
    s.pb_dump("../session_viewer/assets/pb/colors_widths.pb");

    // Why here pdf files are not loaded?
    // Why here we dont have lines and polyliens with various thickness and colors
}
