use crate::mini_test::TestResult;
use crate::tolerance::PI;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use std::cmp::Ordering;

/// Every non-degenerated edge of a solid is used by exactly two faces with opposite composed orientations
fn edges_manifold(b: &crate::BRep) -> bool {
    for ei in 0..b.edge_count() {
        if b.m_edges[ei].degenerated {
            continue;
        }

        let uses = b.edge_faces(ei);

        if uses.len() != 2 {
            return false;
        }

        if uses[0].orientation == uses[1].orientation {
            return false;
        }
    }

    true
}

/// Lexicographic order of two points
fn point_order(a: &[f64; 3], b: &[f64; 3]) -> Ordering {
    a[0].total_cmp(&b[0])
        .then(a[1].total_cmp(&b[1]))
        .then(a[2].total_cmp(&b[2]))
}

/// Sorted positions of the mesh vertices on the v = 0 side
fn boundary_points(mesh: &crate::Mesh) -> Vec<[f64; 3]> {
    let mut points: Vec<[f64; 3]> = Vec::new();

    for v in mesh.vertex.values() {
        if v.attributes.get("v") == Some(&0.0) {
            let position = v.position();
            points.push([position[0], position[1], position[2]]);
        }
    }

    points.sort_by(point_order);

    points
}

/// Unit planar quad face with straight edges and pcurves; returns the face index
fn build_quad_face(b: &mut crate::BRep) -> usize {
    use crate::brep::BRepOrientation;
    use crate::brep::BRepRef;
    use crate::NurbsCurve;
    use crate::NurbsSurface;
    use crate::Point;

    let mut srf = NurbsSurface::new(3, false, 2, 2, 2, 2);
    srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
    srf.set_cv(1, 0, &Point::new(1.0, 0.0, 0.0));
    srf.set_cv(0, 1, &Point::new(0.0, 1.0, 0.0));
    srf.set_cv(1, 1, &Point::new(1.0, 1.0, 0.0));
    let si = b.add_surface(&srf);
    let corners = [
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(1.0, 1.0, 0.0),
        Point::new(0.0, 1.0, 0.0),
    ];

    for corner in &corners {
        b.add_vertex(corner, 0.0);
    }

    let mut refs = Vec::new();

    for i in 0..4usize {
        let j = (i + 1) % 4;
        let ci = b.add_curve_3d(&NurbsCurve::create(
            false,
            1,
            &[corners[i].clone(), corners[j].clone()],
        ));
        let ei = b.add_edge(ci as i32, i as i32, j as i32);
        let c2 = b.add_curve_2d(&NurbsCurve::create(
            false,
            1,
            &[corners[i].clone(), corners[j].clone()],
        ));
        b.add_pcurve(ei, si, c2 as i32, -1);
        refs.push(BRepRef::new(ei as i32, BRepOrientation::Forward));
    }

    let wi = b.add_wire(&refs);

    b.add_face(
        si as i32,
        &[BRepRef::new(wi as i32, BRepOrientation::Forward)],
        0.0,
    )
}

pub fn run_brep_shared_grid_boundary() -> TestResult {
    MINI_TEST!("Shared Grid Boundary", {
        use crate::brep::BRepOrientation;
        use crate::brep::BRepRef;
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
        use crate::BRep;
        use crate::Mesh;
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::Point;

        let mut b = BRep::new();
        let mut surfaces = Vec::new();

        for face in 0..2 {
            let mut points = Vec::new();

            for i in 0..3 {
                for j in 0..2 {
                    let z = if i != 1 {
                        0.0
                    } else if j == 0 || face == 0 {
                        0.5
                    } else {
                        4.0
                    };
                    points.push(Point::new(
                        i as f64 * 0.5,
                        j as f64 * if face == 0 { 1.0 } else { -1.0 },
                        z,
                    ));
                }
            }

            let surface = NurbsSurface::create(false, false, 2, 1, 3, 2, &points).unwrap();
            let si = b.add_surface(&surface);
            let corners = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
            let mut vertices = Vec::new();

            for corner in &corners {
                vertices.push(b.add_vertex(&surface.point_at(corner[0], corner[1]).unwrap(), 0.0));
            }

            let mut edges = Vec::new();

            for side in 0..4 {
                let a = corners[side];
                let z = corners[(side + 1) % 4];
                let mut edge = 0;

                if side != 0 || face != 1 {
                    let dir = if a[0] != z[0] { 0 } else { 1 };
                    let mut curve = surface.iso_curve(dir, a[1 - dir]).unwrap();

                    if a[dir] > z[dir] {
                        curve.reverse();
                    }

                    let ci = b.add_curve_3d(&curve);
                    edge = b.add_edge(
                        ci as i32,
                        vertices[side] as i32,
                        vertices[(side + 1) % 4] as i32,
                    );
                }

                let pc = NurbsCurve::create(
                    false,
                    1,
                    &[Point::new(a[0], a[1], 0.0), Point::new(z[0], z[1], 0.0)],
                );
                let ci = b.add_curve_2d(&pc);
                b.add_pcurve(edge, si, ci as i32, -1);
                edges.push(BRepRef::new(edge as i32, BRepOrientation::Forward));
            }

            let wi = b.add_wire(&edges);
            b.add_face(
                si as i32,
                &[BRepRef::new(wi as i32, BRepOrientation::Forward)],
                1e-8,
            );
            surfaces.push(surface);
        }

        let mut original: Vec<Mesh> = Vec::new();

        for s in &surfaces {
            original.push(RemeshNurbsSurfaceGrid::from_u_v_q(
                s.clone(),
                0,
                0,
                20.0,
                0.005,
            ));
        }

        MINI_CHECK!(
            boundary_points(&original[0]).len() == 7 && boundary_points(&original[1]).len() == 11
        );

        let mut meshes = b.face_meshes_q(Some((20.0, 0.005)));
        let mut first = boundary_points(&meshes[0]);
        let second = boundary_points(&meshes[1]);

        MINI_CHECK!(first == second && first.len() == 7);
        MINI_CHECK!(meshes[0].face.len() == original[0].face.len() && !meshes[1].face.is_empty());

        let mut maximum = 0.0f64;

        for i in 0..first.len() - 1 {
            let a = first[i];
            let z = first[i + 1];
            let actual = surfaces[0].point_at((a[0] + z[0]) * 0.5, 0.0).unwrap();
            let mut sag = 0.0f64;

            for d in 0..3 {
                sag += (actual[d] - (a[d] + z[d]) * 0.5).powi(2);
            }

            maximum = maximum.max(sag.sqrt());
        }

        MINI_CHECK!(maximum <= 0.005 * 1.5);

        let refined = RemeshNurbsSurfaceGrid::from_u_v_q(surfaces[0].clone(), 0, 0, 5.0, 0.001);
        meshes = b.face_meshes_q(Some((5.0, 0.001)));
        first = boundary_points(&meshes[0]);

        MINI_CHECK!(
            first == boundary_points(&meshes[1])
                && !meshes[0].face.is_empty()
                && !meshes[1].face.is_empty()
        );

        for point in boundary_points(&refined) {
            MINI_CHECK!(first.contains(&point));
        }

        let cosine = (5.0 * PI / 180.0).cos();

        for i in 0..first.len() - 1 {
            let a = surfaces[0].normal_at(first[i][0], 0.0);
            let z = surfaces[0].normal_at(first[i + 1][0], 0.0);
            MINI_CHECK!(a.dot(&z) >= cosine - 64.0 * f64::EPSILON);
        }
    })
}

pub fn run_brep_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::BRep;

        let b = BRep::new();

        let sstr = b.str();
        let srepr = b.repr();

        let bcopy = b.duplicate();

        MINI_CHECK!(!b.is_valid());
        MINI_CHECK!(b.face_count() == 0);
        MINI_CHECK!(b.name == "my_brep");
        MINI_CHECK!(!b.guid().is_empty());
        MINI_CHECK!(sstr.contains("BRep"));
        MINI_CHECK!(srepr.contains("name=my_brep"));
        MINI_CHECK!(bcopy.guid() != b.guid());
        MINI_CHECK!(bcopy == b);
        MINI_CHECK!(!(bcopy != b));
    })
}

pub fn run_brep_create_box() -> TestResult {
    MINI_TEST!("Create Box", {
        use crate::BRep;

        let b = BRep::create_box(2.0, 3.0, 4.0);

        MINI_CHECK!(b.is_valid());
        MINI_CHECK!(b.face_count() == 6);
        MINI_CHECK!(b.edge_count() == 12);
        MINI_CHECK!(b.vertex_count() == 8);
        MINI_CHECK!(b.is_solid());
        MINI_CHECK!(b.name == "box");
    })
}

pub fn run_brep_accessors() -> TestResult {
    MINI_TEST!("Accessors", {
        use crate::BRep;

        let b = BRep::create_box(2.0, 3.0, 4.0);

        let vc = b.vertex_count();
        let ec = b.edge_count();
        let wc = b.wire_count();
        let fc = b.face_count();
        let sc = b.shell_count();
        let oc = b.solid_count();
        let pts = b.vertex_points();

        MINI_CHECK!(vc == 8);
        MINI_CHECK!(ec == 12);
        MINI_CHECK!(wc == 6);
        MINI_CHECK!(fc == 6);
        MINI_CHECK!(sc == 1);
        MINI_CHECK!(oc == 1);
        MINI_CHECK!(pts.len() == 8);
        MINI_CHECK!((pts[0][0] + 1.0).abs() < 1e-9);
        MINI_CHECK!(b.m_surfaces.len() == 6);
        MINI_CHECK!(b.m_curves_3d.len() == 12);
        MINI_CHECK!(b.m_curves_2d.len() == 24);
    })
}

pub fn run_brep_add_face() -> TestResult {
    MINI_TEST!("Add Face", {
        use crate::brep::BRepOrientation;
        use crate::brep::BRepRef;
        use crate::BRep;
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::Point;

        let mut b = BRep::new();
        let mut srf = NurbsSurface::new(3, false, 2, 2, 2, 2);
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(1.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 1.0, 0.0));
        srf.set_cv(1, 1, &Point::new(1.0, 1.0, 0.0));
        let si = b.add_surface(&srf);

        let corners = [
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let mut refs = Vec::new();

        for corner in &corners {
            b.add_vertex(corner, 0.0);
        }

        for i in 0..4usize {
            let j = (i + 1) % 4;
            let ci = b.add_curve_3d(&NurbsCurve::create(
                false,
                1,
                &[corners[i].clone(), corners[j].clone()],
            ));
            let ei = b.add_edge(ci as i32, i as i32, j as i32);
            let c2 = b.add_curve_2d(&NurbsCurve::create(
                false,
                1,
                &[corners[i].clone(), corners[j].clone()],
            ));
            b.add_pcurve(ei, si, c2 as i32, -1);
            refs.push(BRepRef::new(ei as i32, BRepOrientation::Forward));
        }

        let wi = b.add_wire(&refs);
        let fi = b.add_face(
            si as i32,
            &[BRepRef::new(wi as i32, BRepOrientation::Forward)],
            0.0,
        );
        let m = b.mesh();

        MINI_CHECK!(b.is_valid());
        MINI_CHECK!(fi == 0);
        MINI_CHECK!(b.face_count() == 1);
        MINI_CHECK!(b.wire_count() == 1);
        MINI_CHECK!(b.edge_count() == 4);
        MINI_CHECK!(b.vertex_count() == 4);
        MINI_CHECK!(b.m_edges[0].pcurves.len() == 1);
        MINI_CHECK!(b.pcurve_index(0, 0, BRepOrientation::Forward) == 0);
        MINI_CHECK!(!b.is_solid());
        MINI_CHECK!(!m.is_empty());
    })
}

pub fn run_brep_mesh() -> TestResult {
    MINI_TEST!("Mesh", {
        use crate::BRep;

        let b = BRep::create_box(2.0, 3.0, 4.0);
        let m = b.mesh();
        let fm = b.face_meshes();

        MINI_CHECK!(!m.is_empty());
        MINI_CHECK!(m.number_of_vertices() > 0);
        MINI_CHECK!(m.number_of_faces() > 0);
        MINI_CHECK!(fm.len() == 6);
        MINI_CHECK!(!fm[0].is_empty());
    })
}

pub fn run_brep_point_at() -> TestResult {
    MINI_TEST!("Point At", {
        use crate::BRep;

        let b = BRep::create_box(2.0, 3.0, 4.0);
        let pt = b.point_at(0, 0.5, 0.5);
        let n = b.normal_at(0, 0.5, 0.5);
        let n_top = b.normal_at(1, 0.5, 0.5);

        MINI_CHECK!((pt[2] + 2.0).abs() < 1e-9);
        MINI_CHECK!(pt[0].abs() < 1e-9);
        MINI_CHECK!(pt[1].abs() < 1e-9);
        MINI_CHECK!(n[2] < -0.99);
        MINI_CHECK!(n_top[2] > 0.99);
    })
}

pub fn run_brep_is_solid() -> TestResult {
    MINI_TEST!("Is Solid", {
        use crate::BRep;
        use crate::Point;
        use crate::Polyline;

        let bx = BRep::create_box(2.0, 3.0, 4.0);
        let cyl = BRep::create_cylinder(1.0, 2.0);
        let sph = BRep::create_sphere(1.0);
        let cone = BRep::create_cone(1.0, 2.0);
        let pyr = BRep::create_pyramid(2.0, 1.0);
        let tor = BRep::create_torus(2.0, 0.5);
        let blk = BRep::create_block_with_hole(4.0, 4.0, 2.0, 1.0);

        let quad = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let sheet = BRep::from_polylines(&[quad], &[]);

        MINI_CHECK!(bx.is_solid() && edges_manifold(&bx));
        MINI_CHECK!(cyl.is_solid() && edges_manifold(&cyl));
        MINI_CHECK!(sph.is_solid() && edges_manifold(&sph));
        MINI_CHECK!(cone.is_solid() && edges_manifold(&cone));
        MINI_CHECK!(pyr.is_solid() && edges_manifold(&pyr));
        MINI_CHECK!(tor.is_solid() && edges_manifold(&tor));
        MINI_CHECK!(blk.is_solid() && edges_manifold(&blk));
        MINI_CHECK!(!sheet.is_solid());
        MINI_CHECK!(sheet.solid_count() == 0);
    })
}

pub fn run_brep_is_closed() -> TestResult {
    MINI_TEST!("Is Closed", {
        use crate::BRep;

        let b = BRep::create_box(2.0, 3.0, 4.0);
        let mut open = b.duplicate();
        open.m_shells[0].faces.pop();

        MINI_CHECK!(b.is_closed(0));
        MINI_CHECK!(!b.is_closed(1));
        MINI_CHECK!(!open.is_closed(0));
        MINI_CHECK!(!open.is_solid());
    })
}

pub fn run_brep_wire_edges() -> TestResult {
    MINI_TEST!("Wire Edges", {
        use crate::brep::brep_compose;
        use crate::brep::brep_reverse;
        use crate::brep::BRepOrientation;
        use crate::brep::BRepRef;
        use crate::BRep;

        let b = BRep::create_box(2.0, 3.0, 4.0);
        let fwd = BRepRef::new(0, BRepOrientation::Forward);
        let rev = BRepRef::new(0, BRepOrientation::Reversed);
        let a = b.wire_edges(&fwd);
        let c = b.wire_edges(&rev);

        MINI_CHECK!(a.len() == 4);
        MINI_CHECK!(c.len() == 4);
        MINI_CHECK!(a[0].index == c[3].index);
        MINI_CHECK!(a[0].orientation == brep_reverse(c[3].orientation));
        MINI_CHECK!(
            brep_compose(BRepOrientation::Reversed, BRepOrientation::Reversed)
                == BRepOrientation::Forward
        );
        MINI_CHECK!(
            brep_compose(BRepOrientation::Forward, BRepOrientation::Reversed)
                == BRepOrientation::Reversed
        );
        MINI_CHECK!(
            brep_compose(BRepOrientation::Internal, BRepOrientation::Reversed)
                == BRepOrientation::Internal
        );
    })
}

pub fn run_brep_edge_faces() -> TestResult {
    MINI_TEST!("Edge Faces", {
        use crate::brep::BRepOrientation;
        use crate::BRep;

        let cyl = BRep::create_cylinder(1.0, 2.0);
        let bot = cyl.edge_faces(0);
        let seam = cyl.edge_faces(2);
        let pc_f = cyl.pcurve_index(2, 0, BRepOrientation::Forward);
        let pc_r = cyl.pcurve_index(2, 0, BRepOrientation::Reversed);

        MINI_CHECK!(bot.len() == 2);
        MINI_CHECK!(bot[0].index == 0 && bot[1].index == 1);
        MINI_CHECK!(bot[0].orientation != bot[1].orientation);
        MINI_CHECK!(seam.len() == 2);
        MINI_CHECK!(seam[0].index == 0 && seam[1].index == 0);
        MINI_CHECK!(pc_f >= 0 && pc_r >= 0 && pc_f != pc_r);
        MINI_CHECK!(cyl.pcurve_index(2, 1, BRepOrientation::Forward) == -1);
        MINI_CHECK!(cyl.face_orientation(0) == BRepOrientation::Forward);
    })
}

pub fn run_brep_update_tolerances() -> TestResult {
    MINI_TEST!("Update Tolerances", {
        use crate::BRep;
        use crate::Point;

        let mut b = BRep::create_box(2.0, 3.0, 4.0);
        let worst = b.update_tolerances();
        let mut bent = b.duplicate();
        bent.m_vertices[0].point = Point::new(-1.0, -1.5, -2.01);
        let worst_bent = bent.update_tolerances();
        let mut worst_prims: f64 = 0.0;

        for mut p in [
            BRep::create_cylinder(1.0, 2.0),
            BRep::create_sphere(1.0),
            BRep::create_cone(1.0, 2.0),
            BRep::create_pyramid(2.0, 1.0),
            BRep::create_torus(2.0, 0.5),
            BRep::create_block_with_hole(4.0, 4.0, 2.0, 1.0),
        ] {
            worst_prims = worst_prims.max(p.update_tolerances());
        }

        MINI_CHECK!(worst < 1e-9);
        MINI_CHECK!(b.m_edges[0].tolerance < 1e-9);
        MINI_CHECK!((worst_bent - 0.01).abs() < 1e-9);
        MINI_CHECK!((bent.m_vertices[0].tolerance - 0.01).abs() < 1e-9);
        MINI_CHECK!(bent.m_vertices[6].tolerance < 1e-9);
        MINI_CHECK!(worst_prims < 1e-6);
    })
}

pub fn run_brep_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::BRep;
        use crate::Xform;

        let b = BRep::create_box(2.0, 3.0, 4.0);
        let box_xf = Xform::translation(10.0, 20.0, 30.0);
        let moved = b.transformed(&box_xf);

        let pt = moved.point_at(0, 0.0, 0.0);
        let pt_orig = b.point_at(0, 0.0, 0.0);

        MINI_CHECK!((pt[0] - pt_orig[0] - 10.0).abs() < 0.01);
        MINI_CHECK!((pt[1] - pt_orig[1] - 20.0).abs() < 0.01);
        MINI_CHECK!((pt[2] - pt_orig[2] - 30.0).abs() < 0.01);
        MINI_CHECK!((moved.m_vertices[0].point[0] - b.m_vertices[0].point[0] - 10.0).abs() < 0.01);
    })
}

pub fn run_brep_transform_roundtrip() -> TestResult {
    MINI_TEST!("Transform Roundtrip", {
        use crate::BRep;
        use crate::Vector;
        use crate::Xform;

        let axis = Vector::new(0.3, 0.5, 0.81);
        let rot = Xform::rotation(&axis, 37.0, true);
        let tr = Xform::translation(10.0, -5.0, 3.0);
        let b = BRep::create_box(2.0, 3.0, 4.0);
        let moved = b.transformed(&rot).transformed(&tr);

        let mut matched = true;

        for i in 0..b.m_vertices.len() {
            let expect = tr.transform_point(&rot.transform_point(&b.m_vertices[i].point));

            if moved.m_vertices[i].point.distance(&expect, None) > 1e-9 {
                matched = false;
            }
        }

        let mut back = moved
            .transformed(&tr.inverse().unwrap())
            .transformed(&rot.inverse().unwrap());

        let mut restored = true;

        for i in 0..b.m_vertices.len() {
            if back.m_vertices[i]
                .point
                .distance(&b.m_vertices[i].point, None)
                > 1e-9
            {
                restored = false;
            }
        }

        MINI_CHECK!(matched);
        MINI_CHECK!(restored);
        MINI_CHECK!(back.is_solid());
        MINI_CHECK!(back.update_tolerances() < 1e-9);
    })
}

pub fn run_brep_cut_by_plane() -> TestResult {
    MINI_TEST!("Cut By Plane", {
        use crate::BRep;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;
        use crate::Xform;

        let b = BRep::create_box(2.0, 2.0, 2.0);
        let half = b.cut_by_plane(&Plane::from_point_normal(
            Point::new(0.0, 0.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            None,
        ));

        let far = Xform::translation(100000.0, 200000.0, 30000.0)
            * Xform::rotation(&Vector::new(1.0, 2.0, 3.0), 40.0, true);
        let beam = BRep::create_box(200.0, 100.0, 600.0).transformed(&far);
        let piece = beam.cut_by_plane(&Plane::from_point_normal(
            far.transform_point(&Point::new(0.0, 0.0, 0.0)),
            far.transform_vector(&Vector::new(0.0, 0.0, 1.0)),
            None,
        ));

        MINI_CHECK!(half.is_solid());
        MINI_CHECK!(half.vertex_count() == 8);
        MINI_CHECK!(half.edge_count() == 12);
        MINI_CHECK!(half.face_count() == 6);
        MINI_CHECK!((half.volume() - 4.0).abs() < 1e-9);
        MINI_CHECK!(piece.is_solid());
        MINI_CHECK!(piece.face_count() == 6);
        MINI_CHECK!((piece.volume() - 6000000.0).abs() < 0.01);
    })
}

pub fn run_brep_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::brep::BRepOrientation;
        use crate::BRep;
        use crate::Color;
        use std::path::PathBuf;

        let mut b = BRep::create_cylinder(1.0, 2.0);
        b.name = "test_brep".to_string();
        b.width = 2.0;
        b.surfacecolor = Color::new(1.0, 0.5, 0.25, 1.0);

        let json = b.jsondump().unwrap();
        let loaded_json = BRep::jsonload(&json).unwrap();

        let json_string = b.file_json_dumps();
        let loaded_json_string = BRep::file_json_loads(&json_string);

        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let filename = src_dir.join("serialization").join("test_brep.json");
        b.file_json_dump(filename.to_str().unwrap()).unwrap();
        let loaded_from_file = BRep::file_json_load(filename.to_str().unwrap()).unwrap();

        MINI_CHECK!(loaded_json == b);
        MINI_CHECK!(loaded_json_string == b);
        MINI_CHECK!(loaded_from_file == b);
        MINI_CHECK!(loaded_from_file.is_solid());
        MINI_CHECK!(loaded_from_file.m_edges[2].pcurves[0].curve_2d_index_2 >= 0);
        MINI_CHECK!(loaded_from_file.m_wires[0].edges[2].orientation == BRepOrientation::Reversed);
    })
}

pub fn run_brep_create_cylinder() -> TestResult {
    MINI_TEST!("Create Cylinder", {
        use crate::BRep;

        let cyl = BRep::create_cylinder(1.0, 2.0);
        let m = cyl.mesh();

        MINI_CHECK!(cyl.is_valid());
        MINI_CHECK!(cyl.face_count() == 3);
        MINI_CHECK!(cyl.edge_count() == 3);
        MINI_CHECK!(cyl.vertex_count() == 2);
        MINI_CHECK!(cyl.is_solid());
        MINI_CHECK!(cyl.name == "cylinder");
        MINI_CHECK!(!m.is_empty());
    })
}

pub fn run_brep_create_sphere() -> TestResult {
    MINI_TEST!("Create Sphere", {
        use crate::BRep;

        let sph = BRep::create_sphere(1.0);
        let m = sph.mesh();

        MINI_CHECK!(sph.is_valid());
        MINI_CHECK!(sph.face_count() == 1);
        MINI_CHECK!(sph.edge_count() == 3);
        MINI_CHECK!(sph.vertex_count() == 2);
        MINI_CHECK!(sph.m_edges[1].degenerated && sph.m_edges[2].degenerated);
        MINI_CHECK!(sph.is_solid());
        MINI_CHECK!(sph.name == "sphere");
        MINI_CHECK!(!m.is_empty());
    })
}

pub fn run_brep_create_cone() -> TestResult {
    MINI_TEST!("Create Cone", {
        use crate::BRep;

        let cone = BRep::create_cone(1.0, 2.0);
        let m = cone.mesh();

        MINI_CHECK!(cone.is_valid());
        MINI_CHECK!(cone.face_count() == 2);
        MINI_CHECK!(cone.edge_count() == 3);
        MINI_CHECK!(cone.vertex_count() == 2);
        MINI_CHECK!(cone.is_solid());
        MINI_CHECK!(cone.name == "cone");
        MINI_CHECK!(!m.is_empty());
    })
}

pub fn run_brep_create_pyramid() -> TestResult {
    MINI_TEST!("Create Pyramid", {
        use crate::BRep;

        let pyr = BRep::create_pyramid(2.0, 1.0);
        let m = pyr.mesh();

        MINI_CHECK!(pyr.is_valid());
        MINI_CHECK!(pyr.face_count() == 5);
        MINI_CHECK!(pyr.edge_count() == 12);
        MINI_CHECK!(pyr.vertex_count() == 5);
        MINI_CHECK!(pyr.is_solid());
        MINI_CHECK!(pyr.name == "pyramid");
        MINI_CHECK!(!m.is_empty());
    })
}

pub fn run_brep_create_torus() -> TestResult {
    MINI_TEST!("Create Torus", {
        use crate::BRep;

        let tor = BRep::create_torus(2.0, 0.5);
        let m = tor.mesh();

        MINI_CHECK!(tor.is_valid());
        MINI_CHECK!(tor.face_count() == 1);
        MINI_CHECK!(tor.edge_count() == 2);
        MINI_CHECK!(tor.vertex_count() == 1);
        MINI_CHECK!(tor.is_solid());
        MINI_CHECK!(tor.name == "torus");
        MINI_CHECK!(!m.is_empty());
    })
}

pub fn run_brep_create_block_with_hole() -> TestResult {
    MINI_TEST!("Create Block With Hole", {
        use crate::brep::BRepOrientation;
        use crate::BRep;

        let bh = BRep::create_block_with_hole(8.0, 6.0, 4.0, 1.5);
        let m = bh.mesh();

        MINI_CHECK!(bh.is_valid());
        MINI_CHECK!(bh.face_count() == 7);
        MINI_CHECK!(bh.edge_count() == 15);
        MINI_CHECK!(bh.vertex_count() == 10);
        MINI_CHECK!(bh.m_faces[6].wires.len() == 2);
        MINI_CHECK!(bh.face_orientation(4) == BRepOrientation::Reversed);
        MINI_CHECK!(bh.is_solid());
        MINI_CHECK!(bh.name == "block_with_hole");
        MINI_CHECK!(!m.is_empty());
    })
}

pub fn run_brep_from_polylines() -> TestResult {
    MINI_TEST!("From Polylines", {
        use crate::BRep;
        use crate::Point;
        use crate::Polyline;

        let hx = 1.0;
        let hy = 1.5;
        let hz = 2.0;
        let c = [
            Point::new(-hx, -hy, -hz),
            Point::new(hx, -hy, -hz),
            Point::new(hx, hy, -hz),
            Point::new(-hx, hy, -hz),
            Point::new(-hx, -hy, hz),
            Point::new(hx, -hy, hz),
            Point::new(hx, hy, hz),
            Point::new(-hx, hy, hz),
        ];

        let bottom = Polyline::new(vec![
            c[0].clone(),
            c[3].clone(),
            c[2].clone(),
            c[1].clone(),
            c[0].clone(),
        ]);
        let top = Polyline::new(vec![
            c[4].clone(),
            c[5].clone(),
            c[6].clone(),
            c[7].clone(),
            c[4].clone(),
        ]);
        let front = Polyline::new(vec![
            c[0].clone(),
            c[1].clone(),
            c[5].clone(),
            c[4].clone(),
            c[0].clone(),
        ]);
        let right = Polyline::new(vec![
            c[1].clone(),
            c[2].clone(),
            c[6].clone(),
            c[5].clone(),
            c[1].clone(),
        ]);
        let back = Polyline::new(vec![
            c[2].clone(),
            c[3].clone(),
            c[7].clone(),
            c[6].clone(),
            c[2].clone(),
        ]);
        let left = Polyline::new(vec![
            c[3].clone(),
            c[0].clone(),
            c[4].clone(),
            c[7].clone(),
            c[3].clone(),
        ]);

        let b = BRep::from_polylines(&[bottom, top, front, right, back, left], &[]);
        let m = b.mesh();

        MINI_CHECK!(b.is_valid());
        MINI_CHECK!(b.face_count() == 6);
        MINI_CHECK!(b.edge_count() == 12);
        MINI_CHECK!(b.vertex_count() == 8);
        MINI_CHECK!(b.shell_count() == 1);
        MINI_CHECK!(b.is_solid() && edges_manifold(&b));
        MINI_CHECK!((b.volume() - 24.0).abs() < 1e-6);
        MINI_CHECK!(!m.is_empty());
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

pub fn run_brep_from_nurbscurves() -> TestResult {
    MINI_TEST!("From Nurbscurves", {
        use crate::BRep;
        use crate::NurbsCurve;
        use crate::Point;

        let hx = 1.0;
        let hy = 1.5;
        let hz = 2.0;
        let c = [
            Point::new(-hx, -hy, -hz),
            Point::new(hx, -hy, -hz),
            Point::new(hx, hy, -hz),
            Point::new(-hx, hy, -hz),
            Point::new(-hx, -hy, hz),
            Point::new(hx, -hy, hz),
            Point::new(hx, hy, hz),
            Point::new(-hx, hy, hz),
        ];

        let bottom = NurbsCurve::create(
            false,
            1,
            &[
                c[0].clone(),
                c[3].clone(),
                c[2].clone(),
                c[1].clone(),
                c[0].clone(),
            ],
        );
        let top = NurbsCurve::create(
            false,
            1,
            &[
                c[4].clone(),
                c[5].clone(),
                c[6].clone(),
                c[7].clone(),
                c[4].clone(),
            ],
        );
        let front = NurbsCurve::create(
            false,
            1,
            &[
                c[0].clone(),
                c[1].clone(),
                c[5].clone(),
                c[4].clone(),
                c[0].clone(),
            ],
        );
        let right = NurbsCurve::create(
            false,
            1,
            &[
                c[1].clone(),
                c[2].clone(),
                c[6].clone(),
                c[5].clone(),
                c[1].clone(),
            ],
        );
        let back = NurbsCurve::create(
            false,
            1,
            &[
                c[2].clone(),
                c[3].clone(),
                c[7].clone(),
                c[6].clone(),
                c[2].clone(),
            ],
        );
        let left = NurbsCurve::create(
            false,
            1,
            &[
                c[3].clone(),
                c[0].clone(),
                c[4].clone(),
                c[7].clone(),
                c[3].clone(),
            ],
        );

        let b = BRep::from_nurbscurves(&[bottom, top, front, right, back, left], &[]);
        let m = b.mesh();

        MINI_CHECK!(b.is_valid());
        MINI_CHECK!(b.face_count() == 6);
        MINI_CHECK!(b.edge_count() == 6);
        MINI_CHECK!(b.vertex_count() == 5);
        MINI_CHECK!(!b.is_solid());
        MINI_CHECK!(!m.is_empty());
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

pub fn run_brep_from_nurbscurves_holes() -> TestResult {
    MINI_TEST!("From Nurbscurves Holes", {
        use crate::primitives::Primitives;
        use crate::BRep;
        use crate::NurbsCurve;
        use crate::Point;

        let outer = NurbsCurve::create(
            false,
            1,
            &[
                Point::new(-5.0, -5.0, 0.0),
                Point::new(5.0, -5.0, 0.0),
                Point::new(5.0, 5.0, 0.0),
                Point::new(-5.0, 5.0, 0.0),
                Point::new(-5.0, -5.0, 0.0),
            ],
        );
        let hole = Primitives::circle(0.0, 0.0, 0.0, 2.0);

        let b = BRep::from_nurbscurves(&[outer], &[vec![hole]]);
        let m = b.mesh();

        MINI_CHECK!(b.is_valid());
        MINI_CHECK!(b.face_count() == 1);
        MINI_CHECK!(b.wire_count() == 2);
        MINI_CHECK!(b.m_faces[0].wires.len() == 2);
        MINI_CHECK!(b.m_faces[0].wires[1].index == 1);
        MINI_CHECK!(!m.is_empty());
        MINI_CHECK!((m.area() - (100.0 - PI * 4.0)).abs() < 0.5);
    })
}

/// Closed square ring at height z through the (x, y) corners
fn ring(pts: &[(f64, f64)], z: f64) -> crate::Polyline {
    use crate::Point;
    use crate::Polyline;

    let mut v: Vec<Point> = Vec::new();

    for p in pts {
        v.push(Point::new(p.0, p.1, z));
    }

    v.push(Point::new(pts[0].0, pts[0].1, z));

    Polyline::new(v)
}

/// 4 x 4 x 2 box with a 1 x 1 through-hole along z: bottom and top with a hole, four outer and four inner side quads
fn box_with_square_hole() -> (Vec<crate::Polyline>, Vec<Vec<crate::Polyline>>) {
    use crate::Point;
    use crate::Polyline;

    let outer = [(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)];
    let inner = [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)];
    let mut faces = vec![ring(&outer, 0.0), ring(&outer, 2.0)];
    let mut holes = vec![vec![ring(&inner, 0.0)], vec![ring(&inner, 2.0)]];

    for pts in [outer, inner] {
        for i in 0..4 {
            let a = pts[i];
            let b = pts[(i + 1) % 4];
            faces.push(Polyline::new(vec![
                Point::new(a.0, a.1, 0.0),
                Point::new(b.0, b.1, 0.0),
                Point::new(b.0, b.1, 2.0),
                Point::new(a.0, a.1, 2.0),
                Point::new(a.0, a.1, 0.0),
            ]));
            holes.push(Vec::new());
        }
    }

    (faces, holes)
}

pub fn run_brep_from_polylines_holes() -> TestResult {
    MINI_TEST!("From Polylines Holes", {
        use crate::BRep;

        let (faces, holes) = box_with_square_hole();
        let b = BRep::from_polylines(&faces, &holes);

        MINI_CHECK!(b.face_count() == 10);
        MINI_CHECK!(b.m_faces[0].wires.len() == 2);
        MINI_CHECK!(b.is_solid());
        MINI_CHECK!((b.mesh().volume() - 30.0).abs() < 1e-6);
    })
}

pub fn run_brep_planar_fast_path() -> TestResult {
    MINI_TEST!("Planar Fast Path", {
        use crate::BRep;

        let (faces, holes) = box_with_square_hole();
        let b = BRep::from_polylines(&faces, &holes);
        let fm = b.face_meshes();
        let mut total = 0.0;

        for m in &fm {
            total += m.area();
        }

        let mut tagged = 0;

        for vd in fm[0].vertex.values() {
            for key in vd.attributes.keys() {
                if key.starts_with("brep_edge/") {
                    tagged += 1;
                    break;
                }
            }
        }

        MINI_CHECK!(fm.len() == 10);
        MINI_CHECK!(fm[0].vertex.len() == 8 && fm[0].face.len() == 8);
        MINI_CHECK!(fm[2].vertex.len() == 4 && fm[2].face.len() == 2);
        MINI_CHECK!((total - 70.0).abs() < 1e-6);
        MINI_CHECK!(tagged == 8);
    })
}

pub fn run_brep_mesh_orientation() -> TestResult {
    MINI_TEST!("Mesh Orientation", {
        use crate::BRep;

        let bh = BRep::create_block_with_hole(8.0, 6.0, 4.0, 1.5);
        let vol = bh.mesh().volume();
        let reference = 8.0 * 6.0 * 4.0 - PI * 1.5 * 1.5 * 4.0;

        MINI_CHECK!((vol - reference).abs() / reference < 0.02);
    })
}

pub fn run_brep_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::brep::BRepOrientation;
        use crate::BRep;
        use crate::Color;
        use std::path::PathBuf;

        let mut b = BRep::create_cylinder(1.0, 2.0);
        b.name = "test_brep".to_string();
        b.width = 2.0;
        b.surfacecolor = Color::new(1.0, 0.5, 0.25, 1.0);

        let loaded_proto = BRep::from_proto(b.to_proto()).unwrap();

        let proto_bytes = b.pb_dumps();
        let loaded_proto_string = BRep::pb_loads(&proto_bytes).unwrap();

        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let filename = src_dir.join("serialization").join("test_brep.bin");
        b.pb_dump(filename.to_str().unwrap());
        let loaded = BRep::pb_load(filename.to_str().unwrap());

        MINI_CHECK!(loaded_proto == b);
        MINI_CHECK!(loaded_proto_string == b);
        MINI_CHECK!(loaded == b);
        MINI_CHECK!(loaded.is_solid());
        MINI_CHECK!(loaded.m_edges[2].pcurves[0].curve_2d_index_2 >= 0);
        MINI_CHECK!(loaded.m_wires[0].edges[2].orientation == BRepOrientation::Reversed);
    })
}

pub fn run_brep_volume() -> TestResult {
    MINI_TEST!("Volume", {
        use crate::BRep;

        let bx = BRep::create_box(2.0, 3.0, 4.0);
        let cyl = BRep::create_cylinder(1.0, 4.0);
        let sph = BRep::create_sphere(2.0);
        let vbox = bx.volume();
        let vcyl = cyl.volume();
        let vsph = sph.volume();

        MINI_CHECK!((vbox - 24.0).abs() < 1e-9);
        MINI_CHECK!((vcyl - 4.0 * PI).abs() / (4.0 * PI) < 0.05);
        MINI_CHECK!((vsph - (4.0 / 3.0) * PI * 8.0).abs() / ((4.0 / 3.0) * PI * 8.0) < 0.05);
    })
}

pub fn run_brep_face_polylines_box() -> TestResult {
    MINI_TEST!("Face Polylines Box", {
        use crate::BRep;

        let b = BRep::create_box(2.0, 2.0, 2.0);
        let pls = b.face_polylines();
        let pls_planes = b.face_planes();

        MINI_CHECK!(pls.len() == 6);
        MINI_CHECK!(pls_planes.len() == pls.len());

        let mut seen = [[false, false], [false, false], [false, false]];

        for fi in 0..pls.len() {
            let p = &pls[fi];
            let pl = &pls_planes[fi];
            MINI_CHECK!(p.point_count() == 5);
            MINI_CHECK!(p.get_point(0) == p.get_point(4));

            let mut axis = 3;
            let mut sign = 0.0;

            for a in 0..3 {
                let mut constant = true;

                for i in 1..p.point_count() {
                    if (p[i][a] - p[0][a]).abs() > 1e-9 {
                        constant = false;
                        break;
                    }
                }

                if constant && (p[0][a].abs() - 1.0).abs() < 1e-9 {
                    axis = a;
                    sign = if p[0][a] > 0.0 { 1.0 } else { -1.0 };
                    break;
                }
            }

            MINI_CHECK!(axis < 3);
            MINI_CHECK!((pl.origin()[axis] - sign).abs() < 1e-9);

            let n = pl.z_axis();
            MINI_CHECK!((n[axis].abs() - 1.0).abs() < 1e-6);

            for a2 in 0..3 {
                if a2 != axis {
                    MINI_CHECK!(n[a2].abs() < 1e-6);
                }
            }

            let normal_sign = if n[axis] > 0.0 { 1 } else { 0 };
            MINI_CHECK!(!seen[axis][normal_sign]);
            seen[axis][normal_sign] = true;
        }

        for axis_seen in &seen {
            for sign_seen in axis_seen {
                MINI_CHECK!(*sign_seen);
            }
        }
    })
}

pub fn run_brep_face_polylines_cylinder_caps_only() -> TestResult {
    MINI_TEST!("Face Polylines Cylinder Caps Only", {
        use crate::BRep;

        let b = BRep::create_cylinder(1.0, 4.0);

        MINI_CHECK!(b.face_count() == 3);
        MINI_CHECK!(b.face_polylines().len() == 2);
        MINI_CHECK!(b.face_planes().len() == 2);
    })
}

pub fn run_brep_face_polylines_ignores_holes() -> TestResult {
    MINI_TEST!("Face Polylines Ignores Holes", {
        use crate::BRep;

        let b = BRep::create_block_with_hole(4.0, 4.0, 2.0, 1.0);
        let pls = b.face_polylines();

        MINI_CHECK!(pls.len() == 6);
        MINI_CHECK!(pls.len() == b.face_planes().len());

        for p in &pls {
            MINI_CHECK!(p.point_count() == 5);

            for i in 0..p.point_count() {
                let pt = &p[i];
                let on_bounds = (pt[0].abs() - 2.0).abs() < 1e-6
                    || (pt[1].abs() - 2.0).abs() < 1e-6
                    || (pt[2].abs() - 1.0).abs() < 1e-6;
                MINI_CHECK!(on_bounds);
                let radius_to_z_axis = (pt[0] * pt[0] + pt[1] * pt[1]).sqrt();
                MINI_CHECK!(radius_to_z_axis > 1.0 + 1e-6);
            }
        }
    })
}

pub fn run_brep_face_polylines_no_planar_faces() -> TestResult {
    MINI_TEST!("Face Polylines No Planar Faces", {
        use crate::BRep;

        let b = BRep::create_sphere(1.0);

        MINI_CHECK!(b.face_polylines().is_empty());
        MINI_CHECK!(b.face_planes().is_empty());
    })
}

pub fn run_brep_face_planes_reversed_flip() -> TestResult {
    MINI_TEST!("Face Planes Reversed Flip", {
        use crate::brep::BRepOrientation;
        use crate::brep::BRepRef;
        use crate::BRep;

        let mut b = BRep::new();
        let fi_forward = build_quad_face(&mut b);
        b.add_shell(&[BRepRef::new(fi_forward as i32, BRepOrientation::Forward)]);
        let fi_reversed = build_quad_face(&mut b);
        b.add_shell(&[BRepRef::new(fi_reversed as i32, BRepOrientation::Reversed)]);

        MINI_CHECK!(b.face_orientation(fi_forward) == BRepOrientation::Forward);
        MINI_CHECK!(b.face_orientation(fi_reversed) == BRepOrientation::Reversed);

        let planes = b.face_planes();
        MINI_CHECK!(planes.len() == 2);

        let n_forward = planes[0].z_axis();
        let n_reversed = planes[1].z_axis();
        MINI_CHECK!((n_forward[0] + n_reversed[0]).abs() < 1e-9);
        MINI_CHECK!((n_forward[1] + n_reversed[1]).abs() < 1e-9);
        MINI_CHECK!((n_forward[2] + n_reversed[2]).abs() < 1e-9);
    })
}

pub fn run_brep_face_planes_point_outward() -> TestResult {
    MINI_TEST!("Face Planes Point Outward", {
        use crate::BRep;

        let bx = BRep::create_box(2.0, 2.0, 2.0);
        let box_planes = bx.face_planes();
        MINI_CHECK!(box_planes.len() == 6);

        for pl in &box_planes {
            let o = pl.origin();
            let n = pl.z_axis();
            let d = o[0] * n[0] + o[1] * n[1] + o[2] * n[2];
            MINI_CHECK!(d > 0.0);
        }

        let cyl = BRep::create_cylinder(1.0, 4.0);
        let cyl_planes = cyl.face_planes();
        MINI_CHECK!(cyl_planes.len() == 2);

        for pl in &cyl_planes {
            let o = pl.origin();
            let n = pl.z_axis();
            let mid_z = 2.0;
            MINI_CHECK!((o[2] - mid_z) * n[2] > 0.0);
        }
    })
}

pub fn run_brep_face_planes_outward_block_with_hole() -> TestResult {
    MINI_TEST!("Face Planes Outward Block With Hole", {
        use crate::BRep;
        use crate::Point;

        let b = BRep::create_block_with_hole(4.0, 4.0, 2.0, 1.0);
        MINI_CHECK!(b.is_solid());

        let solid_centroid = Point::centroid(&b.vertex_points());
        let planes = b.face_planes();
        MINI_CHECK!(planes.len() == 6);

        for pl in &planes {
            let o = pl.origin();
            let n = pl.z_axis();
            let d = (o[0] - solid_centroid[0]) * n[0]
                + (o[1] - solid_centroid[1]) * n[1]
                + (o[2] - solid_centroid[2]) * n[2];
            MINI_CHECK!(d > 0.0);
        }
    })
}

pub fn run_brep_face_planes_outward_under_mirrored_winding() -> TestResult {
    MINI_TEST!("Face Planes Outward Under Mirrored Winding", {
        use crate::brep::BRepOrientation;
        use crate::BRep;
        use crate::Point;
        use crate::Xform;

        let mirror = Xform::scale_xyz(-1.0, 1.0, 1.0);
        let b = BRep::create_box(2.0, 2.0, 2.0).transformed(&mirror);
        MINI_CHECK!(b.is_solid());

        let pls = b.face_polylines();
        let planes = b.face_planes();
        MINI_CHECK!(pls.len() == 6);
        MINI_CHECK!(planes.len() == 6);

        let solid_centroid = Point::centroid(&b.vertex_points());

        for fi in 0..pls.len() {
            let o = planes[fi].origin();
            let n = planes[fi].z_axis();
            let d = (o[0] - solid_centroid[0]) * n[0]
                + (o[1] - solid_centroid[1]) * n[1]
                + (o[2] - solid_centroid[2]) * n[2];
            MINI_CHECK!(d > 0.0);

            let mut expected = Vec::new();

            for er in b.wire_edges(&b.m_faces[fi].wires[0]) {
                let edge = &b.m_edges[er.index as usize];
                let reversed = er.orientation == BRepOrientation::Reversed;
                let start = if reversed {
                    edge.end_vertex
                } else {
                    edge.start_vertex
                };
                expected.push(b.m_vertices[start as usize].point.clone());
            }

            expected.push(expected[0].clone());

            let actual = pls[fi].get_points();
            MINI_CHECK!(actual.len() == expected.len());

            for k in 0..actual.len() {
                MINI_CHECK!(actual[k] == expected[k]);
            }
        }
    })
}

REGISTER_MINI_TEST!(
    "BRep",
    "Shared Grid Boundary",
    crate::brep_test::run_brep_shared_grid_boundary
);
REGISTER_MINI_TEST!(
    "BRep",
    "Constructor",
    crate::brep_test::run_brep_constructor
);
REGISTER_MINI_TEST!("BRep", "Create Box", crate::brep_test::run_brep_create_box);
REGISTER_MINI_TEST!("BRep", "Accessors", crate::brep_test::run_brep_accessors);
REGISTER_MINI_TEST!("BRep", "Add Face", crate::brep_test::run_brep_add_face);
REGISTER_MINI_TEST!("BRep", "Mesh", crate::brep_test::run_brep_mesh);
REGISTER_MINI_TEST!("BRep", "Point At", crate::brep_test::run_brep_point_at);
REGISTER_MINI_TEST!("BRep", "Is Solid", crate::brep_test::run_brep_is_solid);
REGISTER_MINI_TEST!("BRep", "Is Closed", crate::brep_test::run_brep_is_closed);
REGISTER_MINI_TEST!("BRep", "Wire Edges", crate::brep_test::run_brep_wire_edges);
REGISTER_MINI_TEST!("BRep", "Edge Faces", crate::brep_test::run_brep_edge_faces);
REGISTER_MINI_TEST!(
    "BRep",
    "Update Tolerances",
    crate::brep_test::run_brep_update_tolerances
);
REGISTER_MINI_TEST!(
    "BRep",
    "Transformation",
    crate::brep_test::run_brep_transformation
);
REGISTER_MINI_TEST!(
    "BRep",
    "Transform Roundtrip",
    crate::brep_test::run_brep_transform_roundtrip
);
REGISTER_MINI_TEST!(
    "BRep",
    "Cut By Plane",
    crate::brep_test::run_brep_cut_by_plane
);
REGISTER_MINI_TEST!(
    "BRep",
    "Json Roundtrip",
    crate::brep_test::run_brep_json_roundtrip
);
REGISTER_MINI_TEST!(
    "BRep",
    "Create Cylinder",
    crate::brep_test::run_brep_create_cylinder
);
REGISTER_MINI_TEST!(
    "BRep",
    "Create Sphere",
    crate::brep_test::run_brep_create_sphere
);
REGISTER_MINI_TEST!(
    "BRep",
    "Create Cone",
    crate::brep_test::run_brep_create_cone
);
REGISTER_MINI_TEST!(
    "BRep",
    "Create Pyramid",
    crate::brep_test::run_brep_create_pyramid
);
REGISTER_MINI_TEST!(
    "BRep",
    "Create Torus",
    crate::brep_test::run_brep_create_torus
);
REGISTER_MINI_TEST!(
    "BRep",
    "Create Block With Hole",
    crate::brep_test::run_brep_create_block_with_hole
);
REGISTER_MINI_TEST!(
    "BRep",
    "From Polylines",
    crate::brep_test::run_brep_from_polylines
);
REGISTER_MINI_TEST!(
    "BRep",
    "From Nurbscurves",
    crate::brep_test::run_brep_from_nurbscurves
);
REGISTER_MINI_TEST!(
    "BRep",
    "From Nurbscurves Holes",
    crate::brep_test::run_brep_from_nurbscurves_holes
);
REGISTER_MINI_TEST!(
    "BRep",
    "From Polylines Holes",
    crate::brep_test::run_brep_from_polylines_holes
);
REGISTER_MINI_TEST!(
    "BRep",
    "Planar Fast Path",
    crate::brep_test::run_brep_planar_fast_path
);
REGISTER_MINI_TEST!(
    "BRep",
    "Mesh Orientation",
    crate::brep_test::run_brep_mesh_orientation
);
REGISTER_MINI_TEST!(
    "BRep",
    "Protobuf Roundtrip",
    crate::brep_test::run_brep_protobuf_roundtrip
);
REGISTER_MINI_TEST!("BRep", "Volume", crate::brep_test::run_brep_volume);
REGISTER_MINI_TEST!(
    "BRep",
    "Face Polylines Box",
    crate::brep_test::run_brep_face_polylines_box
);
REGISTER_MINI_TEST!(
    "BRep",
    "Face Polylines Cylinder Caps Only",
    crate::brep_test::run_brep_face_polylines_cylinder_caps_only
);
REGISTER_MINI_TEST!(
    "BRep",
    "Face Polylines Ignores Holes",
    crate::brep_test::run_brep_face_polylines_ignores_holes
);
REGISTER_MINI_TEST!(
    "BRep",
    "Face Polylines No Planar Faces",
    crate::brep_test::run_brep_face_polylines_no_planar_faces
);
REGISTER_MINI_TEST!(
    "BRep",
    "Face Planes Reversed Flip",
    crate::brep_test::run_brep_face_planes_reversed_flip
);
REGISTER_MINI_TEST!(
    "BRep",
    "Face Planes Point Outward",
    crate::brep_test::run_brep_face_planes_point_outward
);
REGISTER_MINI_TEST!(
    "BRep",
    "Face Planes Outward Block With Hole",
    crate::brep_test::run_brep_face_planes_outward_block_with_hole
);
REGISTER_MINI_TEST!(
    "BRep",
    "Face Planes Outward Under Mirrored Winding",
    crate::brep_test::run_brep_face_planes_outward_under_mirrored_winding
);
