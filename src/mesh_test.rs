use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::{TOLERANCE, PI};

pub fn run_mesh_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Mesh;
        use crate::Polyline;
        use crate::Color;
        use crate::mesh::ColorMode;

        let vertices = Polyline::from_sides(6, 1.0, false).get_points();
        let mut mesh = Mesh::from_vertices_and_faces(vertices, vec![vec![0, 1, 2, 3, 4, 5]]);
        let _sstr = mesh.str();
        let _srepr = mesh.repr();
        let _mcopy = mesh.duplicate();
        MINI_CHECK!(mesh.is_valid());
        mesh.name = "hexagon".to_string();

        let palette = Color::palette();

        // set_objectcolor does not change color_mode
        mesh.set_objectcolor(Color::grey());
        MINI_CHECK!(mesh.color_mode == ColorMode::OBJECTCOLOR);

        // set_pointcolors → color_mode = PointColors
        let mut pc: Vec<Color> = Vec::new();
        pc.reserve(mesh.number_of_vertices());
        for i in 0..mesh.number_of_vertices() {
            pc.push(palette[i % palette.len()].clone());
        }
        mesh.set_pointcolors(pc);
        MINI_CHECK!(mesh.color_mode == ColorMode::POINTCOLORS);
        MINI_CHECK!(mesh.get_pointcolors().len() == mesh.number_of_vertices());

        // set_facecolors → color_mode = FaceColors
        let mut fc: Vec<Color> = Vec::new();
        fc.reserve(mesh.number_of_faces());
        for i in 0..mesh.number_of_faces() {
            fc.push(palette[i % palette.len()].clone());
        }
        mesh.set_facecolors(fc);
        MINI_CHECK!(mesh.color_mode == ColorMode::FACECOLORS);
        MINI_CHECK!(mesh.get_facecolors().len() == mesh.number_of_faces());

        // set_linecolors does not change color_mode
        let mut lc: Vec<Color> = Vec::new();
        let lw: Vec<f64> = vec![0.1; mesh.number_of_edges()];
        lc.reserve(mesh.number_of_edges());
        for i in 0..mesh.number_of_edges() {
            lc.push(palette[i % palette.len()].clone());
        }
        mesh.set_linecolors(lc, lw);
        MINI_CHECK!(mesh.color_mode == ColorMode::FACECOLORS);
        MINI_CHECK!(mesh.get_linecolors().len() == mesh.number_of_edges());

        // clear_facecolors reverts color_mode only if currently FaceColors
        mesh.color_mode = ColorMode::FACECOLORS;
        MINI_CHECK!(mesh.color_mode == ColorMode::FACECOLORS);
        mesh.clear_facecolors();
        MINI_CHECK!(mesh.color_mode == ColorMode::OBJECTCOLOR);
        MINI_CHECK!(mesh.get_facecolors().is_empty());

        // clear_pointcolors does not revert if color_mode != PointColors
        mesh.color_mode = ColorMode::FACECOLORS;
        MINI_CHECK!(mesh.color_mode == ColorMode::FACECOLORS);
        mesh.clear_pointcolors();
        MINI_CHECK!(mesh.color_mode == ColorMode::FACECOLORS);

        // clear_linecolors does not change color_mode
        mesh.color_mode = ColorMode::POINTCOLORS;
        mesh.clear_linecolors();
        MINI_CHECK!(mesh.color_mode == ColorMode::POINTCOLORS);
        MINI_CHECK!(mesh.get_linecolors().is_empty());
    })
}

pub fn run_mesh_from_polylines() -> TestResult {
    MINI_TEST!("From Polylines", {
        use crate::Mesh;
        use crate::Point;

        let mesh = Mesh::from_polylines(vec![
            vec![
                Point::new(1.28955, 0.0, 1.127558),
                Point::new(0.85791, 0.0, 0.225512),
                Point::new(0.64209, -0.866025, -0.225512),
                Point::new(0.85791, -1.732051, 0.225512),
                Point::new(1.458565, -1.732051, 1.127558),
                Point::new(1.50537, -0.866025, 1.578581),
            ],
            vec![
                Point::new(0.64209, 0.866025, -0.225512),
                Point::new(0.114274, 0.866025, -0.686294),
                Point::new(-0.00537, 0.0, -1.578581),
                Point::new(0.21045, -0.866025, -1.127558),
                Point::new(0.64209, -0.866025, -0.225512),
                Point::new(0.85791, 0.0, 0.225512),
            ],
            vec![
                Point::new(1.28955, 1.732051, 1.127558),
                Point::new(0.85791, 1.732051, 0.225512),
                Point::new(0.64209, 0.866025, -0.225512),
                Point::new(0.85791, 0.0, 0.225512),
                Point::new(1.28955, 0.0, 1.127558),
                Point::new(1.853404, 0.866025, 1.578581),
            ],
        ], Some(0.001));
        MINI_CHECK!(mesh.is_valid());
    })
}

pub fn run_mesh_from_lines() -> TestResult {
    MINI_TEST!("From Lines", {
        use crate::Mesh;
        use crate::Line;
        use crate::Point;

        let lines = vec![
            Line::from_points(&Point::new(4.948083, -0.149798, 1.00765),
                              &Point::new(4.395544, -0.996413, 1.196018)),
            Line::from_points(&Point::new(3.866593, 0.371225, 1.376346),
                              &Point::new(4.567265, 0.584361, 1.137476)),
            Line::from_points(&Point::new(3.915298, -0.157402, 1.359741),
                              &Point::new(3.282977, -0.051356, 1.575309)),
            Line::from_points(&Point::new(4.286215, -0.224964, 1.23329),
                              &Point::new(3.607284, -0.987075, 1.464748)),
            Line::from_points(&Point::new(3.744351, 0.971574, 1.41802),
                              &Point::new(3.266367, 0.841359, 1.580972)),
            Line::from_points(&Point::new(4.567265, 0.584361, 1.137476),
                              &Point::new(4.948083, -0.149798, 1.00765)),
            Line::from_points(&Point::new(4.395544, -0.996413, 1.196018),
                              &Point::new(3.607284, -0.987075, 1.464748)),
            Line::from_points(&Point::new(3.915298, -0.157402, 1.359741),
                              &Point::new(4.286215, -0.224964, 1.23329)),
            Line::from_points(&Point::new(3.282977, -0.051356, 1.575309),
                              &Point::new(3.266367, 0.841359, 1.580972)),
            Line::from_points(&Point::new(3.744351, 0.971574, 1.41802),
                              &Point::new(3.866593, 0.371225, 1.376346)),
        ];
        let mesh = Mesh::from_lines(&lines, true, None);
        MINI_CHECK!(mesh.is_valid());
    })
}

pub fn run_mesh_from_polygon_with_holes() -> TestResult {
    MINI_TEST!("From Polygon With Holes", {
        use crate::Mesh;
        use crate::Point;

        let mesh = Mesh::from_polygon_with_holes(&[
            vec![
                Point::new(8.940934, 0.917382, 0.049546),
                Point::new(8.930493, 1.36458, 0.251429),
                Point::new(8.954508, 1.595448, 0.346958),
                Point::new(9.457671, 1.821395, 0.298639),
                Point::new(9.717078, 1.014296, -0.136839),
                Point::new(9.363048, 0.91534, -0.07616),
                Point::new(9.33327, 0.459713, -0.269899),
                Point::new(9.065708, 0.635281, -0.112748),
            ],
            vec![
                Point::new(7.494779, -0.556523, -0.178103),
                Point::new(6.542877, 0.148384, 0.416685),
                Point::new(6.967337, 2.119511, 1.167431),
                Point::new(11.204553, 2.961749, 0.289102),
                Point::new(9.658416, 0.465135, -0.363618),
                Point::new(10.247775, -1.032727, -1.203717),
            ],
            vec![
                Point::new(7.922105, 0.548716, 0.186877),
                Point::new(7.410178, 0.844297, 0.469625),
                Point::new(7.408889, 1.185147, 0.621527),
                Point::new(7.885956, 1.424645, 0.586947),
                Point::new(8.178727, 1.32996, 0.458299),
                Point::new(8.307609, 0.88254, 0.2213),
                Point::new(7.950364, 0.924872, 0.345738),
            ],
        ], true);
        MINI_CHECK!(mesh.is_valid());

        let mesh_sorted = Mesh::from_polygon_with_holes(&[
            vec![
                Point::new(1.0, 1.0, 0.0), Point::new(3.0, 1.0, 0.0),
                Point::new(3.0, 3.0, 0.0), Point::new(1.0, 3.0, 0.0),
            ],
            vec![
                Point::new(0.0, 0.0, 0.0), Point::new(4.0, 0.0, 0.0),
                Point::new(4.0, 4.0, 0.0), Point::new(0.0, 4.0, 0.0),
            ],
        ], true);
        MINI_CHECK!(mesh_sorted.is_valid());
    })
}

pub fn run_mesh_loft() -> TestResult {
    MINI_TEST!("Loft", {
        use crate::Mesh;
        use crate::Point;
        use crate::Polyline;

        let bottom = vec![
            Polyline::new(vec![
                Point::new(13.20069, -0.556523, -0.178103),
                Point::new(12.248787, 0.148384, 0.416685),
                Point::new(12.673247, 2.119511, 1.167431),
                Point::new(16.910464, 2.961749, 0.289102),
                Point::new(15.364327, 0.465135, -0.363618),
                Point::new(15.953685, -1.032727, -1.203717),
                Point::new(13.20069, -0.556523, -0.178103),
            ]),
            Polyline::new(vec![
                Point::new(14.646845, 0.917382, 0.049546),
                Point::new(14.636404, 1.36458, 0.251429),
                Point::new(14.660418, 1.595448, 0.346958),
                Point::new(15.163581, 1.821395, 0.298639),
                Point::new(15.422988, 1.014296, -0.136839),
                Point::new(15.068958, 0.91534, -0.07616),
                Point::new(15.03918, 0.459713, -0.269899),
                Point::new(14.771618, 0.635281, -0.112748),
                Point::new(14.646845, 0.917382, 0.049546),
            ]),
            Polyline::new(vec![
                Point::new(13.628016, 0.548716, 0.186877),
                Point::new(13.116088, 0.844297, 0.469625),
                Point::new(13.114799, 1.185147, 0.621527),
                Point::new(13.591866, 1.424645, 0.586947),
                Point::new(13.884637, 1.32996, 0.458299),
                Point::new(14.013519, 0.88254, 0.2213),
                Point::new(13.656275, 0.924872, 0.345738),
                Point::new(13.628016, 0.548716, 0.186877),
            ]),
        ];
        let top = vec![
            Polyline::new(vec![
                Point::new(13.375135, -0.818817, 0.411936),
                Point::new(12.423233, -0.113909, 1.006724),
                Point::new(12.847692, 1.857217, 1.75747),
                Point::new(17.084909, 2.699455, 0.879141),
                Point::new(15.538772, 0.202841, 0.226421),
                Point::new(16.12813, -1.295021, -0.613678),
                Point::new(13.375135, -0.818817, 0.411936),
            ]),
            Polyline::new(vec![
                Point::new(14.82129, 0.655088, 0.639585),
                Point::new(14.810849, 1.102286, 0.841468),
                Point::new(14.834864, 1.333154, 0.936997),
                Point::new(15.338026, 1.559101, 0.888678),
                Point::new(15.597433, 0.752002, 0.4532),
                Point::new(15.243404, 0.653046, 0.513879),
                Point::new(15.213626, 0.197419, 0.32014),
                Point::new(14.946063, 0.372987, 0.477291),
                Point::new(14.82129, 0.655088, 0.639585),
            ]),
            Polyline::new(vec![
                Point::new(13.802461, 0.286422, 0.776916),
                Point::new(13.290534, 0.582003, 1.059664),
                Point::new(13.289245, 0.922853, 1.211566),
                Point::new(13.766312, 1.162351, 1.176986),
                Point::new(14.059082, 1.067666, 1.048338),
                Point::new(14.187964, 0.620246, 0.811339),
                Point::new(13.83072, 0.662578, 0.935777),
                Point::new(13.802461, 0.286422, 0.776916),
            ]),
        ];
        let mesh = Mesh::loft(&bottom, &top, true);
        MINI_CHECK!(mesh.is_valid());
        MINI_CHECK!(mesh.is_closed());

        let mesh_no_cap = Mesh::loft(&bottom, &top, false);
        MINI_CHECK!(mesh_no_cap.is_valid());
        MINI_CHECK!(!mesh_no_cap.is_closed());
    })
}

pub fn run_mesh_from_polygon_with_holes_many() -> TestResult {
    MINI_TEST!("From Polygon With Holes Many", {
        use crate::Mesh;
        use crate::Point;

        let inputs: Vec<Vec<Vec<Point>>> = (0..4).map(|i| {
            let x = i as f64 * 7.0;
            vec![
                vec![
                    Point::new(x, 0.0, 0.0), Point::new(x+5.0, 0.0, 0.0),
                    Point::new(x+5.0, 5.0, 0.0), Point::new(x, 5.0, 0.0),
                ],
                vec![
                    Point::new(x+1.0, 1.0, 0.0), Point::new(x+4.0, 1.0, 0.0),
                    Point::new(x+4.0, 4.0, 0.0), Point::new(x+1.0, 4.0, 0.0),
                ],
            ]
        }).collect();
        let meshes = Mesh::from_polygon_with_holes_many(inputs.clone(), false, true);
        for m in &meshes { MINI_CHECK!(m.is_valid()); }
        let meshes_seq = Mesh::from_polygon_with_holes_many(inputs, false, false);
        MINI_CHECK!(meshes_seq[0].number_of_faces() == meshes[0].number_of_faces());
    })
}

pub fn run_mesh_loft_many() -> TestResult {
    MINI_TEST!("Loft Many", {
        use crate::Mesh;
        use crate::Point;
        use crate::Polyline;

        let pairs: Vec<(Vec<Polyline>, Vec<Polyline>)> = (0..6).map(|i| {
            let x = i as f64 * 3.0;
            let h = 1.0 + i as f64 * 0.5;
            let b = Polyline::new(vec![
                Point::new(x, 0.0, 0.0), Point::new(x+1.0, 0.0, 0.0),
                Point::new(x+1.0, 1.0, 0.0), Point::new(x, 1.0, 0.0), Point::new(x, 0.0, 0.0),
            ]);
            let t = Polyline::new(vec![
                Point::new(x, 0.0, h), Point::new(x+1.0, 0.0, h),
                Point::new(x+1.0, 1.0, h), Point::new(x, 1.0, h), Point::new(x, 0.0, h),
            ]);
            (vec![b], vec![t])
        }).collect();
        let meshes = Mesh::loft_many(pairs.clone(), true, true);
        for m in &meshes {
            MINI_CHECK!(m.is_valid());
            MINI_CHECK!(m.is_closed());
        }
        let meshes_seq = Mesh::loft_many(pairs, true, false);
        MINI_CHECK!(meshes_seq[0].number_of_faces() == meshes[0].number_of_faces());
    })
}

pub fn run_mesh_boolean_queries() -> TestResult {
    MINI_TEST!("Boolean Queries", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        let f0 = mesh.add_face(vec![v0, v1, v2], None).unwrap();

        let not_empty = mesh.is_empty();
        MINI_CHECK!(!not_empty);

        let valid = mesh.is_valid();
        MINI_CHECK!(valid);

        let closed = mesh.is_closed();
        MINI_CHECK!(!closed);

        let vertex_on_boundary = mesh.is_vertex_on_boundary(v0);
        MINI_CHECK!(vertex_on_boundary);

        let edge_on_boundary = mesh.is_edge_on_boundary(v0, v1);
        MINI_CHECK!(edge_on_boundary);

        let face_on_boundary = mesh.is_face_on_boundary(f0);
        MINI_CHECK!(face_on_boundary);
    })
}

pub fn run_mesh_attributes() -> TestResult {
    MINI_TEST!("Attributes", {
        use crate::Mesh;
        use crate::Point;

        let mesh = Mesh::create_box(1.0, 1.0, 1.0);

        let n_vertices = mesh.number_of_vertices();
        MINI_CHECK!(n_vertices == 8);

        let n_faces = mesh.number_of_faces();
        MINI_CHECK!(n_faces == 6);

        let n_edges = mesh.number_of_edges();
        MINI_CHECK!(n_edges == 12);

        let euler = mesh.euler();
        MINI_CHECK!(euler == 2);

        let (vertices, faces) = mesh.to_vertices_and_faces();
        MINI_CHECK!(faces.len() == n_faces);
        MINI_CHECK!(vertices.len() == n_vertices);
        MINI_CHECK!(TOLERANCE.is_point_close(&vertices[0], &Point::new(-0.5, -0.5, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&vertices[1], &Point::new( 0.5, -0.5, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&vertices[2], &Point::new( 0.5,  0.5, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&vertices[3], &Point::new(-0.5,  0.5, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&vertices[4], &Point::new(-0.5, -0.5,  0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&vertices[5], &Point::new( 0.5, -0.5,  0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&vertices[6], &Point::new( 0.5,  0.5,  0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&vertices[7], &Point::new(-0.5,  0.5,  0.5)));
        MINI_CHECK!(faces[0] == vec![0, 3, 2, 1]);
        MINI_CHECK!(faces[1] == vec![4, 5, 6, 7]);
        MINI_CHECK!(faces[2] == vec![0, 1, 5, 4]);
        MINI_CHECK!(faces[3] == vec![2, 3, 7, 6]);
        MINI_CHECK!(faces[4] == vec![0, 4, 7, 3]);
        MINI_CHECK!(faces[5] == vec![1, 2, 6, 5]);

        let vindex = mesh.vertex_index();
        MINI_CHECK!(vindex.len() == n_vertices);
        MINI_CHECK!(vindex[&0] == 0);
        MINI_CHECK!(vindex[&1] == 1);
        MINI_CHECK!(vindex[&2] == 2);
        MINI_CHECK!(vindex[&3] == 3);
        MINI_CHECK!(vindex[&4] == 4);
        MINI_CHECK!(vindex[&5] == 5);
        MINI_CHECK!(vindex[&6] == 6);
        MINI_CHECK!(vindex[&7] == 7);
    })
}

pub fn run_mesh_edges() -> TestResult {
    MINI_TEST!("Edges", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(1.0, 1.0, 0.0), None);
        let v3 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        mesh.add_face(vec![v0, v1, v2, v3], None);

        let edges = mesh.edges();
        MINI_CHECK!(edges.len() == 4);
        MINI_CHECK!(edges[0] == (0, 3));
    })
}

pub fn run_mesh_vertex_and_face_operations() -> TestResult {
    MINI_TEST!("Vertex and Face Operations", {
        use crate::Mesh;
        use crate::Point;

        // add_vertex — None key auto-assigns sequentially from 0
        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(1.0, 2.0, 3.0), None);
        MINI_CHECK!(v0 == 0);
        MINI_CHECK!(mesh.number_of_vertices() == 1);
        MINI_CHECK!(!mesh.is_empty());
        let v1 = mesh.add_vertex(Point::new(4.0, 5.0, 6.0), Some(42));
        MINI_CHECK!(v1 == 42);
        MINI_CHECK!(mesh.number_of_vertices() == 2);

        // add_face
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        let f = mesh.add_face(vec![v0, v1, v2], None);
        MINI_CHECK!(f.is_some());
        let invalid1 = mesh.add_face(vec![v0, v1], None);
        MINI_CHECK!(invalid1.is_none());
        let invalid2 = mesh.add_face(vec![v0, v1, v0], None);
        MINI_CHECK!(invalid2.is_none());

        // clear
        mesh.clear();
        MINI_CHECK!(mesh.is_empty());
        MINI_CHECK!(mesh.number_of_vertices() == 0);
        MINI_CHECK!(mesh.number_of_faces() == 0);

        // unify_winding — two triangles sharing edge p1-p2, f1 has same-direction halfedge (wrong winding)
        let p0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let p1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let p2 = mesh.add_vertex(Point::new(1.0, 1.0, 0.0), None);
        let p3 = mesh.add_vertex(Point::new(2.0, 1.0, 0.0), None);
        let f0 = mesh.add_face(vec![p0, p1, p2], None).unwrap();
        let f1 = mesh.add_face(vec![p1, p2, p3], None).unwrap();

        let n0_before = mesh.face_normal(f0).unwrap();
        let n1_before = mesh.face_normal(f1).unwrap();
        MINI_CHECK!(n0_before[0]*n1_before[0] + n0_before[1]*n1_before[1] + n0_before[2]*n1_before[2] < 0.0);

        mesh.unify_winding();

        let n0_after = mesh.face_normal(f0).unwrap();
        let n1_after = mesh.face_normal(f1).unwrap();
        MINI_CHECK!(n0_after[0]*n1_after[0] + n0_after[1]*n1_after[1] + n0_after[2]*n1_after[2] > 0.0);
    })
}

pub fn run_mesh_unweld() -> TestResult {
    MINI_TEST!("Unweld", {
        use crate::Mesh;

        let box_mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let u = box_mesh.unweld();

        MINI_CHECK!(u.number_of_faces() == box_mesh.number_of_faces());
        MINI_CHECK!(u.number_of_vertices() == 24);
        for vk in u.vertex.keys() {
            MINI_CHECK!(u.vertex_faces(*vk).len() == 1);
        }
    })
}

pub fn run_mesh_connectivity_queries() -> TestResult {
    MINI_TEST!("Connectivity Queries", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(1.0, 1.0, 0.0), None);
        let v3 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        let v4 = mesh.add_vertex(Point::new(2.0, 0.0, 0.0), None);
        let f0 = mesh.add_face(vec![v0, v1, v2, v3], None).unwrap();
        let f1 = mesh.add_face(vec![v1, v4, v2], None).unwrap();

        // vertex_position
        let pos = mesh.vertex_position(v0);
        MINI_CHECK!(pos.is_some());
        MINI_CHECK!(TOLERANCE.is_point_close(&pos.unwrap(), &Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(mesh.vertex_position(999).is_none());

        // face_vertices
        let fv = mesh.face_vertices(f0);
        MINI_CHECK!(fv.is_some());
        MINI_CHECK!(fv.unwrap().len() == 4);
        MINI_CHECK!(fv.unwrap()[0] == v0 && fv.unwrap()[1] == v1 && fv.unwrap()[2] == v2 && fv.unwrap()[3] == v3);

        // vertex_neighbors
        let nb = mesh.vertex_neighbors(v1);
        MINI_CHECK!(nb.len() == 3);

        // vertex_faces
        let vf0 = mesh.vertex_faces(v0);
        MINI_CHECK!(vf0.len() == 1);
        let vf1 = mesh.vertex_faces(v1);
        MINI_CHECK!(vf1.len() == 2);

        // vertex_edges
        let ve = mesh.vertex_edges(v1);
        MINI_CHECK!(ve.len() == 3);
        MINI_CHECK!(ve.contains(&(v1, v0)));
        MINI_CHECK!(ve.contains(&(v1, v2)));
        MINI_CHECK!(ve.contains(&(v1, v4)));

        // face_edges
        let fe = mesh.face_edges(f0);
        MINI_CHECK!(fe.len() == 4);
        MINI_CHECK!(fe[0] == (v0, v1));
        MINI_CHECK!(fe[1] == (v1, v2));
        MINI_CHECK!(fe[2] == (v2, v3));
        MINI_CHECK!(fe[3] == (v3, v0));

        // face_neighbors
        let fn0 = mesh.face_neighbors(f0);
        MINI_CHECK!(fn0.len() == 1);
        MINI_CHECK!(fn0[0] == f1);

        // edge_vertices
        let ev = mesh.edge_vertices(v0, v1);
        MINI_CHECK!(ev[0] == v0 && ev[1] == v1);

        // edge_faces
        let ef_inner = mesh.edge_faces(v1, v2);
        MINI_CHECK!(ef_inner.0.is_some() && ef_inner.1.is_some());
        let ef_boundary = mesh.edge_faces(v0, v1);
        MINI_CHECK!(ef_boundary.0.is_some() != ef_boundary.1.is_some());

        // edge_edges
        let ee = mesh.edge_edges(v1, v2);
        MINI_CHECK!(ee.len() == 4);
        MINI_CHECK!(!ee.contains(&(v1, v2)));
        MINI_CHECK!(!ee.contains(&(v2, v1)));
    })
}

pub fn run_mesh_geometric_properties() -> TestResult {
    MINI_TEST!("Geometric Properties", {
        use crate::Mesh;
        use crate::Point;
        use crate::mesh::NormalWeighting;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(-1.0, 0.0, 0.0), None);
        let v3 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        let f0 = mesh.add_face(vec![v0, v1, v3], None).unwrap();
        let _f1 = mesh.add_face(vec![v0, v3, v2], None).unwrap();

        // face_normal
        let fn_ = mesh.face_normal(f0);
        MINI_CHECK!(fn_.is_some());
        MINI_CHECK!(TOLERANCE.is_close(fn_.unwrap()[2], 1.0));

        // vertex_normal
        let vn = mesh.vertex_normal(v0);
        MINI_CHECK!(vn.is_some());
        MINI_CHECK!(vn.unwrap()[2].abs() == 1.0);

        // vertex_normal_weighted
        let vnw = mesh.vertex_normal_weighted(v0, NormalWeighting::Angle);
        MINI_CHECK!(vnw.is_some());
        MINI_CHECK!(TOLERANCE.is_close(vnw.unwrap()[2], 1.0));

        // face_area
        let area = mesh.face_area(f0);
        MINI_CHECK!(area.is_some());
        MINI_CHECK!(TOLERANCE.is_close(area.unwrap(), 0.5));

        // vertex_angle_in_face
        let angle = mesh.vertex_angle_in_face(v0, f0);
        MINI_CHECK!(angle.is_some());
        MINI_CHECK!(TOLERANCE.is_close(angle.unwrap(), PI / 2.0));
        MINI_CHECK!(mesh.vertex_angle_in_face(999, f0).is_none());

        // dihedral_angle — interior edge v0-v3 shared by f0 and f1 (coplanar = PI)
        let da = mesh.dihedral_angle(v3, v0);
        MINI_CHECK!(da.is_some());
        MINI_CHECK!(TOLERANCE.is_close(da.unwrap(), PI));
        // boundary edge — only one face
        MINI_CHECK!(mesh.dihedral_angle(v0, v1).is_none());

        // face_normals
        let fns = mesh.face_normals();
        MINI_CHECK!(fns.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(fns[&f0][2], 1.0));

        // vertex_normals
        let vns = mesh.vertex_normals();
        MINI_CHECK!(vns.len() == mesh.number_of_vertices());
        MINI_CHECK!(TOLERANCE.is_close(vns[&v0][2], 1.0));

        // vertex_normals_weighted
        let vnsw = mesh.vertex_normals_weighted(NormalWeighting::Angle);
        MINI_CHECK!(vnsw.len() == mesh.number_of_vertices());
        MINI_CHECK!(TOLERANCE.is_close(vnsw[&v0][2], 1.0));
    })
}

pub fn run_mesh_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::Mesh;
        use crate::Point;
        use crate::Xform;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        mesh.add_face(vec![v0, v1, v2], None);

        // transform(None) — apply stored xform in-place; xform field unchanged
        let mut mesh1 = mesh.duplicate();
        mesh1.xform = Xform::translation(0.0, 0.0, 1.0);
        mesh1.transform(None);
        MINI_CHECK!(!mesh1.xform.is_identity());
        MINI_CHECK!(mesh1.vertex_position(v0).unwrap()[2] == 1.0);

        // transform(Some(xf)) — apply given xform in-place; stored xform unchanged
        let mut mesh2 = mesh.duplicate();
        let x = Xform::translation(0.0, 0.0, 1.0);
        mesh2.transform(Some(&x));
        MINI_CHECK!(mesh2.xform.is_identity());
        MINI_CHECK!(mesh2.vertex_position(v0).unwrap()[2] == 1.0);

        // transformed(None) — copy with stored xform applied
        let mut mesh3 = mesh.duplicate();
        mesh3.xform = Xform::translation(0.0, 0.0, 10.0);
        let mesh3t = mesh3.transformed(None);
        MINI_CHECK!(!mesh3t.xform.is_identity());
        MINI_CHECK!(mesh3t.vertex_position(v0).unwrap()[2] == 10.0);

        // transformed(Some(xf)) — copy with given xform applied
        let mesh4 = mesh.duplicate();
        let x = Xform::translation(0.0, 0.0, 10.0);
        let mesh4t = mesh4.transformed(Some(&x));
        MINI_CHECK!(mesh4t.xform.is_identity());
        MINI_CHECK!(mesh4t.vertex_position(v0).unwrap()[2] == 10.0);
    })
}

pub fn run_mesh_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Mesh;
        use crate::Point;
        use std::path::PathBuf;

        let mut mesh = Mesh::new();
        mesh.name = "test_mesh".to_string();
        mesh.color_mode = crate::mesh::ColorMode::FACECOLORS;
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        mesh.add_face(vec![v0, v1, v2], None);

        // JSON object
        let json = mesh.jsondump();
        let loaded_json = Mesh::jsonload(&json).unwrap();
        MINI_CHECK!(loaded_json.name == mesh.name);
        MINI_CHECK!(loaded_json.color_mode == mesh.color_mode);
        MINI_CHECK!(loaded_json.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK!(loaded_json.number_of_faces() == mesh.number_of_faces());

        // String
        let json_string = mesh.json_dumps();
        let loaded_string = Mesh::json_loads(&json_string);
        MINI_CHECK!(loaded_string.name == mesh.name);
        MINI_CHECK!(loaded_string.number_of_vertices() == mesh.number_of_vertices());

        // File
        let filename = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent().unwrap()
            .join("serialization").join("test_mesh.json");
        mesh.json_dump(filename.to_str().unwrap()).unwrap();
        let loaded_file = Mesh::json_load(filename.to_str().unwrap()).unwrap();
        MINI_CHECK!(loaded_file.name == mesh.name);
        MINI_CHECK!(loaded_file.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK!(loaded_file.number_of_faces() == mesh.number_of_faces());
    })
}

pub fn run_mesh_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Mesh;
        use crate::Point;
        use std::path::PathBuf;

        let mut mesh = Mesh::new();
        mesh.name = "test_mesh_proto".to_string();
        mesh.color_mode = crate::mesh::ColorMode::FACECOLORS;
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        mesh.add_face(vec![v0, v1, v2], None);

        // String
        let proto_bytes = mesh.pb_dumps();
        let loaded_string = Mesh::pb_loads(&proto_bytes).unwrap();
        MINI_CHECK!(loaded_string.name == mesh.name);
        MINI_CHECK!(loaded_string.color_mode == mesh.color_mode);
        MINI_CHECK!(loaded_string.number_of_vertices() == mesh.number_of_vertices());

        // File
        let filename = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent().unwrap()
            .join("serialization").join("test_mesh.bin");
        mesh.pb_dump(filename.to_str().unwrap());
        let loaded_file = Mesh::pb_load(filename.to_str().unwrap());
        MINI_CHECK!(loaded_file.name == mesh.name);
        MINI_CHECK!(loaded_file.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK!(loaded_file.number_of_faces() == mesh.number_of_faces());
        MINI_CHECK!(loaded_file.guid == mesh.guid);
    })
}

// Register tests with the shared registry
REGISTER_MINI_TEST!("Mesh", "Constructor", crate::mesh_test::run_mesh_constructor);
REGISTER_MINI_TEST!("Mesh", "From Polylines", crate::mesh_test::run_mesh_from_polylines);
REGISTER_MINI_TEST!("Mesh", "From Lines", crate::mesh_test::run_mesh_from_lines);
REGISTER_MINI_TEST!("Mesh", "From Polygon With Holes", crate::mesh_test::run_mesh_from_polygon_with_holes);
REGISTER_MINI_TEST!("Mesh", "Loft", crate::mesh_test::run_mesh_loft);
REGISTER_MINI_TEST!("Mesh", "From Polygon With Holes Many", crate::mesh_test::run_mesh_from_polygon_with_holes_many);
REGISTER_MINI_TEST!("Mesh", "Loft Many", crate::mesh_test::run_mesh_loft_many);
REGISTER_MINI_TEST!("Mesh", "Boolean Queries", crate::mesh_test::run_mesh_boolean_queries);
REGISTER_MINI_TEST!("Mesh", "Attributes", crate::mesh_test::run_mesh_attributes);
REGISTER_MINI_TEST!("Mesh", "Edges", crate::mesh_test::run_mesh_edges);
REGISTER_MINI_TEST!("Mesh", "Vertex and Face Operations", crate::mesh_test::run_mesh_vertex_and_face_operations);
REGISTER_MINI_TEST!("Mesh", "Unweld", crate::mesh_test::run_mesh_unweld);
REGISTER_MINI_TEST!("Mesh", "Connectivity Queries", crate::mesh_test::run_mesh_connectivity_queries);
REGISTER_MINI_TEST!("Mesh", "Geometric Properties", crate::mesh_test::run_mesh_geometric_properties);
REGISTER_MINI_TEST!("Mesh", "Transformation", crate::mesh_test::run_mesh_transformation);
REGISTER_MINI_TEST!("Mesh", "Json Roundtrip", crate::mesh_test::run_mesh_json_roundtrip);
REGISTER_MINI_TEST!("Mesh", "Protobuf Roundtrip", crate::mesh_test::run_mesh_protobuf_roundtrip);
