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

pub fn run_mesh_loft_panels() -> TestResult {
    MINI_TEST!("Loft with quads and triangles", {
        use crate::Mesh;
        use crate::Point;
        use crate::Color;

        let top7: Vec<Vec<Point>> = vec![
            vec![
                Point::new(250.,-250.,500.),
                Point::new(250.,250.,500.),
                Point::new(-250.,250.,500.),
                Point::new(-250.,-250.,500.),
                Point::new(250.,-250.,500.),
            ],
            vec![
                Point::new(-250.,500.,250.),
                Point::new(-250.,250.,500.),
                Point::new(250.,250.,500.),
                Point::new(250.,500.,250.),
                Point::new(-250.,500.,250.),
            ],
            vec![
                Point::new(250.,-250.,500.),
                Point::new(500.,-250.,250.),
                Point::new(500.,250.,250.),
                Point::new(250.,250.,500.),
                Point::new(250.,-250.,500.),
            ],
            vec![
                Point::new(250.,500.,250.),
                Point::new(250.,250.,500.),
                Point::new(500.,250.,250.),
                Point::new(250.,500.,250.),
            ],
            vec![
                Point::new(-250.,500.,250.),
                Point::new(250.,500.,250.),
                Point::new(250.,500.,-250.),
                Point::new(-250.,500.,-250.),
                Point::new(-250.,500.,250.),
            ],
            vec![
                Point::new(250.,500.,250.),
                Point::new(500.,250.,250.),
                Point::new(500.,250.,-250.),
                Point::new(250.,500.,-250.),
                Point::new(250.,500.,250.),
            ],
            vec![
                Point::new(500.,-250.,250.),
                Point::new(500.,-250.,-250.),
                Point::new(500.,250.,-250.),
                Point::new(500.,250.,250.),
                Point::new(500.,-250.,250.),
            ],
        ];
        let bot7: Vec<Vec<Point>> = vec![
            vec![
                Point::new(270.710678,-250.,550.),
                Point::new(270.710678,265.891862,550.),
                Point::new(265.891862,270.710678,550.),
                Point::new(-250.,270.710678,550.),
                Point::new(-250.,-250.,550.),
                Point::new(270.710678,-250.,550.),
            ],
            vec![
                Point::new(270.710678,-250.,550.),
                Point::new(550.,-250.,270.710678),
                Point::new(550.,265.891862,270.710678),
                Point::new(270.710678,265.891862,550.),
                Point::new(270.710678,-250.,550.),
            ],
            vec![
                Point::new(-250.,550.,270.710678),
                Point::new(-250.,270.710678,550.),
                Point::new(265.891862,270.710678,550.),
                Point::new(265.891862,550.,270.710678),
                Point::new(-250.,550.,270.710678),
            ],
            vec![
                Point::new(265.891862,550.,270.710678),
                Point::new(265.891862,270.710678,550.),
                Point::new(270.710678,265.891862,550.),
                Point::new(550.,265.891862,270.710678),
                Point::new(550.,270.710678,265.891862),
                Point::new(270.710678,550.,265.891862),
                Point::new(265.891862,550.,270.710678),
            ],
            vec![
                Point::new(-250.,550.,270.710678),
                Point::new(265.891862,550.,270.710678),
                Point::new(270.710678,550.,265.891862),
                Point::new(270.710678,550.,-250.),
                Point::new(-250.,550.,-250.),
                Point::new(-250.,550.,270.710678),
            ],
            vec![
                Point::new(270.710678,550.,265.891862),
                Point::new(550.,270.710678,265.891862),
                Point::new(550.,270.710678,-250.),
                Point::new(270.710678,550.,-250.),
                Point::new(270.710678,550.,265.891862),
            ],
            vec![
                Point::new(550.,-250.,270.710678),
                Point::new(550.,-250.,-250.),
                Point::new(550.,270.710678,-250.),
                Point::new(550.,270.710678,265.891862),
                Point::new(550.,265.891862,270.710678),
                Point::new(550.,-250.,270.710678),
            ],
        ];
        let (mut panels, adj, _top_mesh, _bot_mesh) = Mesh::loft_panels(top7, bot7, 0.001, 0.0, 2.0, true, false);

        // Color faces: blue=top cap, red=bot cap, gray=quad wall, yellow=tri wall
        for i in 0..panels.len() {
            let mut face_colors: Vec<Color> = Vec::new();
            for (_, role) in &panels[i].face_roles {
                let color = match *role {
                    "TopCap" => Color::blue(),
                    "BotCap" => Color::red(),
                    "TriWall" => Color::yellow(),
                    _ => Color::grey(),
                };
                face_colors.push(color);
            }
            panels[i].mesh.set_facecolors(face_colors);
        }

        // face centroids labelled with panel index
        for i in 0..panels.len() {
            let mut c = panels[i].mesh.centroid();
            c.name = format!("p{}", i);
        }

        // adjacency: for each shared edge — text dot at midpoint labelled "p{i}f{idx}<->p{j}f{idx}"
        for pair in &adj {
            let w = &panels[pair.pi].wall_faces[pair.wi];
            let mut pt = panels[pair.pi].mesh.face_centroid(w.face_key).unwrap();
            pt.name = format!("p{} f{} - p{} f{}", pair.pi, w.face_index, pair.pj, panels[pair.pj].wall_faces[pair.wj].face_index);
        }
        MINI_CHECK!(panels.len() == 7);
        MINI_CHECK!(panels[0].mesh.is_valid());
        MINI_CHECK!(panels[1].mesh.is_valid());
        MINI_CHECK!(panels[2].mesh.is_valid());
        MINI_CHECK!(panels[3].mesh.is_valid());
        MINI_CHECK!(panels[4].mesh.is_valid());
        MINI_CHECK!(panels[5].mesh.is_valid());
        MINI_CHECK!(panels[6].mesh.is_valid());
        MINI_CHECK!(adj.len() == 9);
        MINI_CHECK!(adj[0].pi == 0 && adj[0].pj == 2);
        MINI_CHECK!(adj[1].pi == 0 && adj[1].pj == 1);
        MINI_CHECK!(adj[2].pi == 1 && adj[2].pj == 3);
        MINI_CHECK!(adj[3].pi == 1 && adj[3].pj == 4);
        MINI_CHECK!(adj[4].pi == 2 && adj[4].pj == 6);
        MINI_CHECK!(adj[5].pi == 2 && adj[5].pj == 3);
        MINI_CHECK!(adj[6].pi == 3 && adj[6].pj == 5);
        MINI_CHECK!(adj[7].pi == 4 && adj[7].pj == 5);
        MINI_CHECK!(adj[8].pi == 5 && adj[8].pj == 6);
    })
}

pub fn run_mesh_boolean_queries() -> TestResult {
    MINI_TEST!("Boolean Queries", {
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
        let v0: usize = 1;
        let v1: usize = 2;
        let v2: usize = 3;
        let f0: usize = 0;

        let empty = mesh.is_empty();
        MINI_CHECK!(!empty);

        let valid = mesh.is_valid();
        MINI_CHECK!(valid);

        let closed = mesh.is_closed();
        MINI_CHECK!(!closed);

        let vertex_on_boundary = mesh.is_vertex_on_boundary(v0);
        MINI_CHECK!(!vertex_on_boundary);

        let edge_not_on_boundary = mesh.is_edge_on_boundary(v0, v1);
        MINI_CHECK!(!edge_not_on_boundary);

        let edge_on_boundary = mesh.is_edge_on_boundary(v1, v2);
        MINI_CHECK!(edge_on_boundary);

        let face_on_boundary = mesh.is_face_on_boundary(f0);
        MINI_CHECK!(face_on_boundary);
    })
}

pub fn run_mesh_attributes() -> TestResult {
    MINI_TEST!("Attributes", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);

        let n_vertices = mesh.number_of_vertices();
        MINI_CHECK!(n_vertices == 8);

        let n_faces = mesh.number_of_faces();
        MINI_CHECK!(n_faces == 6);

        let n_edges = mesh.number_of_edges();
        MINI_CHECK!(n_edges == 12);

        let euler = mesh.euler();
        MINI_CHECK!(euler == 2);

        let (pts, fidx) = mesh.to_vertices_and_faces();
        MINI_CHECK!(fidx.len() == n_faces);
        MINI_CHECK!(pts.len() == n_vertices);
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[0], &Point::new(-0.5, -0.5, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[1], &Point::new( 0.5, -0.5, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[2], &Point::new( 0.5,  0.5, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[3], &Point::new(-0.5,  0.5, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[4], &Point::new(-0.5, -0.5,  0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[5], &Point::new( 0.5, -0.5,  0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[6], &Point::new( 0.5,  0.5,  0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[7], &Point::new(-0.5,  0.5,  0.5)));
        MINI_CHECK!(fidx[0] == vec![0, 3, 2, 1]);
        MINI_CHECK!(fidx[1] == vec![4, 5, 6, 7]);
        MINI_CHECK!(fidx[2] == vec![0, 1, 5, 4]);
        MINI_CHECK!(fidx[3] == vec![2, 3, 7, 6]);
        MINI_CHECK!(fidx[4] == vec![0, 4, 7, 3]);
        MINI_CHECK!(fidx[5] == vec![1, 2, 6, 5]);

        let mut vertex_to_index = mesh.vertex_index();
        MINI_CHECK!(vertex_to_index.len() == n_vertices);
        MINI_CHECK!(vertex_to_index[&0] == 0);
        MINI_CHECK!(vertex_to_index[&1] == 1);
        MINI_CHECK!(vertex_to_index[&2] == 2);
        MINI_CHECK!(vertex_to_index[&3] == 3);
        MINI_CHECK!(vertex_to_index[&4] == 4);
        MINI_CHECK!(vertex_to_index[&5] == 5);
        MINI_CHECK!(vertex_to_index[&6] == 6);
        MINI_CHECK!(vertex_to_index[&7] == 7);

        // sparse keys via remove_vertex: key != index after removal
        let mut mesh2 = mesh.duplicate();
        let kr = mesh2.vertices()[3];
        mesh2.remove_vertex(kr);
        vertex_to_index = mesh2.vertex_index();
        MINI_CHECK!(vertex_to_index.len() == 7);
        MINI_CHECK!(vertex_to_index[&0] == 0);
        MINI_CHECK!(vertex_to_index[&1] == 1);
        MINI_CHECK!(vertex_to_index[&2] == 2);
        MINI_CHECK!(!vertex_to_index.contains_key(&3));
        MINI_CHECK!(vertex_to_index[&4] == 3);
        MINI_CHECK!(vertex_to_index[&5] == 4);
        MINI_CHECK!(vertex_to_index[&6] == 5);
        MINI_CHECK!(vertex_to_index[&7] == 6);

        // vertices / faces / edges
        let vertices = mesh.vertices();
        MINI_CHECK!(vertices.len() == 8);
        MINI_CHECK!(vertices[0] == 0);
        MINI_CHECK!(vertices[1] == 1);
        MINI_CHECK!(vertices[2] == 2);
        MINI_CHECK!(vertices[3] == 3);
        MINI_CHECK!(vertices[4] == 4);
        MINI_CHECK!(vertices[5] == 5);
        MINI_CHECK!(vertices[6] == 6);
        MINI_CHECK!(vertices[7] == 7);
        let faces = mesh.faces();
        MINI_CHECK!(faces.len() == 6);
        MINI_CHECK!(faces[0] == 0);
        MINI_CHECK!(faces[1] == 1);
        MINI_CHECK!(faces[2] == 2);
        MINI_CHECK!(faces[3] == 3);
        MINI_CHECK!(faces[4] == 4);
        MINI_CHECK!(faces[5] == 5);
        let edges = mesh.edges();
        MINI_CHECK!(edges.len() == 12);
        MINI_CHECK!(edges[0]  == (0, 1));
        MINI_CHECK!(edges[1]  == (0, 3));
        MINI_CHECK!(edges[2]  == (0, 4));
        MINI_CHECK!(edges[3]  == (1, 2));
        MINI_CHECK!(edges[4]  == (1, 5));
        MINI_CHECK!(edges[5]  == (2, 3));
        MINI_CHECK!(edges[6]  == (2, 6));
        MINI_CHECK!(edges[7]  == (3, 7));
        MINI_CHECK!(edges[8]  == (4, 5));
        MINI_CHECK!(edges[9]  == (4, 7));
        MINI_CHECK!(edges[10] == (5, 6));
        MINI_CHECK!(edges[11] == (6, 7));

        // naked (closed box: no naked edges before removal)
        MINI_CHECK!(mesh.naked_edges(true).len() == 0);
        MINI_CHECK!(mesh.naked_faces(false).len() == 6);
        // remove one face — box becomes open, check naked
        let fk0 = mesh.faces()[0];
        mesh.remove_face(fk0);
        let ne = mesh.naked_edges(true);
        MINI_CHECK!(ne.len() == 4);
        MINI_CHECK!(ne[0] == (0, 1));
        let ni = mesh.naked_edges(false);
        MINI_CHECK!(ni.len() == 8);
        let nv = mesh.naked_vertices(true);
        MINI_CHECK!(nv.len() == 4);
        let nvi = mesh.naked_vertices(false);
        MINI_CHECK!(nvi.len() == 4);
        let nf = mesh.naked_faces(true);
        MINI_CHECK!(nf.len() == 4);
        let nfi = mesh.naked_faces(false);
        MINI_CHECK!(nfi.len() == 1);
    })
}

pub fn run_mesh_edges() -> TestResult {
    MINI_TEST!("Edges", {
        use crate::Mesh;

        let mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let v0 = mesh.vertices()[0];
        let v1 = mesh.vertices()[1];
        let edges = mesh.edges();
        MINI_CHECK!(edges.len() == 12);
        MINI_CHECK!(edges[0] == (v0, v1));
    })
}

pub fn run_mesh_vertex_and_face_operations() -> TestResult {
    MINI_TEST!("Vertex and Face Operations", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let vkeys = mesh.vertices();
        let v0 = vkeys[0];
        let v1 = vkeys[1];
        MINI_CHECK!(!mesh.is_empty());
        MINI_CHECK!(mesh.number_of_vertices() == 8);

        // add_face: invalid (too few vertices)
        let invalid1 = mesh.add_face(vec![v0, v1], None);
        MINI_CHECK!(invalid1.is_none());
        // add_face: invalid (duplicate vertex)
        let invalid2 = mesh.add_face(vec![v0, v1, v0], None);
        MINI_CHECK!(invalid2.is_none());

        // clear
        let mut mesh2 = mesh.duplicate();
        mesh2.clear();
        MINI_CHECK!(mesh2.is_empty());
        MINI_CHECK!(mesh2.number_of_vertices() == 0);
        MINI_CHECK!(mesh2.number_of_faces() == 0);

        // unify_winding — from_vertices_and_faces creates 2 triangles with mismatched normals
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 1.0, 0.0),
        ];
        let mut mesh3 = Mesh::from_vertices_and_faces(pts, vec![vec![0, 1, 2], vec![1, 2, 3]]);
        let fkeys3 = mesh3.faces();
        let f0 = fkeys3[0];
        let f1 = fkeys3[1];
        let n0_before = mesh3.face_normal(f0);
        let n1_before = mesh3.face_normal(f1);
        MINI_CHECK!(n0_before.is_some() && n1_before.is_some());
        MINI_CHECK!(n0_before.unwrap().dot(&n1_before.unwrap()) < 0.0);  // wrong: normals point opposite ways

        mesh3.unify_winding();

        let n0_after = mesh3.face_normal(f0);
        let n1_after = mesh3.face_normal(f1);
        MINI_CHECK!(n0_after.is_some() && n1_after.is_some());
        MINI_CHECK!(n0_after.unwrap().dot(&n1_after.unwrap()) > 0.0);  // correct: normals agree

        // unweld and weld
        let u = mesh.unweld();
        MINI_CHECK!(u.number_of_vertices() == 24);

        let w = u.weld(0.001);
        MINI_CHECK!(w.number_of_vertices() == 8);
        MINI_CHECK!(w.number_of_faces() == 6);
        for vk in w.vertex.keys() {
            MINI_CHECK!(w.vertex_faces(*vk).len() == 3);
        }

        // remove_face
        let mut mesh5 = mesh.duplicate();
        let fa = mesh5.faces()[0];
        mesh5.remove_face(fa);
        MINI_CHECK!(mesh5.number_of_faces() == 5);
        MINI_CHECK!(mesh5.number_of_edges() == 12);
        MINI_CHECK!(mesh5.number_of_vertices() == 8);

        // remove_vertex
        let mut mesh6 = mesh.duplicate();
        let vr = mesh6.vertices()[0];
        mesh6.remove_vertex(vr);
        let vi6 = mesh6.vertex_index();
        MINI_CHECK!(!vi6.contains_key(&vr));
        MINI_CHECK!(mesh6.number_of_faces() == 3);
        MINI_CHECK!(mesh6.number_of_vertices() == 7);

        // remove_edge
        let mut mesh7 = mesh.duplicate();
        let ea = mesh7.vertices()[0];
        let eb = mesh7.vertices()[1];
        mesh7.remove_edge(ea, eb);
        MINI_CHECK!(mesh7.number_of_faces() == 4);
        MINI_CHECK!(mesh7.number_of_edges() == 11);
        MINI_CHECK!(mesh7.number_of_vertices() == 8);

        // remove_face then check naked: box minus one face → 5 faces with 4 naked edges
        let mut mesh8 = mesh.duplicate();
        let fd0 = mesh8.faces()[0];
        mesh8.remove_face(fd0);
        MINI_CHECK!(mesh8.number_of_faces() == 5);
        MINI_CHECK!(mesh8.naked_edges(true).len() == 4);
        MINI_CHECK!(mesh8.naked_edges(false).len() == 8);
        MINI_CHECK!(mesh8.naked_faces(true).len() == 4);
        MINI_CHECK!(mesh8.naked_faces(false).len() == 1);
    })
}

pub fn run_mesh_connectivity_queries() -> TestResult {
    MINI_TEST!("Connectivity Queries", {
        use crate::Mesh;
        use crate::Point;

        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];
        let mesh = Mesh::from_vertices_and_faces(pts, vec![vec![0, 1, 2, 3], vec![1, 4, 2]]);
        let vkeys = mesh.vertices();
        let v0 = vkeys[0];
        let v1 = vkeys[1];
        let v2 = vkeys[2];
        let v3 = vkeys[3];
        let v4 = vkeys[4];
        let fkeys = mesh.faces();
        let f0 = fkeys[0];
        let f1 = fkeys[1];

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

        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(-1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let mesh = Mesh::from_vertices_and_faces(pts, vec![vec![0, 1, 3], vec![0, 3, 2]]);
        let vkeys = mesh.vertices();
        let v0 = vkeys[0];
        let v1 = vkeys[1];
        let v3 = vkeys[3];
        let f0 = mesh.faces()[0];

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

        // area
        let box_ = Mesh::create_box(2.0, 2.0, 2.0);
        MINI_CHECK!(TOLERANCE.is_close(box_.area(), 24.0));

        // volume
        MINI_CHECK!(TOLERANCE.is_close(box_.volume(), 8.0));
    })
}

pub fn run_mesh_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::Mesh;
        use crate::Point;
        use crate::Xform;

        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let mesh = Mesh::from_vertices_and_faces(pts, vec![vec![0, 1, 2]]);
        let v0 = mesh.vertices()[0];

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

        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.name = "test_mesh".to_string();
        mesh.color_mode = crate::mesh::ColorMode::FACECOLORS;

        // JSON object
        use crate::Xform;
        mesh.xform = Xform::translation(1.0, 2.0, 3.0);
        let json = mesh.jsondump();
        let loaded_json = Mesh::jsonload(&json).unwrap();
        MINI_CHECK!(loaded_json.name == mesh.name);
        MINI_CHECK!(loaded_json.color_mode == mesh.color_mode);
        MINI_CHECK!(loaded_json.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK!(loaded_json.number_of_faces() == mesh.number_of_faces());
        MINI_CHECK!(loaded_json.xform == mesh.xform);

        // String
        let json_string = mesh.json_dumps();
        let loaded_string = Mesh::json_loads(&json_string);
        MINI_CHECK!(loaded_string.name == mesh.name);
        MINI_CHECK!(loaded_string.number_of_vertices() == mesh.number_of_vertices());

        // File
        let filename = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("serialization").join("test_mesh.json");
        mesh.json_dump(filename.to_str().unwrap()).unwrap();
        let loaded_file = Mesh::json_load(filename.to_str().unwrap()).unwrap();
        MINI_CHECK!(loaded_file.name == mesh.name);
        MINI_CHECK!(loaded_file.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK!(loaded_file.number_of_faces() == mesh.number_of_faces());

        // Triangulation roundtrip
        let polys = vec![vec![Point::new(0.0,0.0,0.0), Point::new(1.0,0.0,0.0), Point::new(1.0,1.0,0.0), Point::new(0.0,1.0,0.0)]];
        let pmesh = Mesh::from_polylines(polys, None);
        MINI_CHECK!(!pmesh.triangulation.is_empty());
        let pjson = pmesh.jsondump();
        let loaded_tri = Mesh::jsonload(&pjson).unwrap();
        let fk = *pmesh.triangulation.keys().next().unwrap();
        MINI_CHECK!(!loaded_tri.triangulation.is_empty());
        MINI_CHECK!(loaded_tri.triangulation.contains_key(&fk));

        // Face holes roundtrip
        let hmesh = Mesh::from_polygon_with_holes(&[
            vec![Point::new(0.0,0.0,0.0), Point::new(4.0,0.0,0.0), Point::new(4.0,4.0,0.0), Point::new(0.0,4.0,0.0)],
            vec![Point::new(1.0,1.0,0.0), Point::new(3.0,1.0,0.0), Point::new(3.0,3.0,0.0), Point::new(1.0,3.0,0.0)],
        ], true);
        MINI_CHECK!(!hmesh.face_holes.is_empty());
        let loaded_holes = Mesh::jsonload(&hmesh.jsondump()).unwrap();
        let hfk = *hmesh.face_holes.keys().next().unwrap();
        MINI_CHECK!(!loaded_holes.face_holes.is_empty());
        MINI_CHECK!(loaded_holes.face_holes[&hfk] == hmesh.face_holes[&hfk]);
    })
}

pub fn run_mesh_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Mesh;
        use crate::Point;
        use std::path::PathBuf;

        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.name = "test_mesh_proto".to_string();
        mesh.color_mode = crate::mesh::ColorMode::FACECOLORS;

        // String
        let proto_bytes = mesh.pb_dumps();
        let loaded_string = Mesh::pb_loads(&proto_bytes).unwrap();
        MINI_CHECK!(loaded_string.name == mesh.name);
        MINI_CHECK!(loaded_string.color_mode == mesh.color_mode);
        MINI_CHECK!(loaded_string.number_of_vertices() == mesh.number_of_vertices());

        // File
        let filename = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("serialization").join("test_mesh.bin");
        mesh.pb_dump(filename.to_str().unwrap());
        let loaded_file = Mesh::pb_load(filename.to_str().unwrap());
        MINI_CHECK!(loaded_file.name == mesh.name);
        MINI_CHECK!(loaded_file.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK!(loaded_file.number_of_faces() == mesh.number_of_faces());
        MINI_CHECK!(loaded_file.guid == mesh.guid);

        // Triangulation roundtrip
        let polys = vec![vec![Point::new(0.0,0.0,0.0), Point::new(1.0,0.0,0.0), Point::new(1.0,1.0,0.0), Point::new(0.0,1.0,0.0)]];
        let pmesh = Mesh::from_polylines(polys, None);
        MINI_CHECK!(!pmesh.triangulation.is_empty());
        let loaded_tri = Mesh::pb_loads(&pmesh.pb_dumps()).unwrap();
        let fk = *pmesh.triangulation.keys().next().unwrap();
        MINI_CHECK!(!loaded_tri.triangulation.is_empty());
        MINI_CHECK!(loaded_tri.triangulation.contains_key(&fk));

        // Face holes roundtrip
        let hmesh = Mesh::from_polygon_with_holes(&[
            vec![Point::new(0.0,0.0,0.0), Point::new(4.0,0.0,0.0), Point::new(4.0,4.0,0.0), Point::new(0.0,4.0,0.0)],
            vec![Point::new(1.0,1.0,0.0), Point::new(3.0,1.0,0.0), Point::new(3.0,3.0,0.0), Point::new(1.0,3.0,0.0)],
        ], true);
        MINI_CHECK!(!hmesh.face_holes.is_empty());
        let loaded_holes = Mesh::pb_loads(&hmesh.pb_dumps()).unwrap();
        let hfk = *hmesh.face_holes.keys().next().unwrap();
        MINI_CHECK!(!loaded_holes.face_holes.is_empty());
        MINI_CHECK!(loaded_holes.face_holes[&hfk] == hmesh.face_holes[&hfk]);
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
REGISTER_MINI_TEST!("Mesh", "Loft with quads and triangles", crate::mesh_test::run_mesh_loft_panels);
REGISTER_MINI_TEST!("Mesh", "Boolean Queries", crate::mesh_test::run_mesh_boolean_queries);
REGISTER_MINI_TEST!("Mesh", "Attributes", crate::mesh_test::run_mesh_attributes);
REGISTER_MINI_TEST!("Mesh", "Edges", crate::mesh_test::run_mesh_edges);
REGISTER_MINI_TEST!("Mesh", "Vertex and Face Operations", crate::mesh_test::run_mesh_vertex_and_face_operations);
REGISTER_MINI_TEST!("Mesh", "Connectivity Queries", crate::mesh_test::run_mesh_connectivity_queries);
REGISTER_MINI_TEST!("Mesh", "Geometric Properties", crate::mesh_test::run_mesh_geometric_properties);
REGISTER_MINI_TEST!("Mesh", "Transformation", crate::mesh_test::run_mesh_transformation);
REGISTER_MINI_TEST!("Mesh", "Json Roundtrip", crate::mesh_test::run_mesh_json_roundtrip);
REGISTER_MINI_TEST!("Mesh", "Protobuf Roundtrip", crate::mesh_test::run_mesh_protobuf_roundtrip);
