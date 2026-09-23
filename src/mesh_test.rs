use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_mesh_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::mesh::ColorMode;
        use crate::Color;
        use crate::Mesh;
        use crate::Polyline;

        let vertices = Polyline::from_sides(6, 1.0, false).get_points();
        let mut mesh = Mesh::from_vertices_and_faces(vertices, vec![vec![0, 1, 2, 3, 4, 5]]);
        let sstr = mesh.str();
        let srepr = mesh.repr();
        let mcopy = mesh.clone();

        MINI_CHECK!(!sstr.is_empty());
        MINI_CHECK!(!srepr.is_empty());
        MINI_CHECK!(mcopy == mesh);

        MINI_CHECK!(mesh.is_valid());
        mesh.name = "hexagon".to_string();

        let palette = Color::palette();

        mesh.set_objectcolor(Color::grey());

        MINI_CHECK!(mesh.color_mode == ColorMode::OBJECTCOLOR);

        let mut pc: Vec<Color> = Vec::with_capacity(mesh.number_of_vertices());

        for i in 0..mesh.number_of_vertices() {
            pc.push(palette[i % palette.len()].clone());
        }

        mesh.set_pointcolors(pc);

        MINI_CHECK!(mesh.color_mode == ColorMode::POINTCOLORS);
        MINI_CHECK!(mesh.get_pointcolors().len() == mesh.number_of_vertices());

        let mut fc: Vec<Color> = Vec::with_capacity(mesh.number_of_faces());

        for i in 0..mesh.number_of_faces() {
            fc.push(palette[i % palette.len()].clone());
        }

        mesh.set_facecolors(fc);

        MINI_CHECK!(mesh.color_mode == ColorMode::FACECOLORS);
        MINI_CHECK!(mesh.get_facecolors().len() == mesh.number_of_faces());

        let mut lc: Vec<Color> = Vec::with_capacity(mesh.number_of_edges());
        let lw: Vec<f64> = vec![0.1; mesh.number_of_edges()];

        for i in 0..mesh.number_of_edges() {
            lc.push(palette[i % palette.len()].clone());
        }

        mesh.set_linecolors(lc, lw);

        MINI_CHECK!(mesh.color_mode == ColorMode::FACECOLORS);
        MINI_CHECK!(mesh.get_linecolors().len() == mesh.number_of_edges());

        mesh.color_mode = ColorMode::FACECOLORS;

        MINI_CHECK!(mesh.color_mode == ColorMode::FACECOLORS);
        mesh.clear_facecolors();

        MINI_CHECK!(mesh.color_mode == ColorMode::OBJECTCOLOR);
        MINI_CHECK!(mesh.get_facecolors().is_empty());

        mesh.color_mode = ColorMode::FACECOLORS;

        MINI_CHECK!(mesh.color_mode == ColorMode::FACECOLORS);
        mesh.clear_pointcolors();

        MINI_CHECK!(mesh.color_mode == ColorMode::FACECOLORS);

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

        let mesh = Mesh::from_polylines(
            vec![
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
            ],
            Some(0.001),
        );

        MINI_CHECK!(mesh.is_valid());
    })
}

pub fn run_mesh_from_polylines_polyline() -> TestResult {
    MINI_TEST!("From Polylines Polyline", {
        use crate::Mesh;
        use crate::Point;
        use crate::Polyline;

        let mesh = Mesh::from_polylines_polyline(
            &[
                Polyline::new(vec![
                    Point::new(1.28955, 0.0, 1.127558),
                    Point::new(0.85791, 0.0, 0.225512),
                    Point::new(0.64209, -0.866025, -0.225512),
                    Point::new(0.85791, -1.732051, 0.225512),
                    Point::new(1.458565, -1.732051, 1.127558),
                    Point::new(1.50537, -0.866025, 1.578581),
                ]),
                Polyline::new(vec![
                    Point::new(0.64209, 0.866025, -0.225512),
                    Point::new(0.114274, 0.866025, -0.686294),
                    Point::new(-0.00537, 0.0, -1.578581),
                    Point::new(0.21045, -0.866025, -1.127558),
                    Point::new(0.64209, -0.866025, -0.225512),
                    Point::new(0.85791, 0.0, 0.225512),
                ]),
                Polyline::new(vec![
                    Point::new(1.28955, 1.732051, 1.127558),
                    Point::new(0.85791, 1.732051, 0.225512),
                    Point::new(0.64209, 0.866025, -0.225512),
                    Point::new(0.85791, 0.0, 0.225512),
                    Point::new(1.28955, 0.0, 1.127558),
                    Point::new(1.853404, 0.866025, 1.578581),
                ]),
            ],
            Some(0.001),
        );

        MINI_CHECK!(mesh.is_valid());
    })
}

pub fn run_mesh_from_lines() -> TestResult {
    MINI_TEST!("From Lines", {
        use crate::Line;
        use crate::Mesh;
        use crate::Point;

        let lines = vec![
            Line::from_points(
                &Point::new(4.948083, -0.149798, 1.00765),
                &Point::new(4.395544, -0.996413, 1.196018),
            ),
            Line::from_points(
                &Point::new(3.866593, 0.371225, 1.376346),
                &Point::new(4.567265, 0.584361, 1.137476),
            ),
            Line::from_points(
                &Point::new(3.915298, -0.157402, 1.359741),
                &Point::new(3.282977, -0.051356, 1.575309),
            ),
            Line::from_points(
                &Point::new(4.286215, -0.224964, 1.23329),
                &Point::new(3.607284, -0.987075, 1.464748),
            ),
            Line::from_points(
                &Point::new(3.744351, 0.971574, 1.41802),
                &Point::new(3.266367, 0.841359, 1.580972),
            ),
            Line::from_points(
                &Point::new(4.567265, 0.584361, 1.137476),
                &Point::new(4.948083, -0.149798, 1.00765),
            ),
            Line::from_points(
                &Point::new(4.395544, -0.996413, 1.196018),
                &Point::new(3.607284, -0.987075, 1.464748),
            ),
            Line::from_points(
                &Point::new(3.915298, -0.157402, 1.359741),
                &Point::new(4.286215, -0.224964, 1.23329),
            ),
            Line::from_points(
                &Point::new(3.282977, -0.051356, 1.575309),
                &Point::new(3.266367, 0.841359, 1.580972),
            ),
            Line::from_points(
                &Point::new(3.744351, 0.971574, 1.41802),
                &Point::new(3.866593, 0.371225, 1.376346),
            ),
        ];
        let mesh = Mesh::from_lines(&lines, true, None);

        MINI_CHECK!(mesh.is_valid());
    })
}

pub fn run_mesh_from_polygon_with_holes() -> TestResult {
    MINI_TEST!("From Polygon With Holes", {
        use crate::Mesh;
        use crate::Point;

        let mesh = Mesh::from_polygon_with_holes(
            &[
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
            ],
            true,
        );

        MINI_CHECK!(mesh.is_valid());

        let mesh_sorted = Mesh::from_polygon_with_holes(
            &[
                vec![
                    Point::new(1.0, 1.0, 0.0),
                    Point::new(3.0, 1.0, 0.0),
                    Point::new(3.0, 3.0, 0.0),
                    Point::new(1.0, 3.0, 0.0),
                ],
                vec![
                    Point::new(0.0, 0.0, 0.0),
                    Point::new(4.0, 0.0, 0.0),
                    Point::new(4.0, 4.0, 0.0),
                    Point::new(0.0, 4.0, 0.0),
                ],
            ],
            true,
        );

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
        let mesh = Mesh::loft(&bottom, &top, true, true);

        MINI_CHECK!(mesh.is_valid());
        MINI_CHECK!(mesh.is_closed());

        let mesh_no_cap = Mesh::loft(&bottom, &top, false, true);

        MINI_CHECK!(mesh_no_cap.is_valid());
        MINI_CHECK!(!mesh_no_cap.is_closed());
    })
}

pub fn run_mesh_loft_concave_with_holes_and_collinear() -> TestResult {
    MINI_TEST!("Loft Concave With Holes", {
        use crate::Mesh;
        use crate::Point;
        use crate::Polyline;

        let annen_bot = vec![
            Polyline::new(vec![
                Point::new(2142.008, -530.170, 1172.487),
                Point::new(2142.008, -530.170, -318.768),
                Point::new(2142.008, -318.102, -318.768),
                Point::new(2142.008, -347.792, -414.110),
                Point::new(2142.008, -106.034, -414.110),
                Point::new(2142.008, -135.724, -318.768),
                Point::new(2142.008, 106.034, -318.768),
                Point::new(2142.008, 76.344, -414.110),
                Point::new(2142.008, 318.102, -414.110),
                Point::new(2142.008, 288.412, -318.768),
                Point::new(2142.008, 530.170, -318.768),
                Point::new(2142.008, 530.170, 1172.487),
                Point::new(2142.008, -530.170, 1172.487),
            ]),
            Polyline::new(vec![
                Point::new(2142.008, 97.448, 841.097),
                Point::new(2142.008, 0.000, 841.097),
                Point::new(2142.008, 0.000, 1006.792),
                Point::new(2142.008, 97.448, 1006.792),
                Point::new(2142.008, 97.448, 841.097),
            ]),
            Polyline::new(vec![
                Point::new(2142.008, 97.448, 178.317),
                Point::new(2142.008, 0.000, 178.317),
                Point::new(2142.008, 0.000, 344.012),
                Point::new(2142.008, 97.448, 344.012),
                Point::new(2142.008, 97.448, 178.317),
            ]),
        ];
        let annen_top = vec![
            Polyline::new(vec![
                Point::new(2223.416, -530.170, 1172.487),
                Point::new(2223.416, -530.170, -269.141),
                Point::new(2223.416, -318.102, -269.141),
                Point::new(2223.416, -347.792, -364.483),
                Point::new(2223.416, -106.034, -364.483),
                Point::new(2223.416, -135.724, -269.141),
                Point::new(2223.416, 106.034, -269.141),
                Point::new(2223.416, 76.344, -364.483),
                Point::new(2223.416, 318.102, -364.483),
                Point::new(2223.416, 288.412, -269.141),
                Point::new(2223.416, 530.170, -269.141),
                Point::new(2223.416, 530.170, 1172.487),
                Point::new(2223.416, -530.170, 1172.487),
            ]),
            Polyline::new(vec![
                Point::new(2223.416, 97.448, 841.097),
                Point::new(2223.416, 0.000, 841.097),
                Point::new(2223.416, 0.000, 1006.792),
                Point::new(2223.416, 97.448, 1006.792),
                Point::new(2223.416, 97.448, 841.097),
            ]),
            Polyline::new(vec![
                Point::new(2223.416, 97.448, 178.317),
                Point::new(2223.416, 0.000, 178.317),
                Point::new(2223.416, 0.000, 344.012),
                Point::new(2223.416, 97.448, 344.012),
                Point::new(2223.416, 97.448, 178.317),
            ]),
        ];
        let annen = Mesh::loft(&annen_bot, &annen_top, true, true);

        MINI_CHECK!(annen.is_valid());
        MINI_CHECK!(annen.is_closed());
        MINI_CHECK!(annen.vertex.len() == 40);
        MINI_CHECK!(annen.face.len() == 22);

        let col_bot = vec![Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(7.0, 0.0, 0.0),
            Point::new(12.0, 0.0, 0.0),
            Point::new(12.0, 5.0, 0.0),
            Point::new(0.0, 5.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ])];
        let col_top = vec![Polyline::new(vec![
            Point::new(0.0, 0.0, 1.5),
            Point::new(4.0, 0.0, 1.5),
            Point::new(7.0, 0.0, 1.5),
            Point::new(12.0, 0.0, 1.5),
            Point::new(12.0, 5.0, 1.5),
            Point::new(0.0, 5.0, 1.5),
            Point::new(0.0, 0.0, 1.5),
        ])];
        let colmesh = Mesh::loft(&col_bot, &col_top, true, true);

        MINI_CHECK!(colmesh.is_valid());
        MINI_CHECK!(colmesh.is_closed());
        MINI_CHECK!(colmesh.vertex.len() == 12);
        MINI_CHECK!(colmesh.face.len() == 8);
    })
}

pub fn run_mesh_from_polygon_with_holes_many() -> TestResult {
    MINI_TEST!("From Polygon With Holes Many", {
        use crate::Mesh;
        use crate::Point;

        let mut inputs: Vec<Vec<Vec<Point>>> = Vec::new();

        for i in 0..4 {
            let x = i as f64 * 7.0;
            inputs.push(vec![
                vec![
                    Point::new(x, 0.0, 0.0),
                    Point::new(x + 5.0, 0.0, 0.0),
                    Point::new(x + 5.0, 5.0, 0.0),
                    Point::new(x, 5.0, 0.0),
                ],
                vec![
                    Point::new(x + 1.0, 1.0, 0.0),
                    Point::new(x + 4.0, 1.0, 0.0),
                    Point::new(x + 4.0, 4.0, 0.0),
                    Point::new(x + 1.0, 4.0, 0.0),
                ],
            ]);
        }

        let meshes = Mesh::from_polygon_with_holes_many(inputs.clone(), false, true);

        MINI_CHECK!(meshes[0].is_valid());
        MINI_CHECK!(meshes[1].is_valid());
        MINI_CHECK!(meshes[2].is_valid());
        MINI_CHECK!(meshes[3].is_valid());
        let meshes_seq = Mesh::from_polygon_with_holes_many(inputs, false, false);

        MINI_CHECK!(meshes_seq[0].number_of_faces() == meshes[0].number_of_faces());
    })
}

pub fn run_mesh_loft_many() -> TestResult {
    MINI_TEST!("Loft Many", {
        use crate::Mesh;
        use crate::Point;
        use crate::Polyline;

        let mut loft_inputs: Vec<(Vec<Polyline>, Vec<Polyline>)> = Vec::new();

        for i in 0..6 {
            let x = i as f64 * 3.0;
            let b = Polyline::new(vec![
                Point::new(x, 0.0, 0.0),
                Point::new(x + 1.0, 0.0, 0.0),
                Point::new(x + 1.0, 1.0, 0.0),
                Point::new(x, 1.0, 0.0),
                Point::new(x, 0.0, 0.0),
            ]);
            let t = Polyline::new(vec![
                Point::new(x, 0.0, 1.0 + i as f64 * 0.5),
                Point::new(x + 1.0, 0.0, 1.0 + i as f64 * 0.5),
                Point::new(x + 1.0, 1.0, 1.0 + i as f64 * 0.5),
                Point::new(x, 1.0, 1.0 + i as f64 * 0.5),
                Point::new(x, 0.0, 1.0 + i as f64 * 0.5),
            ]);
            loft_inputs.push((vec![b], vec![t]));
        }

        let meshes = Mesh::loft_many(loft_inputs.clone(), true, true, true);

        MINI_CHECK!(meshes[0].is_valid());
        MINI_CHECK!(meshes[0].is_closed());
        MINI_CHECK!(meshes[1].is_valid());
        MINI_CHECK!(meshes[1].is_closed());
        MINI_CHECK!(meshes[2].is_valid());
        MINI_CHECK!(meshes[2].is_closed());
        MINI_CHECK!(meshes[3].is_valid());
        MINI_CHECK!(meshes[3].is_closed());
        MINI_CHECK!(meshes[4].is_valid());
        MINI_CHECK!(meshes[4].is_closed());
        MINI_CHECK!(meshes[5].is_valid());
        MINI_CHECK!(meshes[5].is_closed());
        let meshes_seq = Mesh::loft_many(loft_inputs, true, false, true);

        MINI_CHECK!(meshes_seq[0].is_valid());
        MINI_CHECK!(meshes_seq[0].is_closed());
        MINI_CHECK!(meshes_seq[1].is_valid());
        MINI_CHECK!(meshes_seq[1].is_closed());
        MINI_CHECK!(meshes_seq[2].is_valid());
        MINI_CHECK!(meshes_seq[2].is_closed());
        MINI_CHECK!(meshes_seq[3].is_valid());
        MINI_CHECK!(meshes_seq[3].is_closed());
        MINI_CHECK!(meshes_seq[4].is_valid());
        MINI_CHECK!(meshes_seq[4].is_closed());
        MINI_CHECK!(meshes_seq[5].is_valid());
        MINI_CHECK!(meshes_seq[5].is_closed());
    })
}

pub fn run_mesh_loft_panels() -> TestResult {
    MINI_TEST!("Loft With Quads And Triangles", {
        use crate::mesh::LoftFaceRole;
        use crate::mesh::LoftResult;
        use crate::Color;
        use crate::Mesh;
        use crate::Point;

        let top7: Vec<Vec<Point>> = vec![
            vec![
                Point::new(250., -250., 500.),
                Point::new(250., 250., 500.),
                Point::new(-250., 250., 500.),
                Point::new(-250., -250., 500.),
                Point::new(250., -250., 500.),
            ],
            vec![
                Point::new(-250., 500., 250.),
                Point::new(-250., 250., 500.),
                Point::new(250., 250., 500.),
                Point::new(250., 500., 250.),
                Point::new(-250., 500., 250.),
            ],
            vec![
                Point::new(250., -250., 500.),
                Point::new(500., -250., 250.),
                Point::new(500., 250., 250.),
                Point::new(250., 250., 500.),
                Point::new(250., -250., 500.),
            ],
            vec![
                Point::new(250., 500., 250.),
                Point::new(250., 250., 500.),
                Point::new(500., 250., 250.),
                Point::new(250., 500., 250.),
            ],
            vec![
                Point::new(-250., 500., 250.),
                Point::new(250., 500., 250.),
                Point::new(250., 500., -250.),
                Point::new(-250., 500., -250.),
                Point::new(-250., 500., 250.),
            ],
            vec![
                Point::new(250., 500., 250.),
                Point::new(500., 250., 250.),
                Point::new(500., 250., -250.),
                Point::new(250., 500., -250.),
                Point::new(250., 500., 250.),
            ],
            vec![
                Point::new(500., -250., 250.),
                Point::new(500., -250., -250.),
                Point::new(500., 250., -250.),
                Point::new(500., 250., 250.),
                Point::new(500., -250., 250.),
            ],
        ];
        let bot7: Vec<Vec<Point>> = vec![
            vec![
                Point::new(270.710678, -250., 550.),
                Point::new(270.710678, 265.891862, 550.),
                Point::new(265.891862, 270.710678, 550.),
                Point::new(-250., 270.710678, 550.),
                Point::new(-250., -250., 550.),
                Point::new(270.710678, -250., 550.),
            ],
            vec![
                Point::new(270.710678, -250., 550.),
                Point::new(550., -250., 270.710678),
                Point::new(550., 265.891862, 270.710678),
                Point::new(270.710678, 265.891862, 550.),
                Point::new(270.710678, -250., 550.),
            ],
            vec![
                Point::new(-250., 550., 270.710678),
                Point::new(-250., 270.710678, 550.),
                Point::new(265.891862, 270.710678, 550.),
                Point::new(265.891862, 550., 270.710678),
                Point::new(-250., 550., 270.710678),
            ],
            vec![
                Point::new(265.891862, 550., 270.710678),
                Point::new(265.891862, 270.710678, 550.),
                Point::new(270.710678, 265.891862, 550.),
                Point::new(550., 265.891862, 270.710678),
                Point::new(550., 270.710678, 265.891862),
                Point::new(270.710678, 550., 265.891862),
                Point::new(265.891862, 550., 270.710678),
            ],
            vec![
                Point::new(-250., 550., 270.710678),
                Point::new(265.891862, 550., 270.710678),
                Point::new(270.710678, 550., 265.891862),
                Point::new(270.710678, 550., -250.),
                Point::new(-250., 550., -250.),
                Point::new(-250., 550., 270.710678),
            ],
            vec![
                Point::new(270.710678, 550., 265.891862),
                Point::new(550., 270.710678, 265.891862),
                Point::new(550., 270.710678, -250.),
                Point::new(270.710678, 550., -250.),
                Point::new(270.710678, 550., 265.891862),
            ],
            vec![
                Point::new(550., -250., 270.710678),
                Point::new(550., -250., -250.),
                Point::new(550., 270.710678, -250.),
                Point::new(550., 270.710678, 265.891862),
                Point::new(550., 265.891862, 270.710678),
                Point::new(550., -250., 270.710678),
            ],
        ];
        let LoftResult {
            mut panels,
            adjacency: adj,
            top_mesh: _top_mesh,
            bot_mesh: _bot_mesh,
        } = Mesh::loft_panels(top7, bot7, 0.001, 0.0, 2.0, true, false);

        for panel in panels.iter_mut() {
            let mut face_colors: Vec<Color> = Vec::new();

            for role in panel.face_roles.values() {
                let color = match *role {
                    LoftFaceRole::TopCap => Color::blue(),
                    LoftFaceRole::BotCap => Color::red(),
                    LoftFaceRole::TriWall => Color::yellow(),
                    LoftFaceRole::QuadWall => Color::grey(),
                };
                face_colors.push(color);
            }

            panel.mesh.set_facecolors(face_colors);
        }

        for (i, panel) in panels.iter().enumerate() {
            let mut c = panel.mesh.centroid();
            c.name = format!("p{}", i);
        }

        for pair in &adj {
            let w = &panels[pair.pi].wall_faces[pair.wi];
            let mut pt = panels[pair.pi].mesh.face_centroid(w.face_key).unwrap();
            pt.name = format!(
                "p{} f{} - p{} f{}",
                pair.pi, w.face_index, pair.pj, panels[pair.pj].wall_faces[pair.wj].face_index
            );
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

        let mesh = Mesh::from_polylines(
            vec![
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
            ],
            Some(0.001),
        );
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
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[1], &Point::new(0.5, -0.5, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[2], &Point::new(0.5, 0.5, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[3], &Point::new(-0.5, 0.5, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[4], &Point::new(-0.5, -0.5, 0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[5], &Point::new(0.5, -0.5, 0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[6], &Point::new(0.5, 0.5, 0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[7], &Point::new(-0.5, 0.5, 0.5)));
        MINI_CHECK!(fidx[0] == vec![0, 3, 2, 1]);
        MINI_CHECK!(fidx[1] == vec![4, 5, 6, 7]);
        MINI_CHECK!(fidx[2] == vec![0, 1, 5, 4]);
        MINI_CHECK!(fidx[3] == vec![2, 3, 7, 6]);
        MINI_CHECK!(fidx[4] == vec![0, 4, 7, 3]);
        MINI_CHECK!(fidx[5] == vec![1, 2, 6, 5]);

        let vertex_to_index = mesh.vertex_index();

        MINI_CHECK!(vertex_to_index.len() == n_vertices);
        MINI_CHECK!(vertex_to_index[&0] == 0);
        MINI_CHECK!(vertex_to_index[&1] == 1);
        MINI_CHECK!(vertex_to_index[&2] == 2);
        MINI_CHECK!(vertex_to_index[&3] == 3);
        MINI_CHECK!(vertex_to_index[&4] == 4);
        MINI_CHECK!(vertex_to_index[&5] == 5);
        MINI_CHECK!(vertex_to_index[&6] == 6);
        MINI_CHECK!(vertex_to_index[&7] == 7);

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
        MINI_CHECK!(edges[0] == (0, 1));
        MINI_CHECK!(edges[1] == (0, 3));
        MINI_CHECK!(edges[2] == (0, 4));
        MINI_CHECK!(edges[3] == (1, 2));
        MINI_CHECK!(edges[4] == (1, 5));
        MINI_CHECK!(edges[5] == (2, 3));
        MINI_CHECK!(edges[6] == (2, 6));
        MINI_CHECK!(edges[7] == (3, 7));
        MINI_CHECK!(edges[8] == (4, 5));
        MINI_CHECK!(edges[9] == (4, 7));
        MINI_CHECK!(edges[10] == (5, 6));
        MINI_CHECK!(edges[11] == (6, 7));

        MINI_CHECK!(mesh.naked_edges(true).is_empty());
        MINI_CHECK!(mesh.naked_faces(false).len() == 6);
        mesh.remove_face(mesh.faces()[0]);
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

pub fn run_mesh_create_dodecahedron() -> TestResult {
    MINI_TEST!("Create Dodecahedron", {
        use crate::Mesh;

        let m = Mesh::create_dodecahedron(2.0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 20);
        MINI_CHECK!(m.number_of_faces() == 12);
    })
}

pub fn run_mesh_vertex_and_face_operations() -> TestResult {
    MINI_TEST!("Vertex And Face Operations", {
        use crate::Mesh;
        use crate::Point;

        let hx = 0.5_f64;
        let hy = 0.5_f64;
        let hz = 0.5_f64;
        let verts = vec![
            Point::new(-hx, -hy, -hz),
            Point::new(hx, -hy, -hz),
            Point::new(hx, hy, -hz),
            Point::new(-hx, hy, -hz),
            Point::new(-hx, -hy, hz),
            Point::new(hx, -hy, hz),
            Point::new(hx, hy, hz),
            Point::new(-hx, hy, hz),
        ];
        let faces: Vec<Vec<usize>> = vec![
            vec![0, 3, 2, 1],
            vec![4, 5, 6, 7],
            vec![0, 1, 5, 4],
            vec![2, 3, 7, 6],
            vec![0, 4, 7, 3],
            vec![1, 2, 6, 5],
        ];

        let mut mesh = Mesh::new();

        for v in &verts {
            mesh.add_vertex(v.clone(), None);
        }

        for f in &faces {
            mesh.add_face(f.clone(), None);
        }

        MINI_CHECK!(mesh.add_face(vec![0, 1], None).is_none());
        MINI_CHECK!(mesh.add_face(vec![0, 1, 0], None).is_none());

        mesh.remove_vertex(0);

        MINI_CHECK!(mesh.number_of_vertices() == 7);
        MINI_CHECK!(mesh.number_of_faces() == 3);

        mesh.remove_edge(1, 2);

        MINI_CHECK!(mesh.number_of_faces() == 2);

        mesh.remove_face(1);

        MINI_CHECK!(mesh.number_of_faces() == 1);

        mesh.clear();

        MINI_CHECK!(mesh.is_empty());

        for v in &verts {
            mesh.add_vertex(v.clone(), None);
        }

        for f in &faces {
            mesh.add_face(f.clone(), None);
        }

        mesh = mesh.unweld();

        MINI_CHECK!(mesh.number_of_vertices() == 24);
        mesh = mesh.weld(0.001);

        MINI_CHECK!(mesh.number_of_vertices() == 8);
        MINI_CHECK!(mesh.number_of_faces() == 6);
        let fv0 = mesh.face_vertices(0).unwrap();
        let fv1 = mesh.face_vertices(1).unwrap();
        let fv2 = mesh.face_vertices(2).unwrap();
        let fv3 = mesh.face_vertices(3).unwrap();
        let fv4 = mesh.face_vertices(4).unwrap();
        let fv5 = mesh.face_vertices(5).unwrap();

        MINI_CHECK!(fv0[0] == 0 && fv0[1] == 1 && fv0[2] == 2 && fv0[3] == 3);
        MINI_CHECK!(fv1[0] == 4 && fv1[1] == 5 && fv1[2] == 6 && fv1[3] == 7);
        MINI_CHECK!(fv2[0] == 0 && fv2[1] == 3 && fv2[2] == 5 && fv2[3] == 4);
        MINI_CHECK!(fv3[0] == 2 && fv3[1] == 1 && fv3[2] == 7 && fv3[3] == 6);
        MINI_CHECK!(fv4[0] == 0 && fv4[1] == 4 && fv4[2] == 7 && fv4[3] == 1);
        MINI_CHECK!(fv5[0] == 3 && fv5[1] == 2 && fv5[2] == 6 && fv5[3] == 5);

        mesh.flip_face(0);
        let fv0 = mesh.face_vertices(0).unwrap();
        let fv1 = mesh.face_vertices(1).unwrap();
        let fv2 = mesh.face_vertices(2).unwrap();
        let fv3 = mesh.face_vertices(3).unwrap();
        let fv4 = mesh.face_vertices(4).unwrap();
        let fv5 = mesh.face_vertices(5).unwrap();

        MINI_CHECK!(fv0[0] == 3 && fv0[1] == 2 && fv0[2] == 1 && fv0[3] == 0);
        MINI_CHECK!(fv1[0] == 4 && fv1[1] == 5 && fv1[2] == 6 && fv1[3] == 7);
        MINI_CHECK!(fv2[0] == 0 && fv2[1] == 3 && fv2[2] == 5 && fv2[3] == 4);
        MINI_CHECK!(fv3[0] == 2 && fv3[1] == 1 && fv3[2] == 7 && fv3[3] == 6);
        MINI_CHECK!(fv4[0] == 0 && fv4[1] == 4 && fv4[2] == 7 && fv4[3] == 1);
        MINI_CHECK!(fv5[0] == 3 && fv5[1] == 2 && fv5[2] == 6 && fv5[3] == 5);

        mesh.unify_winding();
        let fv0 = mesh.face_vertices(0).unwrap();
        let fv1 = mesh.face_vertices(1).unwrap();
        let fv2 = mesh.face_vertices(2).unwrap();
        let fv3 = mesh.face_vertices(3).unwrap();
        let fv4 = mesh.face_vertices(4).unwrap();
        let fv5 = mesh.face_vertices(5).unwrap();

        MINI_CHECK!(fv0[0] == 0 && fv0[1] == 1 && fv0[2] == 2 && fv0[3] == 3);
        MINI_CHECK!(fv1[0] == 4 && fv1[1] == 5 && fv1[2] == 6 && fv1[3] == 7);
        MINI_CHECK!(fv2[0] == 0 && fv2[1] == 3 && fv2[2] == 5 && fv2[3] == 4);
        MINI_CHECK!(fv3[0] == 2 && fv3[1] == 1 && fv3[2] == 7 && fv3[3] == 6);
        MINI_CHECK!(fv4[0] == 0 && fv4[1] == 4 && fv4[2] == 7 && fv4[3] == 1);
        MINI_CHECK!(fv5[0] == 3 && fv5[1] == 2 && fv5[2] == 6 && fv5[3] == 5);

        mesh.flip();
        let fv0 = mesh.face_vertices(0).unwrap();
        let fv1 = mesh.face_vertices(1).unwrap();
        let fv2 = mesh.face_vertices(2).unwrap();
        let fv3 = mesh.face_vertices(3).unwrap();
        let fv4 = mesh.face_vertices(4).unwrap();
        let fv5 = mesh.face_vertices(5).unwrap();

        MINI_CHECK!(fv0[0] == 3 && fv0[1] == 2 && fv0[2] == 1 && fv0[3] == 0);
        MINI_CHECK!(fv1[0] == 7 && fv1[1] == 6 && fv1[2] == 5 && fv1[3] == 4);
        MINI_CHECK!(fv2[0] == 4 && fv2[1] == 5 && fv2[2] == 3 && fv2[3] == 0);
        MINI_CHECK!(fv3[0] == 6 && fv3[1] == 7 && fv3[2] == 1 && fv3[3] == 2);
        MINI_CHECK!(fv4[0] == 1 && fv4[1] == 7 && fv4[2] == 4 && fv4[3] == 0);
        MINI_CHECK!(fv5[0] == 5 && fv5[1] == 6 && fv5[2] == 2 && fv5[3] == 3);

        mesh.orient_outward();
        let fv0 = mesh.face_vertices(0).unwrap();
        let fv1 = mesh.face_vertices(1).unwrap();
        let fv2 = mesh.face_vertices(2).unwrap();
        let fv3 = mesh.face_vertices(3).unwrap();
        let fv4 = mesh.face_vertices(4).unwrap();
        let fv5 = mesh.face_vertices(5).unwrap();

        MINI_CHECK!(fv0[0] == 0 && fv0[1] == 1 && fv0[2] == 2 && fv0[3] == 3);
        MINI_CHECK!(fv1[0] == 4 && fv1[1] == 5 && fv1[2] == 6 && fv1[3] == 7);
        MINI_CHECK!(fv2[0] == 0 && fv2[1] == 3 && fv2[2] == 5 && fv2[3] == 4);
        MINI_CHECK!(fv3[0] == 2 && fv3[1] == 1 && fv3[2] == 7 && fv3[3] == 6);
        MINI_CHECK!(fv4[0] == 0 && fv4[1] == 4 && fv4[2] == 7 && fv4[3] == 1);
        MINI_CHECK!(fv5[0] == 3 && fv5[1] == 2 && fv5[2] == 6 && fv5[3] == 5);
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
        let v = mesh.vertices();
        let f = mesh.faces();

        if let Some(ee) = mesh.edge_edges(1, 2) {
            let (u0, v0) = ee[0];
            let l0 = mesh.edge_line(u0, v0).unwrap();
            let mut mid0 = l0.center();
            mid0.name = format!("e{}-{}", u0, v0);

            let (u1, v1) = ee[1];
            let l1 = mesh.edge_line(u1, v1).unwrap();
            let mut mid1 = l1.center();
            mid1.name = format!("e{}-{}", u1, v1);

            let (u2, v2) = ee[2];
            let l2 = mesh.edge_line(u2, v2).unwrap();
            let mut mid2 = l2.center();
            mid2.name = format!("e{}-{}", u2, v2);

            let (u3, v3) = ee[3];
            let l3 = mesh.edge_line(u3, v3).unwrap();
            let mut mid3 = l3.center();
            mid3.name = format!("e{}-{}", u3, v3);

            MINI_CHECK!(ee.len() == 4);
            MINI_CHECK!(ee[0] == (1, 0));
            MINI_CHECK!(ee[1] == (1, 4));
            MINI_CHECK!(ee[2] == (2, 3));
            MINI_CHECK!(ee[3] == (2, 4));
        }

        if let Some(ef) = mesh.edge_faces(1, 2) {
            let ef0 = ef[0];
            let ef1 = ef[1];
            let mut efp0 = mesh.face_centroid(ef0).unwrap();
            efp0.name = format!("f{}", ef0);
            let mut efp1 = mesh.face_centroid(ef1).unwrap();
            efp1.name = format!("f{}", ef1);

            MINI_CHECK!(ef.len() == 2);
            MINI_CHECK!(ef0 == 0 && ef1 == 1);
        }

        if let Some(fe) = mesh.face_edges(f[0]) {
            let l0 = mesh.edge_line(fe[0].0, fe[0].1).unwrap();
            let l1 = mesh.edge_line(fe[1].0, fe[1].1).unwrap();
            let l2 = mesh.edge_line(fe[2].0, fe[2].1).unwrap();
            let l3 = mesh.edge_line(fe[3].0, fe[3].1).unwrap();
            let mut lmid0 = l0.center();
            lmid0.name = format!("e{}-{}", fe[0].0, fe[0].1);
            let mut lmid1 = l1.center();
            lmid1.name = format!("e{}-{}", fe[1].0, fe[1].1);
            let mut lmid2 = l2.center();
            lmid2.name = format!("e{}-{}", fe[2].0, fe[2].1);
            let mut lmid3 = l3.center();
            lmid3.name = format!("e{}-{}", fe[3].0, fe[3].1);

            MINI_CHECK!(fe.len() == 4);
            MINI_CHECK!(fe[0] == (0, 1));
            MINI_CHECK!(fe[1] == (1, 2));
            MINI_CHECK!(fe[2] == (2, 3));
            MINI_CHECK!(fe[3] == (3, 0));
        }

        if let Some(ff) = mesh.face_faces(f[0]) {
            let ff0 = ff[0];
            let mut ffp = mesh.face_centroid(ff0).unwrap();
            ffp.name = format!("f{}", ff0);

            MINI_CHECK!(ff.len() == 1);
            MINI_CHECK!(ff0 == 1);
        }

        if let Some(points) = mesh.face_points(f[0]) {
            let pointcount = points.len();

            MINI_CHECK!(pointcount == 4);
        }

        if let Some(pl) = mesh.face_polyline(f[0]) {
            let pointcount = pl.get_points().len();

            MINI_CHECK!(pointcount == 4);
        }

        if let Some(fv) = mesh.face_vertices(f[0]) {
            let fv0 = fv[0];
            let fv1 = fv[1];
            let fv2 = fv[2];
            let fv3 = fv[3];
            let mut p0 = mesh.vertex_point(fv0).unwrap();
            p0.name = fv0.to_string();
            let mut p1 = mesh.vertex_point(fv1).unwrap();
            p1.name = fv1.to_string();
            let mut p2 = mesh.vertex_point(fv2).unwrap();
            p2.name = fv2.to_string();
            let mut p3 = mesh.vertex_point(fv3).unwrap();
            p3.name = fv3.to_string();

            MINI_CHECK!(fv0 == 0);
            MINI_CHECK!(fv1 == 1);
            MINI_CHECK!(fv2 == 2);
            MINI_CHECK!(fv3 == 3);
            MINI_CHECK!(fv.len() == 4);
        }

        if let Some(ve) = mesh.vertex_edges(v[1]) {
            let mut vp = mesh.vertex_point(v[1]).unwrap();
            vp.name = format!("v{}", v[1]);

            let l0 = mesh.edge_line(ve[0].0, ve[0].1).unwrap();
            let l1 = mesh.edge_line(ve[1].0, ve[1].1).unwrap();
            let l2 = mesh.edge_line(ve[2].0, ve[2].1).unwrap();
            let mut lmid0 = l0.center();
            lmid0.name = format!("e{}-{}", ve[0].0, ve[0].1);
            let mut lmid1 = l1.center();
            lmid1.name = format!("e{}-{}", ve[1].0, ve[1].1);
            let mut lmid2 = l2.center();
            lmid2.name = format!("e{}-{}", ve[2].0, ve[2].1);

            MINI_CHECK!(ve[0] == (1, 0));
            MINI_CHECK!(ve[1] == (1, 2));
            MINI_CHECK!(ve[2] == (1, 4));
            MINI_CHECK!(ve.len() == 3);
        }

        if let Some(vf) = mesh.vertex_faces(v[1]) {
            let mut vp = mesh.vertex_point(v[1]).unwrap();
            vp.name = format!("v{}", v[1]);

            let mut fp0 = mesh.face_centroid(vf[0]).unwrap();
            fp0.name = format!("f{}", vf[0]);
            let mut fp1 = mesh.face_centroid(vf[1]).unwrap();
            fp1.name = format!("f{}", vf[1]);

            MINI_CHECK!(vf.len() == 2);
            MINI_CHECK!(vf[0] == 0);
            MINI_CHECK!(vf[1] == 1);
        }

        if let Some(vn) = mesh.vertex_vertices(v[1]) {
            let mut p0 = mesh.vertex_point(v[1]).unwrap();
            p0.name = format!("main{}", v[1]);

            let mut np0 = mesh.vertex_point(vn[0]).unwrap();
            np0.name = vn[0].to_string();
            let mut np1 = mesh.vertex_point(vn[1]).unwrap();
            np1.name = vn[1].to_string();
            let mut np2 = mesh.vertex_point(vn[2]).unwrap();
            np2.name = vn[2].to_string();

            MINI_CHECK!(vn[0] == 0);
            MINI_CHECK!(vn[1] == 2);
            MINI_CHECK!(vn[2] == 4);
            MINI_CHECK!(vn.len() == 3);
        }
    })
}

pub fn run_mesh_geometric_properties() -> TestResult {
    MINI_TEST!("Geometric Properties", {
        use crate::mesh::NormalWeighting;
        use crate::Mesh;
        use crate::Point;
        use crate::Vector;

        let mesh = Mesh::create_dodecahedron(1.5);

        let area = mesh.area();

        MINI_CHECK!(TOLERANCE.is_close(area, 46.4528898159021));

        let centroid = mesh.centroid();

        MINI_CHECK!(TOLERANCE.is_point_close(&centroid, &Point::new(0.0, 0.0, 0.0)));

        let (angles, _arcs, _points) = mesh.dihedral_angles(0.3, true, true);

        for angle in angles.values() {
            let angle_in_degrees = *angle;

            MINI_CHECK!(TOLERANCE.is_close(angle_in_degrees, 116.565051177078));
        }

        for f in mesh.faces() {
            let face_area = mesh.face_area(f);

            MINI_CHECK!(face_area.is_some());
            MINI_CHECK!(TOLERANCE.is_close(face_area.unwrap(), 3.87107415132518));
        }

        let mut centroids = Vec::new();

        for f in mesh.faces() {
            centroids.push(mesh.face_centroid(f).unwrap());
        }

        MINI_CHECK!(TOLERANCE.is_point_close(
            &centroids[0],
            &Point::new(0.878115294937453, 0.0, 1.420820393249937)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &centroids[1],
            &Point::new(1.420820393249937, 0.878115294937453, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &centroids[2],
            &Point::new(0.0, 1.420820393249937, 0.878115294937453)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &centroids[3],
            &Point::new(0.878115294937453, 0.0, -1.420820393249937)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &centroids[4],
            &Point::new(0.0, 1.420820393249937, -0.878115294937453)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &centroids[5],
            &Point::new(0.0, -1.420820393249937, 0.878115294937453)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &centroids[6],
            &Point::new(1.420820393249937, -0.878115294937453, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &centroids[7],
            &Point::new(0.0, -1.420820393249937, -0.878115294937453)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &centroids[8],
            &Point::new(-1.420820393249937, 0.878115294937453, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &centroids[9],
            &Point::new(-0.878115294937453, 0.0, 1.420820393249937)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &centroids[10],
            &Point::new(-0.878115294937453, 0.0, -1.420820393249937)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &centroids[11],
            &Point::new(-1.420820393249937, -0.878115294937453, 0.0)
        ));

        let face_normals = mesh.face_normals();

        for f in mesh.faces() {
            let fn_ = mesh.face_normal(f);

            MINI_CHECK!(fn_.is_some());
            MINI_CHECK!(TOLERANCE.is_vector_close(&face_normals[&f], &fn_.unwrap()));
        }

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &face_normals[&0],
            &Vector::new(0.5257311121191336, 0.0, 0.85065080835204)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &face_normals[&1],
            &Vector::new(0.85065080835204, 0.5257311121191336, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &face_normals[&2],
            &Vector::new(0.0, 0.85065080835204, 0.5257311121191336)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &face_normals[&3],
            &Vector::new(0.5257311121191336, 0.0, -0.85065080835204)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &face_normals[&4],
            &Vector::new(0.0, 0.85065080835204, -0.5257311121191336)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &face_normals[&5],
            &Vector::new(0.0, -0.85065080835204, 0.5257311121191336)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &face_normals[&6],
            &Vector::new(0.85065080835204, -0.5257311121191336, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &face_normals[&7],
            &Vector::new(0.0, -0.85065080835204, -0.5257311121191336)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &face_normals[&8],
            &Vector::new(-0.85065080835204, 0.5257311121191336, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &face_normals[&9],
            &Vector::new(-0.5257311121191336, 0.0, 0.85065080835204)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &face_normals[&10],
            &Vector::new(-0.5257311121191336, 0.0, -0.85065080835204)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &face_normals[&11],
            &Vector::new(-0.85065080835204, -0.5257311121191336, 0.0)
        ));

        for f in mesh.faces() {
            let fv = mesh.face_vertices(f).unwrap().clone();

            for v in fv {
                let angle = mesh.vertex_angle_in_face(v, f);

                MINI_CHECK!(angle.is_some());
                MINI_CHECK!(TOLERANCE.is_close(angle.unwrap(), 1.8849555921538759));
            }
        }

        let vertex_normals = mesh.vertex_normals();

        for v in mesh.vertices() {
            let vn = mesh.vertex_normal(v);

            MINI_CHECK!(vn.is_some());
            MINI_CHECK!(TOLERANCE.is_vector_close(&vertex_normals[&v], &vn.unwrap()));
        }

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&0],
            &Vector::new(0.5773502691896258, 0.5773502691896258, 0.5773502691896258)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&1],
            &Vector::new(0.0, 0.3568220897730899, 0.9341723589627158)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&2],
            &Vector::new(0.0, -0.3568220897730899, 0.9341723589627158)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&3],
            &Vector::new(0.5773502691896257, -0.5773502691896258, 0.5773502691896258)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&4],
            &Vector::new(0.9341723589627158, 0.0, 0.3568220897730899)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&5],
            &Vector::new(0.9341723589627158, 0.0, -0.3568220897730899)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&6],
            &Vector::new(0.5773502691896258, 0.5773502691896257, -0.5773502691896258)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&7],
            &Vector::new(0.3568220897730899, 0.9341723589627158, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&8],
            &Vector::new(-0.3568220897730899, 0.9341723589627157, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&9],
            &Vector::new(-0.5773502691896258, 0.5773502691896258, 0.5773502691896257)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&10],
            &Vector::new(0.5773502691896258, -0.5773502691896258, -0.5773502691896257)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&11],
            &Vector::new(0.0, -0.3568220897730899, -0.9341723589627157)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&12],
            &Vector::new(0.0, 0.3568220897730899, -0.9341723589627158)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&13],
            &Vector::new(-0.5773502691896257, 0.5773502691896258, -0.5773502691896258)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&14],
            &Vector::new(-0.5773502691896258, -0.5773502691896257, 0.5773502691896258)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&15],
            &Vector::new(-0.3568220897730899, -0.9341723589627157, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&16],
            &Vector::new(0.3568220897730899, -0.9341723589627158, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&17],
            &Vector::new(
                -0.5773502691896258,
                -0.5773502691896258,
                -0.5773502691896258
            )
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&18],
            &Vector::new(-0.9341723589627157, 0.0, -0.3568220897730899)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals[&19],
            &Vector::new(-0.9341723589627158, 0.0, 0.3568220897730899)
        ));

        let vertex_normals_weighted = mesh.vertex_normals_weighted(NormalWeighting::Angle);

        for v in mesh.vertices() {
            let vnw = mesh.vertex_normal_weighted(v, NormalWeighting::Angle);

            MINI_CHECK!(vnw.is_some());
            MINI_CHECK!(TOLERANCE.is_vector_close(&vertex_normals_weighted[&v], &vnw.unwrap()));
        }

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&0],
            &Vector::new(0.5773502691896257, 0.5773502691896257, 0.5773502691896257)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&1],
            &Vector::new(0.0, 0.3568220897730899, 0.9341723589627158)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&2],
            &Vector::new(0.0, -0.3568220897730899, 0.9341723589627158)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&3],
            &Vector::new(0.5773502691896257, -0.5773502691896257, 0.5773502691896258)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&4],
            &Vector::new(0.9341723589627158, 0.0, 0.3568220897730899)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&5],
            &Vector::new(0.9341723589627158, 0.0, -0.3568220897730899)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&6],
            &Vector::new(0.5773502691896258, 0.5773502691896257, -0.5773502691896257)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&7],
            &Vector::new(0.3568220897730899, 0.9341723589627158, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&8],
            &Vector::new(-0.3568220897730899, 0.9341723589627158, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&9],
            &Vector::new(-0.5773502691896257, 0.5773502691896258, 0.5773502691896257)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&10],
            &Vector::new(0.5773502691896257, -0.5773502691896258, -0.5773502691896257)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&11],
            &Vector::new(0.0, -0.3568220897730899, -0.9341723589627158)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&12],
            &Vector::new(0.0, 0.3568220897730899, -0.9341723589627158)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&13],
            &Vector::new(-0.5773502691896257, 0.5773502691896257, -0.5773502691896258)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&14],
            &Vector::new(-0.5773502691896258, -0.5773502691896257, 0.5773502691896257)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&15],
            &Vector::new(-0.35682208977309, -0.9341723589627158, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&16],
            &Vector::new(0.3568220897730899, -0.9341723589627158, 0.0)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&17],
            &Vector::new(
                -0.5773502691896257,
                -0.5773502691896257,
                -0.5773502691896257
            )
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&18],
            &Vector::new(-0.9341723589627158, 0.0, -0.3568220897730899)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vertex_normals_weighted[&19],
            &Vector::new(-0.9341723589627158, 0.0, 0.3568220897730899)
        ));

        let volume = mesh.volume();

        MINI_CHECK!(TOLERANCE.is_close(volume, 25.8630264921081));
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

        let mut mesh1 = mesh.duplicate();
        let mesh1_xf = Xform::translation(0.0, 0.0, 1.0);
        mesh1.transform(&mesh1_xf);

        MINI_CHECK!(mesh1.vertex_point(v0).unwrap()[2] == 1.0);

        let mut mesh2 = mesh.duplicate();
        let x = Xform::translation(0.0, 0.0, 1.0);
        mesh2.transform(&x);

        MINI_CHECK!(mesh2.vertex_point(v0).unwrap()[2] == 1.0);

        let mesh3 = mesh.duplicate();
        let mesh3_xf = Xform::translation(0.0, 0.0, 10.0);
        let mesh3t = mesh3.transformed(&mesh3_xf);

        MINI_CHECK!(mesh3t.vertex_point(v0).unwrap()[2] == 10.0);

        let mesh4 = mesh.duplicate();
        let x = Xform::translation(0.0, 0.0, 10.0);
        let mesh4t = mesh4.transformed(&x);

        MINI_CHECK!(mesh4t.vertex_point(v0).unwrap()[2] == 10.0);
    })
}

pub fn run_mesh_cut_by_plane() -> TestResult {
    MINI_TEST!("Cut By Plane", {
        use crate::Mesh;
        use crate::Plane;
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;

        let bx = Mesh::create_box(2.0, 2.0, 2.0);
        let half = bx.cut_by_plane(&Plane::from_point_normal(
            Point::new(0.0, 0.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            None,
        ));
        let copy = bx.cut_by_plane(&Plane::from_point_normal(
            Point::new(0.0, 0.0, 5.0),
            Vector::new(0.0, 0.0, -1.0),
            None,
        ));
        let empty = bx.cut_by_plane(&Plane::from_point_normal(
            Point::new(0.0, 0.0, 5.0),
            Vector::new(0.0, 0.0, 1.0),
            None,
        ));

        MINI_CHECK!(half.is_closed());
        MINI_CHECK!(half.number_of_vertices() == 8);
        MINI_CHECK!(half.number_of_faces() == 6);
        MINI_CHECK!(TOLERANCE.is_close(half.volume(), 4.0));
        MINI_CHECK!(copy == bx);
        MINI_CHECK!(empty.is_empty());

        let bottom = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let top = Polyline::new(vec![
            Point::new(0.0, 0.0, 1.0),
            Point::new(2.0, 0.0, 1.0),
            Point::new(2.0, 1.0, 1.0),
            Point::new(1.0, 1.0, 1.0),
            Point::new(1.0, 2.0, 1.0),
            Point::new(0.0, 2.0, 1.0),
            Point::new(0.0, 0.0, 1.0),
        ]);
        let prism = Mesh::loft(&[bottom], &[top], true, true);
        let upper = prism.cut_by_plane(&Plane::from_point_normal(
            Point::new(0.0, 0.0, 0.5),
            Vector::new(0.0, 0.0, 1.0),
            None,
        ));
        let corners = prism.cut_by_plane(&Plane::from_point_normal(
            Point::new(1.25, 1.25, 0.0),
            Vector::new(1.0, 1.0, 0.0),
            None,
        ));

        MINI_CHECK!(upper.is_closed());
        MINI_CHECK!(upper.number_of_faces() == 8);
        MINI_CHECK!(TOLERANCE.is_close(upper.volume(), 1.5));
        MINI_CHECK!(corners.is_closed());
        MINI_CHECK!(corners.number_of_faces() == 10);
        MINI_CHECK!(TOLERANCE.is_close(corners.volume(), 0.25));
    })
}

pub fn run_mesh_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Mesh;
        use crate::Point;

        use std::path::PathBuf;

        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.name = "test_mesh".to_string();

        let json = mesh.jsondump();
        let loaded_json = Mesh::jsonload(&json).unwrap();

        let json_string = mesh.file_json_dumps();
        let loaded_string = Mesh::file_json_loads(&json_string);

        let filename = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("serialization")
            .join("test_mesh.json");
        mesh.file_json_dump(filename.to_str().unwrap()).unwrap();
        let loaded_file = Mesh::file_json_load(filename.to_str().unwrap()).unwrap();

        MINI_CHECK!(loaded_json == mesh);
        MINI_CHECK!(loaded_string == mesh);
        MINI_CHECK!(loaded_file == mesh);

        let polys = vec![vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]];
        let pmesh = Mesh::from_polylines(polys, None);
        let loaded_tri = Mesh::jsonload(&pmesh.jsondump()).unwrap();
        let fk = *pmesh.triangulation.keys().next().unwrap();

        MINI_CHECK!(!loaded_tri.triangulation.is_empty());
        MINI_CHECK!(loaded_tri.triangulation.contains_key(&fk));

        let hmesh = Mesh::from_polygon_with_holes(
            &[
                vec![
                    Point::new(0.0, 0.0, 0.0),
                    Point::new(4.0, 0.0, 0.0),
                    Point::new(4.0, 4.0, 0.0),
                    Point::new(0.0, 4.0, 0.0),
                ],
                vec![
                    Point::new(1.0, 1.0, 0.0),
                    Point::new(3.0, 1.0, 0.0),
                    Point::new(3.0, 3.0, 0.0),
                    Point::new(1.0, 3.0, 0.0),
                ],
            ],
            true,
        );
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

        let proto_bytes = mesh.pb_dumps();
        let loaded_string = Mesh::pb_loads(&proto_bytes).unwrap();

        let filename = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("serialization")
            .join("test_mesh.bin");
        mesh.pb_dump(filename.to_str().unwrap());
        let loaded_file = Mesh::pb_load(filename.to_str().unwrap());
        let converted = Mesh::from_proto(mesh.to_proto());

        MINI_CHECK!(loaded_string == mesh);
        MINI_CHECK!(loaded_file == mesh);
        MINI_CHECK!(converted == mesh);

        let polys = vec![vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]];
        let pmesh = Mesh::from_polylines(polys, None);
        let loaded_tri = Mesh::pb_loads(&pmesh.pb_dumps()).unwrap();
        let fk = *pmesh.triangulation.keys().next().unwrap();

        MINI_CHECK!(!loaded_tri.triangulation.is_empty());
        MINI_CHECK!(loaded_tri.triangulation.contains_key(&fk));

        let hmesh = Mesh::from_polygon_with_holes(
            &[
                vec![
                    Point::new(0.0, 0.0, 0.0),
                    Point::new(4.0, 0.0, 0.0),
                    Point::new(4.0, 4.0, 0.0),
                    Point::new(0.0, 4.0, 0.0),
                ],
                vec![
                    Point::new(1.0, 1.0, 0.0),
                    Point::new(3.0, 1.0, 0.0),
                    Point::new(3.0, 3.0, 0.0),
                    Point::new(1.0, 3.0, 0.0),
                ],
            ],
            true,
        );
        let loaded_holes = Mesh::pb_loads(&hmesh.pb_dumps()).unwrap();
        let hfk = *hmesh.face_holes.keys().next().unwrap();

        MINI_CHECK!(!loaded_holes.face_holes.is_empty());
        MINI_CHECK!(loaded_holes.face_holes[&hfk] == hmesh.face_holes[&hfk]);
    })
}

pub fn run_mesh_loft_plate_four_holes() -> TestResult {
    MINI_TEST!("Loft Plate Four Holes", {
        use crate::Mesh;
        use crate::Point;
        use crate::Polyline;
        let bot = vec![
            Polyline::new(vec![
                Point::new(734.392021, -1906.59468, 1101.588031),
                Point::new(632.396858, -1838.597905, 948.595287),
                Point::new(624.453132, -1769.270846, 984.70313),
                Point::new(113.775484, -1428.81908, 218.686657),
                Point::new(121.719209, -1498.146139, 182.578814),
                Point::new(15.607979, -1427.40532, 23.411969),
                Point::new(0.0, -1441.0, -18.0),
                Point::new(0.0, -1893.0, -357.0),
                Point::new(13.416408, -1917.0, -348.167184),
                Point::new(104.290124, -1917.0, -166.419752),
                Point::new(118.441096, -1964.169906, -173.495238),
                Point::new(664.077103, -1964.169906, 917.776777),
                Point::new(649.926131, -1917.0, 924.852263),
                Point::new(736.583592, -1917.0, 1098.167184),
                Point::new(734.392021, -1906.59468, 1101.588031),
            ]),
            Polyline::new(vec![
                Point::new(322.544527, -1917.0, 270.089054),
                Point::new(213.417326, -1917.0, 51.834651),
                Point::new(199.266354, -1869.830094, 58.910137),
                Point::new(308.393555, -1869.830094, 277.16454),
                Point::new(322.544527, -1917.0, 270.089054),
            ]),
            Polyline::new(vec![
                Point::new(540.79893, -1917.0, 706.59786),
                Point::new(431.671728, -1917.0, 488.343457),
                Point::new(417.520757, -1869.830094, 495.418943),
                Point::new(526.647958, -1869.830094, 713.673346),
                Point::new(540.79893, -1917.0, 706.59786),
            ]),
            Polyline::new(vec![
                Point::new(424.153936, -1667.753669, 660.242619),
                Point::new(526.289465, -1735.844022, 813.445914),
                Point::new(530.261328, -1770.507552, 795.391992),
                Point::new(428.125798, -1702.417199, 642.188697),
                Point::new(424.153936, -1667.753669, 660.242619),
            ]),
            Polyline::new(vec![
                Point::new(219.882876, -1531.572963, 353.83603),
                Point::new(322.018406, -1599.663316, 507.039325),
                Point::new(325.990269, -1634.326846, 488.985403),
                Point::new(223.854739, -1566.236493, 335.782108),
                Point::new(219.882876, -1531.572963, 353.83603),
            ]),
        ];
        let top = vec![
            Polyline::new(vec![
                Point::new(711.660594, -1906.59468, 1126.880036),
                Point::new(605.549364, -1835.85386, 967.713191),
                Point::new(601.577501, -1801.190331, 985.767113),
                Point::new(90.899853, -1460.738565, 219.75064),
                Point::new(94.871715, -1495.402095, 201.696718),
                Point::new(-9.83197, -1425.599638, 44.641191),
                Point::new(-25.439949, -1439.194318, 3.229221),
                Point::new(-25.439949, -1893.0, -337.12504),
                Point::new(-12.023541, -1917.0, -328.292224),
                Point::new(75.988181, -1917.0, -152.26878),
                Point::new(104.290124, -2011.339811, -166.419752),
                Point::new(649.926131, -2011.339811, 924.852263),
                Point::new(621.624188, -1917.0, 939.003234),
                Point::new(713.852165, -1917.0, 1123.459189),
                Point::new(711.660594, -1906.59468, 1126.880036),
            ]),
            Polyline::new(vec![
                Point::new(308.393555, -1964.169906, 277.16454),
                Point::new(199.266354, -1964.169906, 58.910137),
                Point::new(185.115382, -1917.0, 65.985623),
                Point::new(294.242584, -1917.0, 284.240026),
                Point::new(308.393555, -1964.169906, 277.16454),
            ]),
            Polyline::new(vec![
                Point::new(526.647958, -1964.169906, 713.673346),
                Point::new(417.520757, -1964.169906, 495.418943),
                Point::new(403.369785, -1917.0, 502.494429),
                Point::new(512.496987, -1917.0, 720.748832),
                Point::new(526.647958, -1964.169906, 713.673346),
            ]),
            Polyline::new(vec![
                Point::new(401.278305, -1699.673154, 661.306602),
                Point::new(503.413834, -1767.763507, 814.509897),
                Point::new(507.385697, -1802.427037, 796.455975),
                Point::new(405.250167, -1734.336684, 643.25268),
                Point::new(401.278305, -1699.673154, 661.306602),
            ]),
            Polyline::new(vec![
                Point::new(197.007245, -1563.492448, 354.900013),
                Point::new(299.142775, -1631.582801, 508.103307),
                Point::new(303.114638, -1666.246331, 490.049386),
                Point::new(200.979108, -1598.155978, 336.846091),
                Point::new(197.007245, -1563.492448, 354.900013),
            ]),
        ];
        let m = Mesh::loft(&bot, &top, true, true);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_mesh_loft_plate_v2() -> TestResult {
    MINI_TEST!("Loft Plate V2", {
        use crate::Mesh;
        use crate::Point;
        use crate::Polyline;
        let top = vec![
            Polyline::new(vec![
                Point::new(734.392021, -28.40532, 1101.588031),
                Point::new(630.839301, -97.440466, 946.258951),
                Point::new(602.668732, -21.881034, 974.757956),
                Point::new(90.636822, -363.235641, 206.710092),
                Point::new(118.807391, -438.795073, 178.211087),
                Point::new(15.607979, -507.59468, 23.411969),
                Point::new(21.213203, -518.0, 21.213203),
                Point::new(1478.786797, -518.0, 1478.786797),
                Point::new(1476.953362, -502.635574, 1488.476681),
                Point::new(1323.309106, -400.20607, 1411.654553),
                Point::new(1323.309106, -350.20607, 1449.154553),
                Point::new(921.429178, -82.286119, 1248.214589),
                Point::new(921.429178, -132.286119, 1210.714589),
                Point::new(773.046638, -33.364426, 1136.523319),
                Point::new(734.392021, -28.40532, 1101.588031),
            ]),
            Polyline::new(vec![
                Point::new(1055.389154, -196.592769, 1296.444577),
                Point::new(1189.34913, -285.89942, 1363.424565),
                Point::new(1189.34913, -310.89942, 1344.674565),
                Point::new(1055.389154, -221.592769, 1277.694577),
                Point::new(1055.389154, -196.592769, 1296.444577),
            ]),
            Polyline::new(vec![
                Point::new(411.941252, -196.202593, 653.289308),
                Point::new(514.347634, -127.931671, 806.898881),
                Point::new(528.432919, -165.711387, 792.649378),
                Point::new(426.026537, -233.982309, 639.039805),
                Point::new(411.941252, -196.202593, 653.289308),
            ]),
            Polyline::new(vec![
                Point::new(207.128489, -332.744435, 346.070162),
                Point::new(309.53487, -264.473514, 499.679735),
                Point::new(323.620155, -302.25323, 485.430233),
                Point::new(221.213773, -370.524151, 331.82066),
                Point::new(207.128489, -332.744435, 346.070162),
            ]),
        ];
        let bot = vec![
            Polyline::new(vec![
                Point::new(717.764591, -24.335988, 1136.036032),
                Point::new(607.106921, -98.107768, 970.049526),
                Point::new(593.021636, -60.328052, 984.299029),
                Point::new(80.989727, -401.682659, 216.251164),
                Point::new(95.075011, -439.462375, 202.001662),
                Point::new(-28.206905, -521.650319, 17.078787),
                Point::new(-22.601681, -532.055639, 14.880022),
                Point::new(1489.346823, -532.055639, 1526.828525),
                Point::new(1487.513388, -516.691213, 1536.51841),
                Point::new(1323.309106, -407.221692, 1454.416269),
                Point::new(1323.309106, -382.221692, 1473.166269),
                Point::new(921.429178, -114.30174, 1272.226305),
                Point::new(921.429178, -139.30174, 1253.476305),
                Point::new(756.419209, -29.295094, 1170.97132),
                Point::new(717.764591, -24.335988, 1136.036032),
            ]),
            Polyline::new(vec![
                Point::new(1055.389154, -228.608391, 1320.456293),
                Point::new(1189.34913, -317.915041, 1387.436281),
                Point::new(1189.34913, -342.915041, 1368.686281),
                Point::new(1055.389154, -253.608391, 1301.706293),
                Point::new(1055.389154, -228.608391, 1320.456293),
            ]),
            Polyline::new(vec![
                Point::new(402.294157, -234.649611, 662.830381),
                Point::new(504.700539, -166.37869, 816.439954),
                Point::new(518.785824, -204.158406, 802.190451),
                Point::new(416.379442, -272.429327, 648.580878),
                Point::new(402.294157, -234.649611, 662.830381),
            ]),
            Polyline::new(vec![
                Point::new(197.481393, -371.191453, 355.611235),
                Point::new(299.887775, -302.920532, 509.220808),
                Point::new(313.97306, -340.700248, 494.971305),
                Point::new(211.566678, -408.971169, 341.361733),
                Point::new(197.481393, -371.191453, 355.611235),
            ]),
        ];
        let m = Mesh::loft(&top, &bot, true, true);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_mesh_loft_plate_v3() -> TestResult {
    MINI_TEST!("Loft Plate V3", {
        use crate::Mesh;
        use crate::Point;
        use crate::Polyline;
        let top = vec![
            Polyline::new(vec![
                Point::new(734.392021, 352.59468, 1101.588031),
                Point::new(618.973111, 275.648741, 928.459666),
                Point::new(618.973111, 369.988552, 999.214525),
                Point::new(106.941201, 28.633945, 231.16666),
                Point::new(106.941201, -65.705866, 160.411802),
                Point::new(15.607979, -126.59468, 23.411969),
                Point::new(21.213203, -137.0, 21.213203),
                Point::new(1478.786797, -137.0, 1478.786797),
                Point::new(1476.953362, -121.635574, 1488.476681),
                Point::new(1323.309106, -19.20607, 1411.654553),
                Point::new(1323.309106, 30.79393, 1449.154553),
                Point::new(921.429178, 298.713881, 1248.214589),
                Point::new(921.429178, 248.713881, 1210.714589),
                Point::new(773.046638, 347.635574, 1136.523319),
                Point::new(734.392021, 352.59468, 1101.588031),
            ]),
            Polyline::new(vec![
                Point::new(1055.389154, 184.407231, 1296.444577),
                Point::new(1189.34913, 95.10058, 1363.424565),
                Point::new(1189.34913, 70.10058, 1344.674565),
                Point::new(1055.389154, 159.407231, 1277.694577),
                Point::new(1055.389154, 184.407231, 1296.444577),
            ]),
            Polyline::new(vec![
                Point::new(414.160347, 186.276804, 656.61795),
                Point::new(516.566729, 254.547725, 810.227523),
                Point::new(516.566729, 207.377819, 774.850093),
                Point::new(414.160347, 139.106898, 621.240521),
                Point::new(414.160347, 186.276804, 656.61795),
            ]),
            Polyline::new(vec![
                Point::new(209.347583, 49.734961, 349.398804),
                Point::new(311.753965, 118.005882, 503.008377),
                Point::new(311.753965, 70.835977, 467.630948),
                Point::new(209.347583, 2.565055, 314.021375),
                Point::new(209.347583, 49.734961, 349.398804),
            ]),
        ];
        let bot = vec![
            Polyline::new(vec![
                Point::new(717.764591, 356.664012, 1136.036032),
                Point::new(618.973111, 290.803025, 987.848811),
                Point::new(618.973111, 337.972931, 1023.226241),
                Point::new(106.941201, -3.381676, 255.178376),
                Point::new(106.941201, -50.551581, 219.800947),
                Point::new(-28.206905, -140.650319, 17.078787),
                Point::new(-22.601681, -151.055639, 14.880022),
                Point::new(1489.346823, -151.055639, 1526.828525),
                Point::new(1487.513388, -135.691213, 1536.51841),
                Point::new(1323.309106, -26.221692, 1454.416269),
                Point::new(1323.309106, -1.221692, 1473.166269),
                Point::new(921.429178, 266.69826, 1272.226305),
                Point::new(921.429178, 241.69826, 1253.476305),
                Point::new(756.419209, 351.704906, 1170.97132),
                Point::new(717.764591, 356.664012, 1136.036032),
            ]),
            Polyline::new(vec![
                Point::new(1055.389154, 152.391609, 1320.456293),
                Point::new(1189.34913, 63.084959, 1387.436281),
                Point::new(1189.34913, 38.084959, 1368.686281),
                Point::new(1055.389154, 127.391609, 1301.706293),
                Point::new(1055.389154, 152.391609, 1320.456293),
            ]),
            Polyline::new(vec![
                Point::new(414.160347, 154.261182, 680.629666),
                Point::new(516.566729, 222.532104, 834.239239),
                Point::new(516.566729, 175.362198, 798.861809),
                Point::new(414.160347, 107.091277, 645.252236),
                Point::new(414.160347, 154.261182, 680.629666),
            ]),
            Polyline::new(vec![
                Point::new(209.347583, 17.71934, 373.41052),
                Point::new(311.753965, 85.990261, 527.020093),
                Point::new(311.753965, 38.820356, 491.642664),
                Point::new(209.347583, -29.450566, 338.033091),
                Point::new(209.347583, 17.71934, 373.41052),
            ]),
        ];
        let m = Mesh::loft(&top, &bot, true, true);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_mesh_vertex_neighbors() -> TestResult {
    MINI_TEST!("Vertex Neighbors", {
        use crate::Mesh;
        let mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let mut n0 = mesh.vertex_neighbors(0, false).unwrap();
        let mut n0v = mesh.vertex_vertices(0).unwrap();
        n0.sort();
        n0v.sort();

        MINI_CHECK!(n0 == n0v);
        MINI_CHECK!(n0.len() == 3);
    })
}

pub fn run_mesh_vertices_on_boundary() -> TestResult {
    MINI_TEST!("Vertices On Boundary", {
        use crate::Mesh;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);

        MINI_CHECK!(mesh.vertices_on_boundary().is_empty());
        mesh.remove_face(mesh.faces()[0]);

        MINI_CHECK!(mesh.vertices_on_boundary().len() == 4);
    })
}

pub fn run_mesh_edges_on_boundary() -> TestResult {
    MINI_TEST!("Edges On Boundary", {
        use crate::Mesh;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);

        MINI_CHECK!(mesh.edges_on_boundary().is_empty());
        mesh.remove_face(mesh.faces()[0]);

        MINI_CHECK!(mesh.edges_on_boundary().len() == 4);
    })
}

pub fn run_mesh_faces_on_boundary() -> TestResult {
    MINI_TEST!("Faces On Boundary", {
        use crate::Mesh;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);

        MINI_CHECK!(mesh.faces_on_boundary().is_empty());
        mesh.remove_face(mesh.faces()[0]);

        MINI_CHECK!(mesh.faces_on_boundary().len() == 4);
    })
}

pub fn run_mesh_halfedge_face() -> TestResult {
    MINI_TEST!("Halfedge Face", {
        use crate::Mesh;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let f = mesh.halfedge_face((0, 3));

        MINI_CHECK!(f.is_some());
        MINI_CHECK!(f.unwrap() == 0);
        mesh.remove_face(0);

        MINI_CHECK!(mesh.halfedge_face((0, 3)).is_none());
    })
}

pub fn run_mesh_halfedge_after_before() -> TestResult {
    MINI_TEST!("Halfedge After Before", {
        use crate::Mesh;
        let mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let after = mesh.halfedge_after((0, 3));
        let before = mesh.halfedge_before((0, 3));

        MINI_CHECK!(after.is_some());
        MINI_CHECK!(after.unwrap() == (3, 2));
        MINI_CHECK!(before.is_some());
        MINI_CHECK!(before.unwrap() == (1, 0));
    })
}

pub fn run_mesh_halfedge_loop() -> TestResult {
    MINI_TEST!("Halfedge Loop", {
        use crate::Mesh;
        let mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let loop_edges = mesh.halfedge_loop((0, 3));

        MINI_CHECK!(loop_edges.len() == 1);
        MINI_CHECK!(loop_edges[0] == (0, 3));
    })
}

pub fn run_mesh_halfedge_strip() -> TestResult {
    MINI_TEST!("Halfedge Strip", {
        use crate::Mesh;
        let mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let strip = mesh.halfedge_strip((0, 3));

        MINI_CHECK!(strip.len() == 5);
        MINI_CHECK!(strip[0] == (0, 3));
        MINI_CHECK!(strip[strip.len() - 1] == (0, 3));
    })
}

pub fn run_mesh_vertex_sample() -> TestResult {
    MINI_TEST!("Vertex Sample", {
        use crate::Mesh;
        use std::collections::HashSet;
        let mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let s = mesh.vertex_sample(3, 42);

        MINI_CHECK!(s.len() == 3);
        let uniq: HashSet<usize> = s.iter().copied().collect();

        MINI_CHECK!(uniq.len() == 3);
        let s2 = mesh.vertex_sample(3, 42);

        MINI_CHECK!(s == s2);
    })
}

pub fn run_mesh_edge_sample() -> TestResult {
    MINI_TEST!("Edge Sample", {
        use crate::Mesh;
        let mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let s = mesh.edge_sample(2, 7);

        MINI_CHECK!(s.len() == 2);
        let s2 = mesh.edge_sample(2, 7);

        MINI_CHECK!(s == s2);
    })
}

pub fn run_mesh_face_sample() -> TestResult {
    MINI_TEST!("Face Sample", {
        use crate::Mesh;
        let mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let s = mesh.face_sample(2, 11);

        MINI_CHECK!(s.len() == 2);
        let s2 = mesh.face_sample(2, 11);

        MINI_CHECK!(s == s2);
    })
}

pub fn run_mesh_face_center() -> TestResult {
    MINI_TEST!("Face Center", {
        use crate::Mesh;
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let c = mesh.face_center(0).unwrap();
        let cc = mesh.face_centroid(0).unwrap();

        MINI_CHECK!(c == cc);
    })
}

pub fn run_mesh_face_polygon() -> TestResult {
    MINI_TEST!("Face Polygon", {
        use crate::Mesh;
        let mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let poly = mesh.face_polygon(0).unwrap();
        let pts = poly.get_points();

        MINI_CHECK!(pts.len() == 5);
        MINI_CHECK!(pts[0] == pts[pts.len() - 1]);
    })
}

pub fn run_mesh_flip_cycles() -> TestResult {
    MINI_TEST!("Flip Cycles", {
        use crate::Mesh;
        use crate::Tolerance;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let n0 = mesh.face_normal(0).unwrap();
        mesh.flip_cycles();
        let n0b = mesh.face_normal(0).unwrap();

        MINI_CHECK!((n0[0] + n0b[0]).abs() < Tolerance::ZERO_TOLERANCE);
        MINI_CHECK!((n0[1] + n0b[1]).abs() < Tolerance::ZERO_TOLERANCE);
        MINI_CHECK!((n0[2] + n0b[2]).abs() < Tolerance::ZERO_TOLERANCE);
    })
}

pub fn run_mesh_face_normal_unitized() -> TestResult {
    MINI_TEST!("Face Normal Unitized", {
        use crate::Mesh;
        use crate::Tolerance;
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let nu = mesh.face_normal_unitized(0, true).unwrap();
        let nn = mesh.face_normal_unitized(0, false).unwrap();

        MINI_CHECK!((nu.magnitude() - 1.0).abs() < Tolerance::ZERO_TOLERANCE);
        MINI_CHECK!(nn.magnitude() > 1.0);
    })
}

pub fn run_mesh_default_attributes() -> TestResult {
    MINI_TEST!("Default Attributes", {
        use crate::Mesh;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_vertex_attributes(&[("is_support", 0.0), ("load_z", 0.0)]);
        mesh.update_default_face_attributes(&[("stress", 0.0)]);
        mesh.update_default_edge_attributes(&[("weight", 1.0)]);

        MINI_CHECK!(mesh.default_vertex_attributes["is_support"] == 0.0);
        MINI_CHECK!(mesh.default_vertex_attributes["load_z"] == 0.0);
        MINI_CHECK!(mesh.default_face_attributes["stress"] == 0.0);
        MINI_CHECK!(mesh.default_edge_attributes["weight"] == 1.0);
    })
}

pub fn run_mesh_vertex_attribute() -> TestResult {
    MINI_TEST!("Vertex Attribute", {
        use crate::Mesh;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_vertex_attributes(&[("is_support", 0.0)]);
        mesh.set_vertex_attribute(0, "is_support", 1.0);

        MINI_CHECK!(mesh.vertex_attribute(0, "is_support").unwrap() == 1.0);
        MINI_CHECK!(mesh.vertex_attribute(1, "is_support").unwrap() == 0.0);
    })
}

pub fn run_mesh_face_attribute() -> TestResult {
    MINI_TEST!("Face Attribute", {
        use crate::Mesh;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_face_attributes(&[("stress", 0.0)]);
        mesh.set_face_attribute(0, "stress", 2.5);

        MINI_CHECK!(mesh.face_attribute(0, "stress").unwrap() == 2.5);
        MINI_CHECK!(mesh.face_attribute(1, "stress").unwrap() == 0.0);
    })
}

pub fn run_mesh_edge_attribute() -> TestResult {
    MINI_TEST!("Edge Attribute", {
        use crate::Mesh;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_edge_attributes(&[("weight", 1.0)]);
        mesh.set_edge_attribute((0, 1), "weight", 5.0);

        MINI_CHECK!(mesh.edge_attribute((0, 1), "weight").unwrap() == 5.0);
        MINI_CHECK!(mesh.edge_attribute((0, 3), "weight").unwrap() == 1.0);
    })
}

pub fn run_mesh_vertices_attribute_bulk() -> TestResult {
    MINI_TEST!("Vertices Attribute Bulk", {
        use crate::Mesh;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_vertex_attributes(&[("is_support", 0.0)]);
        let keys = vec![0, 1, 2];
        mesh.set_vertices_attribute("is_support", 1.0, Some(&keys));
        let vals = mesh.vertices_attribute("is_support", None);

        MINI_CHECK!(vals[0].unwrap() == 1.0);
        MINI_CHECK!(vals[1].unwrap() == 1.0);
        MINI_CHECK!(vals[2].unwrap() == 1.0);
        MINI_CHECK!(vals[3].unwrap() == 0.0);
    })
}

pub fn run_mesh_vertices_where() -> TestResult {
    MINI_TEST!("Vertices Where", {
        use crate::Mesh;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_vertex_attributes(&[("is_support", 0.0)]);
        let keys = vec![0, 2, 4];
        mesh.set_vertices_attribute("is_support", 1.0, Some(&keys));
        let mut sup = mesh.vertices_where(&[("is_support", 1.0)]);
        sup.sort();

        MINI_CHECK!(sup.len() == 3);
        MINI_CHECK!(sup == vec![0, 2, 4]);
    })
}

pub fn run_mesh_faces_where() -> TestResult {
    MINI_TEST!("Faces Where", {
        use crate::Mesh;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_face_attributes(&[("tag", 0.0)]);
        mesh.set_face_attribute(2, "tag", 7.0);
        mesh.set_face_attribute(4, "tag", 7.0);
        let mut out = mesh.faces_where(&[("tag", 7.0)]);
        out.sort();

        MINI_CHECK!(out == vec![2, 4]);
    })
}

pub fn run_mesh_edges_where() -> TestResult {
    MINI_TEST!("Edges Where", {
        use crate::Mesh;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_edge_attributes(&[("weight", 0.0)]);
        mesh.set_edge_attribute((0, 1), "weight", 3.0);
        let out = mesh.edges_where(&[("weight", 3.0)]);

        MINI_CHECK!(out.len() == 1);
        MINI_CHECK!(out[0] == (0, 1));
    })
}

pub fn run_mesh_vertices_where_predicate() -> TestResult {
    MINI_TEST!("Vertices Where Predicate", {
        use crate::Mesh;
        use std::collections::HashMap;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_vertex_attributes(&[("load", 0.0)]);
        mesh.set_vertex_attribute(0, "load", 5.0);
        mesh.set_vertex_attribute(1, "load", 10.0);
        let mut big = mesh.vertices_where_predicate(&|_k: usize, a: &HashMap<String, f64>| {
            a.get("load").is_some_and(|v| *v > 4.0)
        });
        big.sort();

        MINI_CHECK!(big == vec![0, 1]);
    })
}

pub fn run_mesh_faces_where_predicate() -> TestResult {
    MINI_TEST!("Faces Where Predicate", {
        use crate::Mesh;
        use std::collections::HashMap;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_face_attributes(&[("area", 0.0)]);
        mesh.set_face_attribute(0, "area", 2.0);
        mesh.set_face_attribute(3, "area", 4.0);
        let mut big = mesh.faces_where_predicate(&|_k: usize, a: &HashMap<String, f64>| {
            a.get("area").is_some_and(|v| *v > 1.0)
        });
        big.sort();

        MINI_CHECK!(big == vec![0, 3]);
    })
}

pub fn run_mesh_edges_where_predicate() -> TestResult {
    MINI_TEST!("Edges Where Predicate", {
        use crate::Mesh;
        use std::collections::HashMap;
        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_edge_attributes(&[("weight", 0.0)]);
        mesh.set_edge_attribute((0, 1), "weight", 5.0);
        let big = mesh.edges_where_predicate(&|_e: (usize, usize), a: &HashMap<String, f64>| {
            a.get("weight").is_some_and(|v| *v > 1.0)
        });

        MINI_CHECK!(big.len() == 1);
        MINI_CHECK!(big[0] == (0, 1));
    })
}

pub fn run_mesh_refresh_guid() -> TestResult {
    MINI_TEST!("Refresh Guid", {
        use crate::Mesh;
        let mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let original = mesh.guid().to_string();
        let mut copy = mesh.clone();

        MINI_CHECK!(copy.guid() == original);
        copy.refresh_guid();

        MINI_CHECK!(copy.guid() != original);
        MINI_CHECK!(mesh.guid() == original);
    })
}

pub fn run_mesh_assignment_keeps_objectcolor() -> TestResult {
    MINI_TEST!("Assignment Keeps Objectcolor", {
        use crate::Color;
        use crate::Mesh;
        use crate::Point;
        let mut source = Mesh::from_vertices_and_faces(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![vec![0, 1, 2, 3]],
        );
        source.set_objectcolor(Color::with_name(0.72, 0.72, 0.74, 1.0, "grey"));

        let target = source.duplicate();

        MINI_CHECK!(target.get_objectcolor().r == source.get_objectcolor().r);
        MINI_CHECK!(target.get_objectcolor().g == source.get_objectcolor().g);
        MINI_CHECK!(target.get_objectcolor().b == source.get_objectcolor().b);
        MINI_CHECK!(target.color_mode == source.color_mode);
    })
}

pub fn run_mesh_from_polyline_pairs() -> TestResult {
    MINI_TEST!("From Polyline Pairs", {
        use crate::Mesh;
        use crate::Point;
        use crate::Polyline;

        let top = Polyline::new(vec![
            Point::new(0.0, 0.0, 1.0),
            Point::new(1.0, 0.0, 1.0),
            Point::new(1.0, 1.0, 1.0),
            Point::new(0.0, 1.0, 1.0),
            Point::new(0.0, 0.0, 1.0),
        ]);
        let bot = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let mesh = Mesh::from_polyline_pairs(&[top, bot], 2.0);

        MINI_CHECK!(mesh.is_closed());
        MINI_CHECK!(mesh.number_of_faces() == 6);
        MINI_CHECK!(TOLERANCE.is_close(mesh.volume(), 0.125));
    })
}

pub fn run_mesh_from_polyline_pairs_vnf() -> TestResult {
    MINI_TEST!("From Polyline Pairs Vnf", {
        use crate::Mesh;
        use crate::Point;
        use crate::Polyline;

        let top = Polyline::new(vec![
            Point::new(0.0, 0.0, 1.0),
            Point::new(1.0, 0.0, 1.0),
            Point::new(1.0, 1.0, 1.0),
            Point::new(0.0, 1.0, 1.0),
            Point::new(0.0, 0.0, 1.0),
        ]);
        let bot = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let (vertices, normals, triangles) = Mesh::from_polyline_pairs_vnf(&[top, bot], 1.0);

        MINI_CHECK!(triangles.len() == 36);
        MINI_CHECK!(vertices.len() == 108);
        MINI_CHECK!(normals.len() == 108);
        MINI_CHECK!(triangles[35] == 35);
    })
}

pub fn run_mesh_reflex_fold() -> TestResult {
    MINI_TEST!("Reflex Fold", {
        use crate::Mesh;
        use crate::Point;
        use crate::Polyline;

        let cross_section = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 1.0, 0.0),
        ]);
        let profile = Polyline::new(vec![
            Point::new(0.0, -1.0, 0.0),
            Point::new(0.0, 0.0, 1.0),
            Point::new(0.0, 1.0, 0.0),
        ]);
        let mesh = Mesh::reflex_fold(&cross_section, &profile);

        MINI_CHECK!(mesh.is_valid());
        MINI_CHECK!(mesh.number_of_vertices() == 9);
        MINI_CHECK!(mesh.number_of_faces() == 4);
    })
}

pub fn run_mesh_miter_contours() -> TestResult {
    MINI_TEST!("Miter Contours", {
        use crate::Mesh;
        use crate::Vector;

        let shell = Mesh::create_box(2.0, 2.0, 2.0);
        let contours = Mesh::miter_contours(&shell, 0.1, 0.0, 0.0, false, 90.0);

        MINI_CHECK!(contours.len() == 6);
        MINI_CHECK!(contours[0].0.len() == 4);
        MINI_CHECK!(contours[0].1.len() == 4);
        MINI_CHECK!(contours[0].2.len() == 4);
        MINI_CHECK!(contours[0].3.len() == 4);
        MINI_CHECK!(TOLERANCE.is_vector_close(&contours[0].4, &Vector::new(0.0, 0.0, -1.0)));
    })
}

pub fn run_mesh_set_face_triangulation() -> TestResult {
    MINI_TEST!("Set Face Triangulation", {
        use crate::Mesh;

        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.set_face_triangulation(0, vec![[0, 3, 2], [0, 2, 1]]);

        MINI_CHECK!(mesh.get_triangulation()[&0].len() == 2);
        MINI_CHECK!(mesh.get_triangulation()[&0][1] == [0, 2, 1]);
    })
}

pub fn run_mesh_set_face_holes() -> TestResult {
    MINI_TEST!("Set Face Holes", {
        use crate::Mesh;

        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.set_face_holes(0, vec![vec![4, 5, 6]]);

        MINI_CHECK!(mesh.get_face_holes()[&0].len() == 1);
        MINI_CHECK!(mesh.get_face_holes()[&0][0] == vec![4, 5, 6]);
    })
}

pub fn run_mesh_rebuild_halfedges() -> TestResult {
    MINI_TEST!("Rebuild Halfedges", {
        use crate::Mesh;

        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let halfedge = mesh.halfedge.clone();
        mesh.halfedge.clear();
        mesh.rebuild_halfedges();

        MINI_CHECK!(mesh.halfedge == halfedge);
    })
}

pub fn run_mesh_ensure_halfedges() -> TestResult {
    MINI_TEST!("Ensure Halfedges", {
        use crate::Mesh;

        let mut mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.halfedge.clear();
        mesh.ensure_halfedges();

        MINI_CHECK!(mesh.halfedge.len() == 8);
        MINI_CHECK!(mesh.halfedge_face((0, 3)).is_some());
    })
}

pub fn run_mesh_edge_face_map() -> TestResult {
    MINI_TEST!("Edge Face Map", {
        use crate::Mesh;

        let mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let efm = mesh.edge_face_map();

        MINI_CHECK!(efm.len() == 24);
        MINI_CHECK!(efm[&(0, 3)] == 0);
        MINI_CHECK!(efm[&(3, 0)] == 4);
    })
}

pub fn run_mesh_face_outlines() -> TestResult {
    MINI_TEST!("Face Outlines", {
        use crate::Mesh;

        let mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let outlines = mesh.face_outlines();

        MINI_CHECK!(outlines.len() == 6);
        MINI_CHECK!(outlines[0].point_count() == 5);
        MINI_CHECK!(outlines[0].is_closed());
    })
}

pub fn run_mesh_dihedral_angle() -> TestResult {
    MINI_TEST!("Dihedral Angle", {
        use crate::Mesh;

        let mesh = Mesh::create_box(1.0, 1.0, 1.0);
        let angle = mesh.dihedral_angle(0, 1);

        MINI_CHECK!(angle.is_some());
        MINI_CHECK!(TOLERANCE.is_close(angle.unwrap(), 90.0));
    })
}

pub fn run_mesh_triangle_bvh() -> TestResult {
    MINI_TEST!("Triangle Bvh", {
        use crate::Mesh;
        use crate::Point;
        use crate::Vector;

        let mut mesh = Mesh::create_box(2.0, 2.0, 2.0);
        mesh.build_triangle_bvh(false);
        let mut ids: Vec<usize> = Vec::new();
        let hit = mesh.triangle_bvh_ray_cast(
            &Point::new(0.1, 0.2, -10.0),
            &Vector::new(0.0, 0.0, 1.0),
            &mut ids,
            true,
        );
        let found = mesh.get_triangle_by_id(ids[0]);

        MINI_CHECK!(mesh.get_cached_bvh().is_some());
        MINI_CHECK!(hit);
        MINI_CHECK!(ids.len() == 4);
        MINI_CHECK!(found.is_some());
        MINI_CHECK!(found.as_ref().unwrap().0 < 2);
        MINI_CHECK!(TOLERANCE.is_close(found.as_ref().unwrap().2[2].abs(), 1.0));

        mesh.clear_triangle_bvh();

        MINI_CHECK!(mesh.get_cached_bvh().is_none());
    })
}

pub fn run_mesh_triangle_aabb_tree() -> TestResult {
    MINI_TEST!("Triangle Aabb Tree", {
        use crate::Mesh;

        let mut mesh = Mesh::create_box(2.0, 2.0, 2.0);
        mesh.build_triangle_aabb_tree(false);

        MINI_CHECK!(mesh.get_cached_aabb_tree().is_some());
        MINI_CHECK!(mesh.get_cached_bvh().is_some());
    })
}

REGISTER_MINI_TEST!(
    "Mesh",
    "Constructor",
    crate::mesh_test::run_mesh_constructor
);
REGISTER_MINI_TEST!(
    "Mesh",
    "From Polylines",
    crate::mesh_test::run_mesh_from_polylines
);
REGISTER_MINI_TEST!(
    "Mesh",
    "From Polylines Polyline",
    crate::mesh_test::run_mesh_from_polylines_polyline
);
REGISTER_MINI_TEST!("Mesh", "From Lines", crate::mesh_test::run_mesh_from_lines);
REGISTER_MINI_TEST!(
    "Mesh",
    "From Polygon With Holes",
    crate::mesh_test::run_mesh_from_polygon_with_holes
);
REGISTER_MINI_TEST!("Mesh", "Loft", crate::mesh_test::run_mesh_loft);
REGISTER_MINI_TEST!(
    "Mesh",
    "Loft Concave With Holes",
    crate::mesh_test::run_mesh_loft_concave_with_holes_and_collinear
);
REGISTER_MINI_TEST!(
    "Mesh",
    "From Polygon With Holes Many",
    crate::mesh_test::run_mesh_from_polygon_with_holes_many
);
REGISTER_MINI_TEST!("Mesh", "Loft Many", crate::mesh_test::run_mesh_loft_many);
REGISTER_MINI_TEST!(
    "Mesh",
    "Loft With Quads And Triangles",
    crate::mesh_test::run_mesh_loft_panels
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Boolean Queries",
    crate::mesh_test::run_mesh_boolean_queries
);
REGISTER_MINI_TEST!("Mesh", "Attributes", crate::mesh_test::run_mesh_attributes);
REGISTER_MINI_TEST!("Mesh", "Edges", crate::mesh_test::run_mesh_edges);
REGISTER_MINI_TEST!(
    "Mesh",
    "Create Dodecahedron",
    crate::mesh_test::run_mesh_create_dodecahedron
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Vertex And Face Operations",
    crate::mesh_test::run_mesh_vertex_and_face_operations
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Connectivity Queries",
    crate::mesh_test::run_mesh_connectivity_queries
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Geometric Properties",
    crate::mesh_test::run_mesh_geometric_properties
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Transformation",
    crate::mesh_test::run_mesh_transformation
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Cut By Plane",
    crate::mesh_test::run_mesh_cut_by_plane
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Json Roundtrip",
    crate::mesh_test::run_mesh_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Protobuf Roundtrip",
    crate::mesh_test::run_mesh_protobuf_roundtrip
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Loft Plate Four Holes",
    crate::mesh_test::run_mesh_loft_plate_four_holes
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Loft Plate V2",
    crate::mesh_test::run_mesh_loft_plate_v2
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Loft Plate V3",
    crate::mesh_test::run_mesh_loft_plate_v3
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Vertex Neighbors",
    crate::mesh_test::run_mesh_vertex_neighbors
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Vertices On Boundary",
    crate::mesh_test::run_mesh_vertices_on_boundary
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Edges On Boundary",
    crate::mesh_test::run_mesh_edges_on_boundary
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Faces On Boundary",
    crate::mesh_test::run_mesh_faces_on_boundary
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Halfedge Face",
    crate::mesh_test::run_mesh_halfedge_face
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Halfedge After Before",
    crate::mesh_test::run_mesh_halfedge_after_before
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Halfedge Loop",
    crate::mesh_test::run_mesh_halfedge_loop
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Halfedge Strip",
    crate::mesh_test::run_mesh_halfedge_strip
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Vertex Sample",
    crate::mesh_test::run_mesh_vertex_sample
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Edge Sample",
    crate::mesh_test::run_mesh_edge_sample
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Face Sample",
    crate::mesh_test::run_mesh_face_sample
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Face Center",
    crate::mesh_test::run_mesh_face_center
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Face Polygon",
    crate::mesh_test::run_mesh_face_polygon
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Flip Cycles",
    crate::mesh_test::run_mesh_flip_cycles
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Face Normal Unitized",
    crate::mesh_test::run_mesh_face_normal_unitized
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Default Attributes",
    crate::mesh_test::run_mesh_default_attributes
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Vertex Attribute",
    crate::mesh_test::run_mesh_vertex_attribute
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Face Attribute",
    crate::mesh_test::run_mesh_face_attribute
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Edge Attribute",
    crate::mesh_test::run_mesh_edge_attribute
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Vertices Attribute Bulk",
    crate::mesh_test::run_mesh_vertices_attribute_bulk
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Vertices Where",
    crate::mesh_test::run_mesh_vertices_where
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Faces Where",
    crate::mesh_test::run_mesh_faces_where
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Edges Where",
    crate::mesh_test::run_mesh_edges_where
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Vertices Where Predicate",
    crate::mesh_test::run_mesh_vertices_where_predicate
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Faces Where Predicate",
    crate::mesh_test::run_mesh_faces_where_predicate
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Edges Where Predicate",
    crate::mesh_test::run_mesh_edges_where_predicate
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Refresh Guid",
    crate::mesh_test::run_mesh_refresh_guid
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Assignment Keeps Objectcolor",
    crate::mesh_test::run_mesh_assignment_keeps_objectcolor
);
REGISTER_MINI_TEST!(
    "Mesh",
    "From Polyline Pairs",
    crate::mesh_test::run_mesh_from_polyline_pairs
);
REGISTER_MINI_TEST!(
    "Mesh",
    "From Polyline Pairs Vnf",
    crate::mesh_test::run_mesh_from_polyline_pairs_vnf
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Reflex Fold",
    crate::mesh_test::run_mesh_reflex_fold
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Miter Contours",
    crate::mesh_test::run_mesh_miter_contours
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Set Face Triangulation",
    crate::mesh_test::run_mesh_set_face_triangulation
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Set Face Holes",
    crate::mesh_test::run_mesh_set_face_holes
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Rebuild Halfedges",
    crate::mesh_test::run_mesh_rebuild_halfedges
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Ensure Halfedges",
    crate::mesh_test::run_mesh_ensure_halfedges
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Edge Face Map",
    crate::mesh_test::run_mesh_edge_face_map
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Face Outlines",
    crate::mesh_test::run_mesh_face_outlines
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Dihedral Angle",
    crate::mesh_test::run_mesh_dihedral_angle
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Triangle Bvh",
    crate::mesh_test::run_mesh_triangle_bvh
);
REGISTER_MINI_TEST!(
    "Mesh",
    "Triangle Aabb Tree",
    crate::mesh_test::run_mesh_triangle_aabb_tree
);
