use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_remesh_cdt_triangulate() -> TestResult {
    MINI_TEST!("Triangulate", {
        use crate::remesh_cdt::RemeshCDT;
        use crate::Polyline;
        use crate::Point;
        use crate::Mesh;

        let border = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 4.0, 0.0),
            Point::new(0.0, 4.0, 0.0),
        ]);
        let hole = Polyline::new(vec![
            Point::new(1.0, 1.0, 0.0),
            Point::new(1.0, 3.0, 0.0),
            Point::new(3.0, 3.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
        ]);
        let tris = RemeshCDT::triangulate(&[border.clone(), hole.clone()]);
        let mut flat = Vec::new();
        for p in border.get_points() { flat.push(p); }
        for p in hole.get_points()   { flat.push(p); }
        let mut m = Mesh::new();
        let mut vkeys = Vec::new();
        for p in &flat { vkeys.push(m.add_vertex(p.clone(), None)); }
        for t in &tris { m.add_face(vec![vkeys[t.0], vkeys[t.1], vkeys[t.2]], None); }

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_remesh_cdt_triangle() -> TestResult {
    MINI_TEST!("Triangle", {
        use crate::remesh_cdt::RemeshCDT;
        use crate::Polyline;
        use crate::Point;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);
        let m = RemeshCDT::from_polylines(&[pl], false, true);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_remesh_cdt_rectangle() -> TestResult {
    MINI_TEST!("Rectangle", {
        use crate::remesh_cdt::RemeshCDT;
        use crate::Polyline;
        use crate::Point;

        let pl = Polyline::new(vec![
            Point::new(3.0, 0.0, 0.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(5.0, 2.0, 0.0),
            Point::new(3.0, 2.0, 0.0),
        ]);
        let m = RemeshCDT::from_polylines(&[pl], false, true);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_remesh_cdt_l_shape() -> TestResult {
    MINI_TEST!("L-shape", {
        use crate::remesh_cdt::RemeshCDT;
        use crate::Polyline;
        use crate::Point;

        let pl = Polyline::new(vec![
            Point::new(7.0, 0.0, 0.0),
            Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 1.0, 0.0),
            Point::new(8.0, 1.0, 0.0),
            Point::new(8.0, 3.0, 0.0),
            Point::new(7.0, 3.0, 0.0),
        ]);
        let m = RemeshCDT::from_polylines(&[pl], false, true);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_remesh_cdt_u_shape() -> TestResult {
    MINI_TEST!("U-shape", {
        use crate::remesh_cdt::RemeshCDT;
        use crate::Polyline;
        use crate::Point;

        let pl = Polyline::new(vec![
            Point::new(25.0, 0.0, 0.0),
            Point::new(31.0, 0.0, 0.0),
            Point::new(31.0, 4.0, 0.0),
            Point::new(29.0, 4.0, 0.0),
            Point::new(29.0, 2.0, 0.0),
            Point::new(27.0, 2.0, 0.0),
            Point::new(27.0, 4.0, 0.0),
            Point::new(25.0, 4.0, 0.0),
        ]);
        let m = RemeshCDT::from_polylines(&[pl], false, true);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_remesh_cdt_octagon() -> TestResult {
    MINI_TEST!("Octagon", {
        use crate::remesh_cdt::RemeshCDT;
        use crate::Polyline;
        use crate::Vector;

        let mut pl = Polyline::from_sides(8, 1.5, false);
        pl += &Vector::new(14.0, 1.5, 0.0);
        let m = RemeshCDT::from_polylines(&[pl], false, true);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_remesh_cdt_rectangle_with_rectangle_hole() -> TestResult {
    MINI_TEST!("Rectangle with rectangle hole", {
        use crate::remesh_cdt::RemeshCDT;
        use crate::Polyline;
        use crate::Point;

        let border = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 4.0, 0.0),
            Point::new(0.0, 4.0, 0.0),
        ]);
        let hole = Polyline::new(vec![
            Point::new(1.0, 1.0, 0.0),
            Point::new(1.0, 3.0, 0.0),
            Point::new(3.0, 3.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
        ]);
        let m = RemeshCDT::from_polylines(&[border, hole], false, true);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_remesh_cdt_duplicate_vertices() -> TestResult {
    MINI_TEST!("Duplicate vertices", {
        use crate::remesh_cdt::RemeshCDT;
        use crate::Polyline;
        use crate::Point;

        let pl = Polyline::new(vec![
            Point::new(33.0, 0.0, 0.0),
            Point::new(36.0, 0.0, 0.0),
            Point::new(37.0, 2.0, 0.0),
            Point::new(35.0, 3.0, 0.0),
            Point::new(33.0, 2.0, 0.0),
            Point::new(33.0, 0.0, 0.0),
        ]);
        let m = RemeshCDT::from_polylines(&[pl], false, true);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_remesh_cdt_tilted_rectangle_with_rectangle_hole() -> TestResult {
    MINI_TEST!("Tilted rectangle with rectangle hole", {
        use crate::remesh_cdt::RemeshCDT;
        use crate::Polyline;
        use crate::Point;

        let m = RemeshCDT::from_polylines(&[
            Polyline::new(vec![
                Point::new(55.0, 0.0, 0.0),
                Point::new(62.0, 0.0, 0.0),
                Point::new(62.0, 4.0, 2.0),
                Point::new(55.0, 4.0, 2.0),
            ]),
            Polyline::new(vec![
                Point::new(56.0, 1.0, 0.5),
                Point::new(61.0, 1.0, 0.5),
                Point::new(61.0, 3.0, 1.5),
                Point::new(56.0, 3.0, 1.5),
            ]),
        ], false, false);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_remesh_cdt_irregular_tilted_polyline() -> TestResult {
    MINI_TEST!("Irregular tilted polyline.", {
        use crate::remesh_cdt::RemeshCDT;
        use crate::Polyline;
        use crate::Point;

        let border = vec![
            Point::new(125.390575, 14.236865, -16.468853),
            Point::new(115.72382, 17.091624, -3.285212),
            Point::new(121.789447, 21.876306, 18.811062),
            Point::new(151.252023, 21.746582, 18.211977),
            Point::new(142.916699, 15.485629, -10.701904),
            Point::new(135.657716, 19.277143, 6.807792),
            Point::new(142.118715, 19.142866, 6.187686),
            Point::new(141.57586, 17.857892, 0.253509),
            Point::new(140.11301, 18.526023, 3.339026),
            Point::new(141.21524, 18.751613, 4.38083),
            Point::new(141.292725, 18.989286, 5.478431),
            Point::new(139.566886, 19.055145, 5.78258),
            Point::new(138.763256, 18.580689, 3.59148),
            Point::new(139.310849, 18.055272, 1.165039),
            Point::new(140.386846, 17.829745, 0.123525),
            Point::new(140.874682, 17.186517, -2.846986),
            Point::new(142.977469, 16.726916, -4.969483),
            Point::new(144.463527, 17.732299, -0.326495),
            Point::new(144.003555, 19.468828, 7.693018),
            Point::new(133.951268, 20.010289, 10.193557),
            Point::new(143.92422, 20.548103, 12.677251),
            Point::new(134.28842, 20.656763, 13.179053),
            Point::new(141.475986, 21.027044, 14.889059),
            Point::new(148.395425, 21.092848, 15.192952),
            Point::new(146.816239, 20.209864, 11.115218),
            Point::new(140.129758, 20.039236, 10.327236),
            Point::new(145.879946, 19.943669, 9.885895),
            Point::new(146.207165, 18.815694, 4.676763),
            Point::new(149.810987, 21.502249, 17.083618),
            Point::new(123.62156, 21.64297, 17.733487),
            Point::new(143.846461, 21.388459, 16.558122),
            Point::new(123.531701, 20.756807, 13.641072),
            Point::new(131.906468, 20.861163, 14.123002),
            Point::new(134.607416, 15.741609, -9.519754),
            Point::new(131.232551, 20.580328, 12.826068),
            Point::new(122.959207, 20.008058, 10.183251),
            Point::new(129.590699, 18.785461, 4.537144),
            Point::new(122.199489, 18.387709, 2.700273),
            Point::new(127.811443, 17.061665, -3.423569),
            Point::new(132.763864, 14.563498, -14.960425),
            Point::new(137.425771, 15.246311, -11.807104),
            Point::new(133.976431, 19.415144, 7.445098),
            Point::new(141.147246, 14.792123, -13.904602),
            Point::new(134.321155, 13.729688, -18.811062),
            Point::new(126.514179, 16.245401, -7.193181),
            Point::new(122.064196, 16.141604, -7.672527),
            Point::new(129.662786, 15.027072, -12.81958),
            Point::new(125.390575, 14.236865, -16.468853),
        ];
        let m = RemeshCDT::from_polylines(&[Polyline::new(border)], false, true);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_remesh_cdt_irregular_tilted_polyline_with_holes() -> TestResult {
    MINI_TEST!("Irregular tilted polyline with holes.", {
        use crate::remesh_cdt::RemeshCDT;
        use crate::Polyline;
        use crate::Point;

        let border = vec![
            Point::new(80.805571, 2.103432, 0.0),
            Point::new(77.056348, 6.382318, 0.0),
            Point::new(73.469325, 10.712368, 0.0),
            Point::new(76.913186, 20.429093, 0.0),
            Point::new(81.089809, 22.150636, 0.0),
            Point::new(80.489618, 16.428762, 0.0),
            Point::new(77.593256, 13.859214, 0.0),
            Point::new(79.137808, 10.314549, 0.0),
            Point::new(81.991647, 13.383871, 0.0),
            Point::new(82.6268, 17.180658, 0.0),
            Point::new(83.346466, 22.810648, 0.0),
            Point::new(80.646064, 27.128804, 0.0),
            Point::new(75.409533, 25.864862, 0.0),
            Point::new(72.096289, 20.679753, 0.0),
            Point::new(69.174993, 24.651843, 0.0),
            Point::new(73.459453, 30.365867, 0.0),
            Point::new(78.285356, 32.047254, 0.0),
            Point::new(84.294064, 31.313912, 0.0),
            Point::new(88.751527, 27.70307, 0.0),
            Point::new(89.863118, 24.247859, 0.0),
            Point::new(86.375694, 25.536811, 0.0),
            Point::new(85.671769, 28.704182, 0.0),
            Point::new(83.931992, 29.140561, 0.0),
            Point::new(83.87182, 26.993861, 0.0),
            Point::new(85.013894, 23.331116, 0.0),
            Point::new(86.696592, 22.050319, 0.0),
            Point::new(91.11007, 21.775838, 0.0),
            Point::new(93.068864, 24.22017, 0.0),
            Point::new(90.710749, 28.565741, 0.0),
            Point::new(91.638895, 29.907121, 0.0),
            Point::new(98.350352, 28.577586, 0.0),
            Point::new(97.932933, 19.2177, 0.0),
            Point::new(92.735476, 16.088655, 0.0),
            Point::new(88.933069, 18.173691, 0.0),
            Point::new(87.14089, 18.175371, 0.0),
            Point::new(88.303603, 12.293181, 0.0),
            Point::new(95.930447, 12.110783, 0.0),
            Point::new(99.851107, 14.725665, 0.0),
            Point::new(100.978975, 20.024267, 0.0),
            Point::new(102.243884, 24.855315, 0.0),
            Point::new(107.033349, 22.799502, 0.0),
            Point::new(106.609681, 11.898217, 0.0),
            Point::new(103.249112, 7.789628, 0.0),
            Point::new(96.095448, 7.274641, 0.0),
            Point::new(89.385773, 5.965419, 0.0),
            Point::new(96.860717, 2.216133, 0.0),
            Point::new(100.861767, 4.379475, 0.0),
            Point::new(101.881544, -0.42956, 0.0),
            Point::new(97.45084, -3.301412, 0.0),
            Point::new(91.973637, -1.202488, 0.0),
            Point::new(85.52513, 1.867323, 0.0),
            Point::new(84.223176, 6.692841, 0.0),
            Point::new(89.798972, 8.864768, 0.0),
            Point::new(95.492277, 9.295981, 0.0),
            Point::new(99.888391, 10.064787, 0.0),
            Point::new(103.261854, 11.705226, 0.0),
            Point::new(103.779131, 19.764949, 0.0),
            Point::new(102.711068, 14.994661, 0.0),
            Point::new(99.443654, 12.155559, 0.0),
            Point::new(97.732698, 11.322088, 0.0),
            Point::new(90.50953, 9.911877, 0.0),
            Point::new(85.903966, 9.907526, 0.0),
            Point::new(84.501578, 11.767742, 0.0),
            Point::new(85.498912, 16.396132, 0.0),
            Point::new(82.205875, 10.374609, 0.0),
            Point::new(79.29099, 8.223327, 0.0),
            Point::new(82.140367, 5.90496, 0.0),
            Point::new(80.805571, 2.103432, 0.0),
        ];
        let h1 = vec![
            Point::new(89.032991, 2.017152, 0.0),
            Point::new(87.733487, 4.228529, 0.0),
            Point::new(91.344022, 3.519583, 0.0),
            Point::new(94.484896, 1.422882, 0.0),
            Point::new(89.032991, 2.017152, 0.0),
        ];
        let h2 = vec![
            Point::new(92.03771, 19.272441, 0.0),
            Point::new(93.023809, 18.394685, 0.0),
            Point::new(95.237342, 19.201565, 0.0),
            Point::new(96.523355, 23.096688, 0.0),
            Point::new(96.499215, 27.739772, 0.0),
            Point::new(93.834079, 27.897701, 0.0),
            Point::new(94.368093, 26.068002, 0.0),
            Point::new(94.196483, 22.936949, 0.0),
            Point::new(92.03771, 19.272441, 0.0),
        ];
        let h3 = vec![
            Point::new(75.250389, 29.32022, 0.0),
            Point::new(71.879629, 25.409869, 0.0),
            Point::new(72.297014, 23.977747, 0.0),
            Point::new(74.564259, 26.656863, 0.0),
            Point::new(77.684277, 29.043429, 0.0),
            Point::new(81.250681, 29.040871, 0.0),
            Point::new(81.897777, 29.877526, 0.0),
            Point::new(80.430576, 30.631228, 0.0),
            Point::new(75.250389, 29.32022, 0.0),
        ];
        let h4 = vec![
            Point::new(78.759389, 19.25978, 0.0),
            Point::new(79.548907, 18.00544, 0.0),
            Point::new(77.949677, 16.331313, 0.0),
            Point::new(77.272732, 16.707375, 0.0),
            Point::new(78.759389, 19.25978, 0.0),
        ];
        let m = RemeshCDT::from_polylines(
            &[
                Polyline::new(border),
                Polyline::new(h1),
                Polyline::new(h2),
                Polyline::new(h3),
                Polyline::new(h4),
            ],
            false, false);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_remesh_cdt_large_coordinates() -> TestResult {
    MINI_TEST!("Large coordinates", {
        use crate::remesh_cdt::RemeshCDT;
        use crate::Polyline;
        use crate::Point;

        let border = Polyline::new(vec![
            Point::new(1e13, 1e13, 0.0),
            Point::new(1e13 + 4.0, 1e13, 0.0),
            Point::new(1e13 + 4.0, 1e13 + 4.0, 0.0),
            Point::new(1e13, 1e13 + 4.0, 0.0),
        ]);
        let tris = RemeshCDT::triangulate(&[border]);

        MINI_CHECK!(tris.len() == 2);
    })
}

pub fn run_remesh_cdt_plate_failing_15_vert_outer_4_holes() -> TestResult {
    MINI_TEST!("plate_failing 15-vert outer + 4 holes", {
        use crate::remesh_cdt::RemeshCDT;
        use crate::Polyline;
        use crate::Point;

        let border = vec![
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
        ];
        let h1 = vec![
            Point::new(322.544527, -1917.0, 270.089054),
            Point::new(213.417326, -1917.0, 51.834651),
            Point::new(199.266354, -1869.830094, 58.910137),
            Point::new(308.393555, -1869.830094, 277.16454),
            Point::new(322.544527, -1917.0, 270.089054),
        ];
        let h2 = vec![
            Point::new(540.79893, -1917.0, 706.59786),
            Point::new(431.671728, -1917.0, 488.343457),
            Point::new(417.520757, -1869.830094, 495.418943),
            Point::new(526.647958, -1869.830094, 713.673346),
            Point::new(540.79893, -1917.0, 706.59786),
        ];
        let h3 = vec![
            Point::new(424.153936, -1667.753669, 660.242619),
            Point::new(526.289465, -1735.844022, 813.445914),
            Point::new(530.261328, -1770.507552, 795.391992),
            Point::new(428.125798, -1702.417199, 642.188697),
            Point::new(424.153936, -1667.753669, 660.242619),
        ];
        let h4 = vec![
            Point::new(219.882876, -1531.572963, 353.83603),
            Point::new(322.018406, -1599.663316, 507.039325),
            Point::new(325.990269, -1634.326846, 488.985403),
            Point::new(223.854739, -1566.236493, 335.782108),
            Point::new(219.882876, -1531.572963, 353.83603),
        ];
        let m = RemeshCDT::from_polylines(
            &[
                Polyline::new(border),
                Polyline::new(h1),
                Polyline::new(h2),
                Polyline::new(h3),
                Polyline::new(h4),
            ],
            false, false);

        MINI_CHECK!(m.is_valid());
    })
}

REGISTER_MINI_TEST!("RemeshCDT", "Triangulate", crate::remesh_cdt_test::run_remesh_cdt_triangulate);
REGISTER_MINI_TEST!("RemeshCDT", "Triangle", crate::remesh_cdt_test::run_remesh_cdt_triangle);
REGISTER_MINI_TEST!("RemeshCDT", "Rectangle", crate::remesh_cdt_test::run_remesh_cdt_rectangle);
REGISTER_MINI_TEST!("RemeshCDT", "L-shape", crate::remesh_cdt_test::run_remesh_cdt_l_shape);
REGISTER_MINI_TEST!("RemeshCDT", "U-shape", crate::remesh_cdt_test::run_remesh_cdt_u_shape);
REGISTER_MINI_TEST!("RemeshCDT", "Octagon", crate::remesh_cdt_test::run_remesh_cdt_octagon);
REGISTER_MINI_TEST!("RemeshCDT", "Rectangle with rectangle hole", crate::remesh_cdt_test::run_remesh_cdt_rectangle_with_rectangle_hole);
REGISTER_MINI_TEST!("RemeshCDT", "Duplicate vertices", crate::remesh_cdt_test::run_remesh_cdt_duplicate_vertices);
REGISTER_MINI_TEST!("RemeshCDT", "Tilted rectangle with rectangle hole", crate::remesh_cdt_test::run_remesh_cdt_tilted_rectangle_with_rectangle_hole);
REGISTER_MINI_TEST!("RemeshCDT", "Irregular tilted polyline.", crate::remesh_cdt_test::run_remesh_cdt_irregular_tilted_polyline);
REGISTER_MINI_TEST!("RemeshCDT", "Irregular tilted polyline with holes.", crate::remesh_cdt_test::run_remesh_cdt_irregular_tilted_polyline_with_holes);
REGISTER_MINI_TEST!("RemeshCDT", "plate_failing 15-vert outer + 4 holes", crate::remesh_cdt_test::run_remesh_cdt_plate_failing_15_vert_outer_4_holes);
REGISTER_MINI_TEST!("RemeshCDT", "Large coordinates", crate::remesh_cdt_test::run_remesh_cdt_large_coordinates);
