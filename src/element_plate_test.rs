use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

///////////////////////////////////////////////////////////////////////////////////////////
// ElementPlate
///////////////////////////////////////////////////////////////////////////////////////////

pub fn run_plate_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::element::{Element, ElementGeometry};
        use crate::Point;

        let polygon = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
        ];
        let p = Element::plate(polygon.clone(), 0.2, "plate1");

        let name = &p.name;
        let guid = p.guid().to_string();
        let pstr = p.str();
        let prepr = p.repr();

        let pcopy = p.duplicate();
        let p2 = Element::plate(polygon.clone(), 0.2, "plate1");
        let p3 = Element::plate(polygon.clone(), 0.5, "plate1");

        MINI_CHECK!(name == "plate1");
        MINI_CHECK!(!guid.is_empty());
        MINI_CHECK!(matches!(p.geometry(), ElementGeometry::Mesh(_)));
        MINI_CHECK!(p.polygon().unwrap().len() == 4);
        MINI_CHECK!(p.thickness() == Some(0.2));
        MINI_CHECK!(pstr == "ElementPlate(plate1, 4 pts, 0.2)");
        MINI_CHECK!(prepr == format!("ElementPlate({}, plate1, 4 pts, 0.2)", guid));
        MINI_CHECK!(pcopy == p && pcopy.guid() != p.guid());
        MINI_CHECK!(p == p2);
        MINI_CHECK!(p != p3);
    })
}

pub fn run_plate_default_polygon() -> TestResult {
    MINI_TEST!("Default Polygon", {
        use crate::element::{Element, ElementGeometry};

        let p = Element::plate_default();

        MINI_CHECK!(matches!(p.geometry(), ElementGeometry::Mesh(_)));
        MINI_CHECK!(p.polygon().unwrap().len() == 4);
        MINI_CHECK!(p.thickness() == Some(0.1));
    })
}

pub fn run_plate_setters() -> TestResult {
    MINI_TEST!("Setters", {
        use crate::element::Element;
        use crate::Point;

        let mut p = Element::plate_default();
        p.set_thickness(0.3);
        p.set_polygon(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
            Point::new(3.0, 3.0, 0.0),
            Point::new(0.0, 3.0, 0.0),
        ]);

        MINI_CHECK!(p.thickness() == Some(0.3));
        MINI_CHECK!(p.polygon().unwrap().len() == 4);
        MINI_CHECK!(p.has_geometry());
    })
}

pub fn run_plate_mesh_topology() -> TestResult {
    MINI_TEST!("Mesh Topology", {
        use crate::element::{Element, ElementGeometry};
        use crate::Point;

        let polygon = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let p = Element::plate(polygon, 0.5, "my_plate");

        if let ElementGeometry::Mesh(geo) = p.geometry() {
            MINI_CHECK!(geo.vertex.len() == 8);
            MINI_CHECK!(geo.face.len() == 6);
        }
    })
}

pub fn run_plate_aabb() -> TestResult {
    MINI_TEST!("AABB", {
        use crate::element::Element;
        use crate::Point;

        let polygon = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
        ];
        let mut p = Element::plate(polygon, 0.2, "my_plate");
        let aabb = p.aabb();

        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[2], 0.1));
    })
}

pub fn run_plate_compute_point() -> TestResult {
    MINI_TEST!("Compute Point", {
        use crate::element::Element;
        use crate::Point;

        let polygon = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
        ];
        let mut p = Element::plate(polygon, 0.2, "my_plate");
        let pt = p.point();

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], -0.1));
    })
}

pub fn run_plate_triangle_polygon() -> TestResult {
    MINI_TEST!("Triangle Polygon", {
        use crate::element::{Element, ElementGeometry};
        use crate::Point;

        let polygon = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(0.5, 1.0, 0.0),
        ];
        let p = Element::plate(polygon, 0.1, "my_plate");

        if let ElementGeometry::Mesh(geo) = p.geometry() {
            MINI_CHECK!(geo.vertex.len() == 6);
            MINI_CHECK!(geo.face.len() == 5);
        }
    })
}

pub fn run_plate_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::element::Element;
        use crate::Point;

        let polygon = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
        ];
        let p = Element::plate(polygon, 0.3, "json_plate");

        let fname = "serialization/test_plate_element.json";
        p.file_json_dump(fname);
        let loaded = Element::file_json_load(fname);

        MINI_CHECK!(loaded.name == "json_plate");
        MINI_CHECK!(TOLERANCE.is_close(loaded.thickness().unwrap(), 0.3));
        MINI_CHECK!(loaded.polygon().unwrap().len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(loaded.polygon().unwrap()[1][0], 2.0));
    })
}

pub fn run_plate_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::element::Element;
        use crate::Point;

        let polygon = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
        ];
        let p = Element::plate(polygon, 0.3, "proto_plate");

        let path = "serialization/test_plate_element.bin";
        p.pb_dump(path);
        let loaded = Element::pb_load(path).unwrap();

        MINI_CHECK!(loaded.name == "proto_plate");
        MINI_CHECK!(TOLERANCE.is_close(loaded.thickness().unwrap(), 0.3));
        MINI_CHECK!(loaded.polygon().unwrap().len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(loaded.polygon().unwrap()[1][0], 2.0));
    })
}

pub fn run_plate_from_top_bottom() -> TestResult {
    MINI_TEST!("From Top Bottom", {
        use crate::element::Element;
        use crate::Point;

        let bottom = vec![Point::new(0.0,0.0,0.0), Point::new(2.0,0.0,0.0), Point::new(2.0,2.0,0.0), Point::new(0.0,2.0,0.0), Point::new(0.0,0.0,0.0)];
        let top    = vec![Point::new(0.0,0.0,1.0), Point::new(2.0,0.0,1.0), Point::new(2.0,2.0,1.0), Point::new(0.0,2.0,1.0), Point::new(0.0,0.0,1.0)];
        let p = Element::plate_from_top_bottom(bottom.clone(), top.clone(), "tb_plate");
        MINI_CHECK!(p.polygon().unwrap().len() == 4);
        MINI_CHECK!(p.polygon_top().unwrap().len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(p.thickness().unwrap(), 1.0));
        MINI_CHECK!(TOLERANCE.is_close(p.polygon().unwrap()[0][2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(p.polygon_top().unwrap()[0][2], 1.0));
        // Reversed argument order should auto-swap
        let pr = Element::plate_from_top_bottom(top, bottom, "tb_plate_r");
        MINI_CHECK!(TOLERANCE.is_close(pr.polygon().unwrap()[0][2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pr.polygon_top().unwrap()[0][2], 1.0));
    })
}

///////////////////////////////////////////////////////////////////////////////////////////
// ElementPlate - Polylines/Planes/Edge Vectors/Axis/Joinery
///////////////////////////////////////////////////////////////////////////////////////////

pub fn run_plate_polylines() -> TestResult {
    MINI_TEST!("Polylines", {
        use crate::element::Element;
        use crate::Point;
        let polygon = vec![Point::new(0.0,0.0,0.0), Point::new(1.0,0.0,0.0), Point::new(1.0,1.0,0.0), Point::new(0.0,1.0,0.0)];
        let mut p = Element::plate(polygon, 0.2, "my_plate");
        let pls = p.polylines();
        MINI_CHECK!(pls.len() == 6);
        MINI_CHECK!(pls[0].point_count() == 5);
        MINI_CHECK!(pls[1].point_count() == 5);
        for i in 2..6 { MINI_CHECK!(pls[i].point_count() == 5); }
    })
}

pub fn run_plate_planes() -> TestResult {
    MINI_TEST!("Planes", {
        use crate::element::Element;
        use crate::Point;
        let polygon = vec![Point::new(0.0,0.0,0.0), Point::new(1.0,0.0,0.0), Point::new(1.0,1.0,0.0), Point::new(0.0,1.0,0.0)];
        let mut p = Element::plate(polygon, 0.2, "my_plate");
        let pls = p.planes();
        MINI_CHECK!(pls.len() == 6);
        MINI_CHECK!(TOLERANCE.is_close(pls[0].z_axis()[2], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pls[1].z_axis()[2], -1.0));
    })
}

pub fn run_plate_edge_vectors() -> TestResult {
    MINI_TEST!("Edge Vectors", {
        use crate::element::Element;
        use crate::Point;
        let polygon = vec![Point::new(0.0,0.0,0.0), Point::new(1.0,0.0,0.0), Point::new(1.0,1.0,0.0), Point::new(0.0,1.0,0.0)];
        let mut p = Element::plate(polygon, 0.2, "my_plate");
        let evs = p.edge_vectors();
        MINI_CHECK!(evs.len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(evs[0][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(evs[0][1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(evs[1][0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(evs[1][1], 1.0));
    })
}

pub fn run_plate_axis() -> TestResult {
    MINI_TEST!("Axis", {
        use crate::element::Element;
        use crate::Point;
        let polygon = vec![Point::new(0.0,0.0,0.0), Point::new(2.0,0.0,0.0), Point::new(2.0,2.0,0.0), Point::new(0.0,2.0,0.0)];
        let mut p = Element::plate(polygon, 0.4, "my_plate");
        let ax = p.axis().unwrap();
        MINI_CHECK!(TOLERANCE.is_close(ax.start()[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(ax.start()[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(ax.start()[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(ax.end()[2], -0.4));
    })
}

pub fn run_plate_joint_types() -> TestResult {
    MINI_TEST!("Joint Types", {
        use crate::element::Element;
        let mut p = Element::plate_default();
        MINI_CHECK!(p.joint_types().unwrap().is_empty());
        p.set_joint_types(vec![1, 2, 3, 4]);
        MINI_CHECK!(p.joint_types().unwrap().len() == 4);
        MINI_CHECK!(p.joint_types().unwrap()[0] == 1);
        MINI_CHECK!(p.joint_types().unwrap()[3] == 4);
    })
}

pub fn run_plate_j_mf() -> TestResult {
    MINI_TEST!("J Mf", {
        use crate::element::Element;
        let mut p = Element::plate_default();
        MINI_CHECK!(p.j_mf().unwrap().is_empty());
        p.set_j_mf(vec![
            vec![(0, true, 0.5), (1, false, 0.3)],
            vec![],
            vec![(2, true, 0.8)],
        ]);
        MINI_CHECK!(p.j_mf().unwrap().len() == 3);
        MINI_CHECK!(p.j_mf().unwrap()[0].len() == 2);
        MINI_CHECK!(p.j_mf().unwrap()[0][0] == (0, true, 0.5));
        MINI_CHECK!(p.j_mf().unwrap()[2][0].0 == 2);
    })
}

pub fn run_plate_key() -> TestResult {
    MINI_TEST!("Key", {
        use crate::element::Element;
        let mut p = Element::plate_default();
        MINI_CHECK!(p.key().unwrap() == "");
        p.set_key("plate_A".to_string());
        MINI_CHECK!(p.key().unwrap() == "plate_A");
    })
}

pub fn run_plate_component_plane() -> TestResult {
    MINI_TEST!("Component Plane", {
        use crate::element::Element;
        use crate::plane::Plane;
        use crate::Point;
        use crate::Vector;
        let mut p = Element::plate_default();
        MINI_CHECK!(p.component_plane().is_none());
        let cp = Plane::new(Point::new(1.0, 2.0, 3.0), Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0));
        p.set_component_plane(cp);
        MINI_CHECK!(TOLERANCE.is_close(p.component_plane().unwrap().origin()[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(p.component_plane().unwrap().origin()[1], 2.0));
    })
}

pub fn run_plate_json_roundtrip_joinery() -> TestResult {
    MINI_TEST!("Json Roundtrip Joinery", {
        use crate::element::Element;
        use crate::plane::Plane;
        use crate::Point;
        use crate::Vector;

        let polygon = vec![Point::new(0.0,0.0,0.0), Point::new(2.0,0.0,0.0), Point::new(2.0,2.0,0.0), Point::new(0.0,2.0,0.0)];
        let mut p = Element::plate(polygon, 0.3, "joinery_plate");
        p.set_joint_types(vec![1, 2, 3, 4]);
        p.set_j_mf(vec![vec![(0, true, 0.5)], vec![], vec![(1, false, 0.3)]]);
        p.set_key("plate_A".to_string());
        p.set_component_plane(Plane::new(Point::new(1.0, 2.0, 3.0), Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0)));

        let fname = "serialization/test_plate_element_joinery.json";
        p.file_json_dump(fname);
        let loaded = Element::file_json_load(fname);

        MINI_CHECK!(loaded.joint_types().unwrap() == &vec![1, 2, 3, 4]);
        MINI_CHECK!(loaded.j_mf().unwrap().len() == 3);
        MINI_CHECK!(loaded.key().unwrap() == "plate_A");
        MINI_CHECK!(loaded.component_plane().is_some());
        MINI_CHECK!(TOLERANCE.is_close(loaded.component_plane().unwrap().origin()[0], 1.0));
    })
}

pub fn run_plate_protobuf_roundtrip_joinery() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip Joinery", {
        use crate::element::Element;
        use crate::plane::Plane;
        use crate::Point;
        use crate::Vector;

        let polygon = vec![Point::new(0.0,0.0,0.0), Point::new(2.0,0.0,0.0), Point::new(2.0,2.0,0.0), Point::new(0.0,2.0,0.0)];
        let mut p = Element::plate(polygon, 0.3, "joinery_plate");
        p.set_joint_types(vec![1, 2, 3, 4]);
        p.set_j_mf(vec![vec![(0, true, 0.5)], vec![], vec![(1, false, 0.3)]]);
        p.set_key("plate_A".to_string());
        p.set_component_plane(Plane::new(Point::new(1.0, 2.0, 3.0), Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0)));

        let path = "serialization/test_plate_element_joinery.bin";
        p.pb_dump(path);
        let loaded = Element::pb_load(path).unwrap();

        MINI_CHECK!(loaded.joint_types().unwrap() == &vec![1, 2, 3, 4]);
        MINI_CHECK!(loaded.j_mf().unwrap().len() == 3);
        MINI_CHECK!(loaded.j_mf().unwrap()[0][0] == (0, true, 0.5));
        MINI_CHECK!(loaded.key().unwrap() == "plate_A");
        MINI_CHECK!(loaded.component_plane().is_some());
        MINI_CHECK!(TOLERANCE.is_close(loaded.component_plane().unwrap().origin()[0], 1.0));
    })
}

///////////////////////////////////////////////////////////////////////////////////////////
// Registration
///////////////////////////////////////////////////////////////////////////////////////////

REGISTER_MINI_TEST!("ElementPlate", "Constructor", crate::element_plate_test::run_plate_constructor);
REGISTER_MINI_TEST!("ElementPlate", "Default Polygon", crate::element_plate_test::run_plate_default_polygon);
REGISTER_MINI_TEST!("ElementPlate", "Setters", crate::element_plate_test::run_plate_setters);
REGISTER_MINI_TEST!("ElementPlate", "Mesh Topology", crate::element_plate_test::run_plate_mesh_topology);
REGISTER_MINI_TEST!("ElementPlate", "AABB", crate::element_plate_test::run_plate_aabb);
REGISTER_MINI_TEST!("ElementPlate", "Compute Point", crate::element_plate_test::run_plate_compute_point);
REGISTER_MINI_TEST!("ElementPlate", "Triangle Polygon", crate::element_plate_test::run_plate_triangle_polygon);
REGISTER_MINI_TEST!("ElementPlate", "Json Roundtrip", crate::element_plate_test::run_plate_json_roundtrip);
REGISTER_MINI_TEST!("ElementPlate", "Protobuf Roundtrip", crate::element_plate_test::run_plate_protobuf_roundtrip);
REGISTER_MINI_TEST!("ElementPlate", "From Top Bottom", crate::element_plate_test::run_plate_from_top_bottom);
REGISTER_MINI_TEST!("ElementPlate", "Polylines", crate::element_plate_test::run_plate_polylines);
REGISTER_MINI_TEST!("ElementPlate", "Planes", crate::element_plate_test::run_plate_planes);
REGISTER_MINI_TEST!("ElementPlate", "Edge Vectors", crate::element_plate_test::run_plate_edge_vectors);
REGISTER_MINI_TEST!("ElementPlate", "Axis", crate::element_plate_test::run_plate_axis);
REGISTER_MINI_TEST!("ElementPlate", "Joint Types", crate::element_plate_test::run_plate_joint_types);
REGISTER_MINI_TEST!("ElementPlate", "J Mf", crate::element_plate_test::run_plate_j_mf);
REGISTER_MINI_TEST!("ElementPlate", "Key", crate::element_plate_test::run_plate_key);
REGISTER_MINI_TEST!("ElementPlate", "Component Plane", crate::element_plate_test::run_plate_component_plane);
REGISTER_MINI_TEST!("ElementPlate", "Json Roundtrip Joinery", crate::element_plate_test::run_plate_json_roundtrip_joinery);
REGISTER_MINI_TEST!("ElementPlate", "Protobuf Roundtrip Joinery", crate::element_plate_test::run_plate_protobuf_roundtrip_joinery);
