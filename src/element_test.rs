use crate::mini_test::TestResult;
use crate::tolerance::PI;
use crate::tolerance::TOLERANCE;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

// ═══════════════════════════════════════════════════════════════════════════
// Element
// ═══════════════════════════════════════════════════════════════════════════

pub fn run_element_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::element::Element;
        use crate::element::ElementGeometry;
        use crate::BRep;
        use crate::Mesh;
        use crate::Point;

        let m = Mesh::from_vertices_and_faces(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![vec![0, 1, 2, 3]],
        );
        let e = Element::from_mesh(m, "test_element");

        let geo = e.geometry();
        let name = &e.name;
        let guid = e.guid().to_string();
        let dirty = e.is_dirty();

        let estr = e.str();
        let erepr = e.repr();

        let ecopy = e.duplicate();

        let e2 = Element::from_mesh(Mesh::new(), "test_element");
        let e3 = Element::from_brep(BRep::new(), "other");

        MINI_CHECK!(name == "test_element");
        MINI_CHECK!(!guid.is_empty());
        MINI_CHECK!(dirty);
        MINI_CHECK!(matches!(geo, ElementGeometry::Mesh(_)));
        MINI_CHECK!(estr == "Element(test_element, Mesh)");
        MINI_CHECK!(erepr == format!("Element({}, test_element, Mesh)", guid));
        MINI_CHECK!(ecopy == e && ecopy.guid() != e.guid());
        MINI_CHECK!(e == e2);
        MINI_CHECK!(e != e3);
    })
}

pub fn run_element_place() -> TestResult {
    MINI_TEST!("Place", {
        use crate::element::Element;
        use crate::element::ElementGeometry;
        use crate::Mesh;
        use crate::Point;
        use crate::Xform;

        let m = Mesh::from_vertices_and_faces(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![vec![0, 1, 2, 3]],
        );
        let mut e = Element::from_mesh(m, "my_element");
        let xf = Xform::translation(10.0, 20.0, 30.0);
        e.place(&xf);

        MINI_CHECK!(e.is_dirty());

        if let ElementGeometry::Mesh(mesh) = e.geometry() {
            let mut min_x = f64::MAX;

            for v in mesh.vertex.values() {
                min_x = min_x.min(v.position()[0]);
            }

            MINI_CHECK!(min_x > 9.0);
        }
    })
}

pub fn run_element_place_moves_features() -> TestResult {
    MINI_TEST!("Place Moves Features", {
        use crate::element::Element;
        use crate::element::ElementFeature;
        use crate::Mesh;
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;
        use crate::Xform;

        let m = Mesh::from_vertices_and_faces(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![vec![0, 1, 2, 3]],
        );
        let mut e = Element::from_mesh(m, "my_element");
        e.add_feature(ElementFeature::new(
            "contact",
            0,
            vec![Polyline::new(vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
            ])],
            "",
        ));
        e.set_insertion_vectors(vec![Vector::new(1.0, 0.0, 0.0)]);
        let guid = e.features()[0].guid().to_string();
        e.place(&(&Xform::translation(0.0, 0.0, 5.0) * &Xform::rotation_z(PI / 2.0, false)));

        let moved = e.features()[0].outlines[0].get_point(1).unwrap();
        let turned = e.insertion_vectors()[0].clone();

        MINI_CHECK!(
            TOLERANCE.is_close(moved[0], 0.0)
                && TOLERANCE.is_close(moved[1], 1.0)
                && TOLERANCE.is_close(moved[2], 5.0)
        );
        MINI_CHECK!(
            TOLERANCE.is_close(turned[0], 0.0)
                && TOLERANCE.is_close(turned[1], 1.0)
                && TOLERANCE.is_close(turned[2], 0.0)
        );
        MINI_CHECK!(e.features()[0].guid() == guid);
    })
}

pub fn run_element_add_geometry_op() -> TestResult {
    MINI_TEST!("Add Geometry Op", {
        use crate::element::Element;
        use crate::element::ElementGeometry;
        use crate::BRep;
        use crate::Mesh;
        use crate::Point;
        use crate::Xform;

        let m = Mesh::from_vertices_and_faces(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![vec![0, 1, 2, 3]],
        );
        fn my_feature(geo: Mesh) -> Mesh {
            geo
        }

        fn empty_mesh(_geo: Mesh) -> Mesh {
            Mesh::new()
        }

        let mut e = Element::from_mesh(m, "my_element");
        e.add_geometry_op(my_feature);

        let mut eb = Element::from_brep(BRep::create_box(1.0, 1.0, 1.0), "brep_feature");
        eb.add_geometry_op(empty_mesh);
        let sg = eb.session_geometry(&Xform::identity());

        MINI_CHECK!(e.is_dirty());
        MINI_CHECK!(e.geometry_ops_count() == 1);
        MINI_CHECK!(matches!(sg, ElementGeometry::BRep(_)));
    })
}

pub fn run_element_aabb() -> TestResult {
    MINI_TEST!("AABB", {
        use crate::Element;
        use crate::Mesh;
        use crate::Point;

        let m = Mesh::from_vertices_and_faces(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![vec![0, 1, 2, 3]],
        );
        let mut e = Element::from_mesh(m, "my_element");
        let aabb = e.aabb();

        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[0], 0.5));
        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[1], 0.5));
        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[2], 0.0));
        MINI_CHECK!(!e.is_dirty());

        fn my_feature(geo: Mesh) -> Mesh {
            geo
        }

        e.add_geometry_op(my_feature);

        MINI_CHECK!(e.is_dirty());
        MINI_CHECK!(e.cached_aabb().is_none());
    })
}

pub fn run_element_obb() -> TestResult {
    MINI_TEST!("OBB", {
        use crate::Element;
        use crate::Mesh;
        use crate::Point;

        let m = Mesh::from_vertices_and_faces(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![vec![0, 1, 2, 3]],
        );
        let mut e = Element::from_mesh(m, "my_element");
        let obb = e.obb();

        MINI_CHECK!(TOLERANCE.is_close(obb.half_size[0], 0.5));
        MINI_CHECK!(TOLERANCE.is_close(obb.half_size[1], 0.5));
    })
}

pub fn run_element_session_geometry() -> TestResult {
    MINI_TEST!("Session Geometry", {
        use crate::element::Element;
        use crate::element::ElementGeometry;
        use crate::Mesh;
        use crate::Point;
        use crate::Xform;

        let m = Mesh::from_vertices_and_faces(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![vec![0, 1, 2, 3]],
        );
        let e = Element::from_mesh(m, "my_element");
        let e_xf = Xform::translation(10.0, 0.0, 0.0);
        let sg = e.session_geometry(&e_xf);

        MINI_CHECK!(matches!(&sg, ElementGeometry::Mesh(_)));

        if let ElementGeometry::Mesh(mesh) = &sg {
            MINI_CHECK!(TOLERANCE.is_close(mesh.vertex[&0].position()[0], 10.0));
            MINI_CHECK!(TOLERANCE.is_close(mesh.vertex[&1].position()[0], 11.0));
            MINI_CHECK!(!std::ptr::eq(e.geometry(), &sg));
        }
    })
}

pub fn run_element_reset() -> TestResult {
    MINI_TEST!("Reset", {
        use crate::Element;
        use crate::Mesh;
        use crate::Point;

        let m = Mesh::from_vertices_and_faces(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(2.0, 0.0, 0.0),
                Point::new(2.0, 2.0, 0.0),
                Point::new(0.0, 2.0, 0.0),
            ],
            vec![vec![0, 1, 2, 3]],
        );
        let mut e = Element::from_mesh(m, "my_element");
        let _ = e.aabb();
        let _ = e.point();
        e.reset();

        MINI_CHECK!(e.is_dirty());
        MINI_CHECK!(e.cached_aabb().is_none());
        MINI_CHECK!(e.cached_obb().is_none());
        MINI_CHECK!(e.cached_collision_mesh().is_none());
        MINI_CHECK!(e.cached_point().is_none());
    })
}

pub fn run_element_compute_point() -> TestResult {
    MINI_TEST!("Compute Point", {
        use crate::Element;
        use crate::Mesh;
        use crate::Point;

        let m = Mesh::from_vertices_and_faces(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(2.0, 0.0, 0.0),
                Point::new(2.0, 2.0, 0.0),
                Point::new(0.0, 2.0, 0.0),
            ],
            vec![vec![0, 1, 2, 3]],
        );
        let mut e = Element::from_mesh(m, "my_element");
        let pt = e.point();

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 0.0));
    })
}

pub fn run_element_brep_aabb() -> TestResult {
    MINI_TEST!("Brep Aabb", {
        use crate::BRep;
        use crate::Element;

        let b = BRep::create_box(2.0, 3.0, 4.0);
        let mut e = Element::from_brep(b, "brep_element");
        let aabb = e.aabb();
        let pt = e.point();

        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[1], 1.5));
        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[2], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 0.0));
    })
}

pub fn run_element_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::element::Element;
        use crate::element::ElementGeometry;
        use crate::Mesh;
        use crate::Point;

        let m = Mesh::from_vertices_and_faces(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![vec![0, 1, 2, 3]],
        );
        let e = Element::from_mesh(m, "json_test");

        let fname = "serialization/test_element.json";
        e.file_json_dump(fname);
        let loaded = Element::file_json_load(fname);

        MINI_CHECK!(loaded.name == "json_test");
        MINI_CHECK!(matches!(loaded.geometry(), ElementGeometry::Mesh(_)));

        if let ElementGeometry::Mesh(mesh) = loaded.geometry() {
            MINI_CHECK!(mesh.vertex.len() == 4);
        }
    })
}

pub fn run_element_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::element::Element;
        use crate::element::ElementGeometry;
        use crate::BRep;

        let b = BRep::create_box(2.0, 3.0, 4.0);
        let e = Element::from_brep(b, "proto_test");

        let path = "serialization/test_element.bin";
        e.pb_dump(path);
        let loaded = Element::pb_load(path).unwrap();

        MINI_CHECK!(loaded.name == "proto_test");
        MINI_CHECK!(matches!(loaded.geometry(), ElementGeometry::BRep(_)));

        if let ElementGeometry::BRep(brep) = loaded.geometry() {
            MINI_CHECK!(brep.face_count() == 6);
            MINI_CHECK!(brep.vertex_count() == 8);
        }
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Element - Polylines
// ═══════════════════════════════════════════════════════════════════════════

pub fn run_element_polylines() -> TestResult {
    MINI_TEST!("Polylines", {
        use crate::Element;
        use crate::Mesh;
        use crate::Point;

        let m = Mesh::from_vertices_and_faces(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![vec![0, 1, 2, 3]],
        );
        let mut e = Element::from_mesh(m, "test_element");

        MINI_CHECK!(e.polylines().len() == 1);
        MINI_CHECK!(e.polylines()[0].point_count() == 5);
        MINI_CHECK!(e.polylines()[0].get_point(0) == Some(Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(e.polylines()[0].get_point(4) == Some(Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(e.planes().len() == 1);
        MINI_CHECK!(e.planes()[0].origin() == Point::new(0.5, 0.5, 0.0));

        let normal = e.planes()[0].z_axis();

        MINI_CHECK!(
            TOLERANCE.is_close(normal[0], 0.0)
                && TOLERANCE.is_close(normal[1], 0.0)
                && normal[2] > 0.0
        );
        MINI_CHECK!(e.edge_vectors().is_empty());
        MINI_CHECK!(e.axis().is_none());
    })
}

pub fn run_element_set_polylines_sticks() -> TestResult {
    MINI_TEST!("Set Polylines Sticks", {
        use crate::Element;
        use crate::Mesh;
        use crate::Plane;
        use crate::Point;
        use crate::Polyline;

        let m = Mesh::from_vertices_and_faces(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![vec![0, 1, 2, 3]],
        );
        let mut e = Element::from_mesh(m, "my_element");
        e.set_polylines(vec![Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ])]);
        e.set_planes(vec![Plane::xy_plane()]);

        MINI_CHECK!(e.polylines().len() == 1);
        MINI_CHECK!(e.polylines()[0].point_count() == 2);
        MINI_CHECK!(e.planes()[0].origin() == Point::new(0.0, 0.0, 0.0));
    })
}

pub fn run_element_polylines_empty_without_mesh() -> TestResult {
    MINI_TEST!("Polylines Empty Without Mesh", {
        use crate::Element;

        MINI_CHECK!(Element::new("no_geometry").polylines().is_empty());
        MINI_CHECK!(Element::new("no_geometry").planes().is_empty());
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Element - Polymorphic registry
// ═══════════════════════════════════════════════════════════════════════════

/// Stand-in for a domain package's factory: Rust has no derived type to return, so it marks the element it built.
fn test_plate(data: &[u8]) -> Option<crate::Element> {
    let mut e = crate::Element::pb_loads(data).ok()?;
    e.name = format!("{}_via_factory", e.name);

    Some(e)
}

/// Return a unit square mesh in the xy plane.
fn unit_quad() -> crate::Mesh {
    use crate::Point;

    crate::Mesh::from_vertices_and_faces(
        vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ],
        vec![vec![0, 1, 2, 3]],
    )
}

pub fn run_element_registry_round_trip() -> TestResult {
    MINI_TEST!("Registry Round Trip", {
        use crate::element::Element;
        use crate::element::ElementGeometry;

        Element::register_type("TestPlate", test_plate);

        MINI_CHECK!(Element::is_registered("TestPlate"));

        let mut plate = Element::from_mesh(unit_quad(), "plate_0");
        plate.element_type = "TestPlate".to_string();
        plate.element_data = b"12.5,30,11,20".to_vec();
        let guid = plate.guid().to_string();
        let loaded = Element::pb_loads_polymorphic(&plate.pb_dumps()).unwrap();
        let copy = plate.duplicate();

        MINI_CHECK!(loaded.name == "plate_0_via_factory");
        MINI_CHECK!(loaded.element_type_name() == "TestPlate");

        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(matches!(loaded.geometry(), ElementGeometry::Mesh(_)));
        MINI_CHECK!(loaded.element_data_dumps() == b"12.5,30,11,20");
        MINI_CHECK!(copy.element_data_dumps() == plate.element_data_dumps());
    })
}

pub fn run_element_registry_unknown_type_degrades() -> TestResult {
    MINI_TEST!("Registry Unknown Type Degrades", {
        use crate::element::Element;
        use crate::element::ElementGeometry;

        MINI_CHECK!(!Element::is_registered("NeverRegistered"));

        let mut proto = Element::from_mesh(unit_quad(), "mystery").to_proto();
        proto.element_type = "NeverRegistered".to_string();
        proto.element_data = b"whatever this package meant".to_vec();

        let loaded = Element::pb_loads_polymorphic(&prost::Message::encode_to_vec(&proto)).unwrap();

        MINI_CHECK!(loaded.name == "mystery");
        MINI_CHECK!(matches!(loaded.geometry(), ElementGeometry::Mesh(_)));
    })
}

pub fn run_element_features_round_trip() -> TestResult {
    MINI_TEST!("Features Round Trip", {
        use crate::element::Element;
        use crate::element::ElementFeature;
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;

        let mut e = Element::from_mesh(unit_quad(), "plate_0");
        e.set_insertion_vectors(vec![Vector::new(0.0, 0.0, 1.0), Vector::new(1.0, 0.0, 0.0)]);
        e.set_dimensions(Vector::new(120.0, 80.0, 12.5));
        e.add_feature(ElementFeature::new(
            "cut",
            2,
            vec![Polyline::new(vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 0.0, 0.0),
            ])],
            "notch",
        ));
        let feature_guid = e.features()[0].guid().to_string();

        let loaded = Element::pb_loads(&e.pb_dumps()).unwrap();

        MINI_CHECK!(loaded.insertion_vectors().len() == 2);
        MINI_CHECK!(loaded.insertion_vectors()[0] == Vector::new(0.0, 0.0, 1.0));
        MINI_CHECK!(loaded.dimensions().is_some());
        MINI_CHECK!(TOLERANCE.is_close(loaded.dimensions().as_ref().unwrap()[2], 12.5));
        MINI_CHECK!(loaded.features().len() == 1);
        MINI_CHECK!(loaded.features()[0].feature_type == "cut");
        MINI_CHECK!(loaded.features()[0].face_index == 2);
        MINI_CHECK!(loaded.features()[0].name == "notch");
        MINI_CHECK!(loaded.features()[0].outlines.len() == 1);
        MINI_CHECK!(loaded.features()[0].visible);
        MINI_CHECK!(loaded.features()[0].guid() == feature_guid);
    })
}

pub fn run_element_dimensions_are_nominal_not_measured() -> TestResult {
    MINI_TEST!("Dimensions Are Nominal Not Measured", {
        use crate::element::Element;
        use crate::Vector;

        let mut e = Element::from_mesh(unit_quad(), "plate");

        MINI_CHECK!(e.dimensions().is_none());

        e.set_dimensions(Vector::new(120.0, 80.0, 12.5));
        let measured = e.obb();

        MINI_CHECK!(TOLERANCE.is_close(e.dimensions().as_ref().unwrap()[0], 120.0));
        MINI_CHECK!(measured.half_size[0] < 1.0);
    })
}

pub fn run_element_registry_leaves_base_bytes_unchanged() -> TestResult {
    MINI_TEST!("Registry Leaves Base Bytes Unchanged", {
        use crate::element::Element;

        let e = Element::from_mesh(unit_quad(), "plain");
        let proto: crate::proto::Element = prost::Message::decode(e.pb_dumps().as_slice()).unwrap();

        MINI_CHECK!(proto.element_type.is_empty());
        MINI_CHECK!(proto.element_data.is_empty());
        MINI_CHECK!(e.element_type_name().is_empty());
    })
}

pub fn run_element_registry_json_round_trip() -> TestResult {
    MINI_TEST!("Registry Json Round Trip", {
        use crate::element::Element;

        Element::register_type("TestPlate", test_plate);

        let mut plate = Element::from_mesh(unit_quad(), "plate_json");
        plate.element_type = "TestPlate".to_string();
        plate.element_data = b"9.5,7,8".to_vec();
        let loaded = Element::file_json_loads_polymorphic(&plate.file_json_dumps());

        MINI_CHECK!(loaded.name == "plate_json_via_factory");
        MINI_CHECK!(loaded.guid() == plate.guid());
        MINI_CHECK!(loaded.element_type_name() == "TestPlate");
        MINI_CHECK!(loaded.element_data_dumps() == b"9.5,7,8");
    })
}

pub fn run_element_throwing_factory_degrades_to_base() -> TestResult {
    MINI_TEST!("Throwing Factory Degrades To Base", {
        use crate::element::Element;
        use crate::element::ElementGeometry;

        fn decline(_data: &[u8]) -> Option<Element> {
            None
        }

        Element::register_type("Exploding", decline);

        let mut proto = Element::from_mesh(unit_quad(), "victim").to_proto();
        proto.element_type = "Exploding".to_string();

        let loaded = Element::pb_loads_polymorphic(&prost::Message::encode_to_vec(&proto)).unwrap();

        MINI_CHECK!(loaded.name == "victim");
        MINI_CHECK!(matches!(loaded.geometry(), ElementGeometry::Mesh(_)));
    })
}

pub fn run_element_unknown_type_survives_resave() -> TestResult {
    MINI_TEST!("Unknown Type Survives Resave", {
        use crate::element::Element;

        let mut proto = Element::from_mesh(unit_quad(), "plate").to_proto();
        proto.element_type = "wood::Plate".to_string();
        proto.element_data = b"the package's own bytes".to_vec();
        let original = prost::Message::encode_to_vec(&proto);

        let loaded = Element::pb_loads(&original).unwrap();

        MINI_CHECK!(loaded.element_type_name() == "wood::Plate");
        MINI_CHECK!(loaded.element_data_dumps() == b"the package's own bytes");

        let resaved: crate::proto::Element =
            prost::Message::decode(loaded.pb_dumps().as_slice()).unwrap();

        MINI_CHECK!(resaved.element_type == "wood::Plate");
        MINI_CHECK!(resaved.element_data == b"the package's own bytes".to_vec());
    })
}

pub fn run_element_duplicate_keeps_every_field() -> TestResult {
    MINI_TEST!("Duplicate Keeps Every Field", {
        use crate::element::Element;
        use crate::element::ElementFeature;
        use crate::Vector;

        let mut e = Element::from_mesh(unit_quad(), "original");
        e.set_insertion_vectors(vec![Vector::new(0.0, 0.0, 1.0)]);
        e.set_dimensions(Vector::new(120.0, 80.0, 12.5));
        e.add_feature(ElementFeature::new("cut", 2, vec![], "notch"));

        let copy = e.duplicate();

        MINI_CHECK!(copy == e);
        MINI_CHECK!(copy.guid() != e.guid());
        MINI_CHECK!(copy.insertion_vectors().len() == 1);
        MINI_CHECK!(copy.dimensions().is_some());
        MINI_CHECK!(copy.features().len() == 1);
    })
}

pub fn run_element_equality_compares_carried_fields() -> TestResult {
    MINI_TEST!("Equality Compares Carried Fields", {
        use crate::element::Element;
        use crate::Vector;

        let a = Element::from_mesh(unit_quad(), "same");
        let mut b = Element::from_mesh(unit_quad(), "same");

        MINI_CHECK!(a == b);

        b.set_dimensions(Vector::new(1.0, 2.0, 3.0));

        MINI_CHECK!(a != b);
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// ElementFeature
// ═══════════════════════════════════════════════════════════════════════════

pub fn run_element_feature_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::element::ElementFeature;
        use crate::Point;
        use crate::Polyline;

        let outline = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let f = ElementFeature::new("cut", 2, vec![outline.clone()], "notch");

        MINI_CHECK!(f.feature_type == "cut");
        MINI_CHECK!(f.face_index == 2);
        MINI_CHECK!(f.name == "notch");
        MINI_CHECK!(f.outlines.len() == 1);
        MINI_CHECK!(f.visible);

        let same = ElementFeature::new("cut", 2, vec![outline.clone()], "notch");

        MINI_CHECK!(f == same);
        MINI_CHECK!(!(f != same));
        MINI_CHECK!(f.guid() != same.guid());

        let other = ElementFeature::new("drill", 2, vec![outline], "notch");

        MINI_CHECK!(f != other);

        MINI_CHECK!(f.str() == "ElementFeature(cut, face 2, 1 outline(s))");
        MINI_CHECK!(f.repr() == f.str());

        let empty = ElementFeature::default();

        MINI_CHECK!(empty.face_index == -1);
        MINI_CHECK!(empty.outlines.is_empty());
    })
}

pub fn run_element_feature_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::element::ElementFeature;
        use crate::Point;
        use crate::Polyline;

        let mut f = ElementFeature::new(
            "cut",
            2,
            vec![Polyline::new(vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 0.0, 0.0),
            ])],
            "notch",
        );
        f.visible = false;

        let feature_guid = f.guid().to_string();

        let fname = "serialization/test_element_feature.json";
        f.file_json_dump(fname);
        let loaded = ElementFeature::file_json_load(fname);

        MINI_CHECK!(loaded == f);
        MINI_CHECK!(loaded.outlines.len() == 1);
        MINI_CHECK!(!loaded.visible);
        MINI_CHECK!(loaded.guid() == feature_guid);
    })
}

pub fn run_element_feature_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::element::ElementFeature;
        use crate::Point;
        use crate::Polyline;

        let mut f = ElementFeature::new(
            "drill",
            5,
            vec![Polyline::new(vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 0.0, 0.0),
            ])],
            "hole",
        );
        f.visible = false;

        let feature_guid = f.guid().to_string();

        let path = "serialization/test_element_feature.bin";
        f.pb_dump(path);
        let loaded = ElementFeature::pb_load(path).unwrap();

        MINI_CHECK!(loaded == f);
        MINI_CHECK!(loaded.feature_type == "drill");
        MINI_CHECK!(loaded.face_index == 5);
        MINI_CHECK!(loaded.outlines.len() == 1);
        MINI_CHECK!(!loaded.visible);
        MINI_CHECK!(loaded.guid() == feature_guid);
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Registration
// ═══════════════════════════════════════════════════════════════════════════

REGISTER_MINI_TEST!(
    "Element",
    "Constructor",
    crate::element_test::run_element_constructor
);
REGISTER_MINI_TEST!("Element", "Place", crate::element_test::run_element_place);
REGISTER_MINI_TEST!(
    "Element",
    "Place Moves Features",
    crate::element_test::run_element_place_moves_features
);
REGISTER_MINI_TEST!(
    "Element",
    "Add Geometry Op",
    crate::element_test::run_element_add_geometry_op
);
REGISTER_MINI_TEST!("Element", "AABB", crate::element_test::run_element_aabb);
REGISTER_MINI_TEST!("Element", "OBB", crate::element_test::run_element_obb);
REGISTER_MINI_TEST!(
    "Element",
    "Session Geometry",
    crate::element_test::run_element_session_geometry
);
REGISTER_MINI_TEST!("Element", "Reset", crate::element_test::run_element_reset);
REGISTER_MINI_TEST!(
    "Element",
    "Compute Point",
    crate::element_test::run_element_compute_point
);
REGISTER_MINI_TEST!(
    "Element",
    "Brep Aabb",
    crate::element_test::run_element_brep_aabb
);
REGISTER_MINI_TEST!(
    "Element",
    "Json Roundtrip",
    crate::element_test::run_element_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Element",
    "Protobuf Roundtrip",
    crate::element_test::run_element_protobuf_roundtrip
);
REGISTER_MINI_TEST!(
    "Element",
    "Polylines",
    crate::element_test::run_element_polylines
);
REGISTER_MINI_TEST!(
    "Element",
    "Set Polylines Sticks",
    crate::element_test::run_element_set_polylines_sticks
);
REGISTER_MINI_TEST!(
    "Element",
    "Polylines Empty Without Mesh",
    crate::element_test::run_element_polylines_empty_without_mesh
);
REGISTER_MINI_TEST!(
    "Element",
    "Registry Round Trip",
    crate::element_test::run_element_registry_round_trip
);
REGISTER_MINI_TEST!(
    "Element",
    "Registry Unknown Type Degrades",
    crate::element_test::run_element_registry_unknown_type_degrades
);
REGISTER_MINI_TEST!(
    "Element",
    "Features Round Trip",
    crate::element_test::run_element_features_round_trip
);
REGISTER_MINI_TEST!(
    "Element",
    "Dimensions Are Nominal Not Measured",
    crate::element_test::run_element_dimensions_are_nominal_not_measured
);
REGISTER_MINI_TEST!(
    "Element",
    "Registry Leaves Base Bytes Unchanged",
    crate::element_test::run_element_registry_leaves_base_bytes_unchanged
);
REGISTER_MINI_TEST!(
    "Element",
    "Registry Json Round Trip",
    crate::element_test::run_element_registry_json_round_trip
);
REGISTER_MINI_TEST!(
    "Element",
    "Throwing Factory Degrades To Base",
    crate::element_test::run_element_throwing_factory_degrades_to_base
);
REGISTER_MINI_TEST!(
    "Element",
    "Unknown Type Survives Resave",
    crate::element_test::run_element_unknown_type_survives_resave
);
REGISTER_MINI_TEST!(
    "Element",
    "Duplicate Keeps Every Field",
    crate::element_test::run_element_duplicate_keeps_every_field
);
REGISTER_MINI_TEST!(
    "Element",
    "Equality Compares Carried Fields",
    crate::element_test::run_element_equality_compares_carried_fields
);
REGISTER_MINI_TEST!(
    "ElementFeature",
    "Constructor",
    crate::element_test::run_element_feature_constructor
);
REGISTER_MINI_TEST!(
    "ElementFeature",
    "Json Roundtrip",
    crate::element_test::run_element_feature_json_roundtrip
);
REGISTER_MINI_TEST!(
    "ElementFeature",
    "Protobuf Roundtrip",
    crate::element_test::run_element_feature_protobuf_roundtrip
);
