use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

///////////////////////////////////////////////////////////////////////////////////////////
// Element
///////////////////////////////////////////////////////////////////////////////////////////

pub fn run_element_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Mesh;
        use crate::element::{Element, ElementGeometry};
        use crate::BRep;
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
        MINI_CHECK!(matches!(e.geometry(), ElementGeometry::Mesh(_)));
        MINI_CHECK!(estr == "Element(test_element, Mesh)");
        MINI_CHECK!(erepr == format!("Element({}, test_element, Mesh)", guid));
        MINI_CHECK!(ecopy == e && ecopy.guid() != e.guid());
        MINI_CHECK!(e == e2);
        MINI_CHECK!(e != e3);
    })
}

pub fn run_element_session_transformation() -> TestResult {
    MINI_TEST!("Session Transformation", {
        use crate::Mesh;
        use crate::Xform;
        use crate::element::Element;
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
        let xf = Xform::translation(10.0, 20.0, 30.0);
        e.session_transformation = xf.clone();

        MINI_CHECK!(e.is_dirty());
        MINI_CHECK!(e.session_transformation == xf);
    })
}

pub fn run_element_add_feature() -> TestResult {
    MINI_TEST!("Add Feature", {
        use crate::Mesh;
        use crate::element::Element;
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

        fn my_feature(geo: Mesh) -> Mesh { geo }
        e.add_feature(my_feature);

        MINI_CHECK!(e.is_dirty());
        MINI_CHECK!(e.features_count() == 1);
    })
}

pub fn run_element_aabb() -> TestResult {
    MINI_TEST!("Aabb", {
        use crate::Mesh;
        use crate::element::Element;
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
    })
}

pub fn run_element_obb() -> TestResult {
    MINI_TEST!("OBB", {
        use crate::Mesh;
        use crate::element::Element;
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
        use crate::Mesh;
        use crate::Xform;
        use crate::element::{Element, ElementGeometry};
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
        e.session_transformation = Xform::translation(10.0, 0.0, 0.0);
        let sg = e.session_geometry();

        MINI_CHECK!(matches!(&sg, ElementGeometry::Mesh(_)));
        if let ElementGeometry::Mesh(mesh) = &sg {
            let mut vkeys: Vec<usize> = mesh.vertex.keys().cloned().collect();
            vkeys.sort();
            let verts: Vec<_> = vkeys.iter().map(|k| mesh.vertex.get(k).unwrap()).collect();
            MINI_CHECK!(TOLERANCE.is_close(verts[0].x, 10.0));
            MINI_CHECK!(TOLERANCE.is_close(verts[1].x, 11.0));
        }
    })
}

pub fn run_element_reset() -> TestResult {
    MINI_TEST!("Reset", {
        use crate::Mesh;
        use crate::element::Element;
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
        MINI_CHECK!(e.cached_aabb_ref().is_none());
        MINI_CHECK!(e.cached_obb_ref().is_none());
        MINI_CHECK!(e.cached_collision_mesh_ref().is_none());
        MINI_CHECK!(e.cached_point_ref().is_none());
    })
}

pub fn run_element_compute_point() -> TestResult {
    MINI_TEST!("Compute Point", {
        use crate::Mesh;
        use crate::element::Element;
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
        use crate::element::Element;

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
        use crate::Mesh;
        use crate::Xform;
        use crate::element::{Element, ElementGeometry};
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
        let mut e = Element::from_mesh(m, "json_test");
        e.session_transformation = Xform::translation(1.0, 2.0, 3.0);

        let fname = "serialization/test_element.json";
        e.json_dump(fname);
        let loaded = Element::json_load(fname);

        MINI_CHECK!(loaded.name == "json_test");
        MINI_CHECK!(matches!(loaded.geometry(), ElementGeometry::Mesh(_)));
        if let ElementGeometry::Mesh(mesh) = loaded.geometry() {
            MINI_CHECK!(mesh.vertex.len() == 4);
        }
    })
}

pub fn run_element_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::BRep;
        use crate::Xform;
        use crate::element::{Element, ElementGeometry};

        let b = BRep::create_box(2.0, 3.0, 4.0);
        let mut e = Element::from_brep(b, "proto_test");
        e.session_transformation = Xform::translation(1.0, 2.0, 3.0);

        let path = "serialization/test_element.bin";
        e.pb_dump(path);
        let loaded = Element::pb_load(path).unwrap();

        MINI_CHECK!(loaded.name == "proto_test");
        MINI_CHECK!(matches!(loaded.geometry(), ElementGeometry::BRep(_)));
        if let ElementGeometry::BRep(brep) = loaded.geometry() {
            MINI_CHECK!(brep.m_faces.len() == 6);
            MINI_CHECK!(brep.m_vertices.len() == 8);
        }
    })
}

///////////////////////////////////////////////////////////////////////////////////////////
// Element - Polylines
///////////////////////////////////////////////////////////////////////////////////////////

pub fn run_element_polylines() -> TestResult {
    MINI_TEST!("Polylines", {
        use crate::Mesh;
        use crate::element::Element;
        use crate::Point;

        let m = Mesh::from_vertices_and_faces(
            vec![Point::new(0.0,0.0,0.0), Point::new(1.0,0.0,0.0), Point::new(1.0,1.0,0.0), Point::new(0.0,1.0,0.0)],
            vec![vec![0,1,2,3]],
        );
        let mut e = Element::from_mesh(m, "test_element");

        MINI_CHECK!(e.polylines().is_empty());
        MINI_CHECK!(e.planes().is_empty());
        MINI_CHECK!(e.edge_vectors().is_empty());
        MINI_CHECK!(e.axis().is_none());
    })
}

///////////////////////////////////////////////////////////////////////////////////////////
// Registration
///////////////////////////////////////////////////////////////////////////////////////////

REGISTER_MINI_TEST!("Element", "Constructor", crate::element_test::run_element_constructor);
REGISTER_MINI_TEST!("Element", "Session Transformation", crate::element_test::run_element_session_transformation);
REGISTER_MINI_TEST!("Element", "Add Feature", crate::element_test::run_element_add_feature);
REGISTER_MINI_TEST!("Element", "Aabb", crate::element_test::run_element_aabb);
REGISTER_MINI_TEST!("Element", "OBB", crate::element_test::run_element_obb);
REGISTER_MINI_TEST!("Element", "Session Geometry", crate::element_test::run_element_session_geometry);
REGISTER_MINI_TEST!("Element", "Reset", crate::element_test::run_element_reset);
REGISTER_MINI_TEST!("Element", "Compute Point", crate::element_test::run_element_compute_point);
REGISTER_MINI_TEST!("Element", "Brep Aabb", crate::element_test::run_element_brep_aabb);
REGISTER_MINI_TEST!("Element", "Json Roundtrip", crate::element_test::run_element_json_roundtrip);
REGISTER_MINI_TEST!("Element", "Protobuf Roundtrip", crate::element_test::run_element_protobuf_roundtrip);
REGISTER_MINI_TEST!("Element", "Polylines", crate::element_test::run_element_polylines);
