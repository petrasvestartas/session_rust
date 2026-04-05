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
// ColumnElement
///////////////////////////////////////////////////////////////////////////////////////////

pub fn run_column_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::element::{Element, ElementGeometry};

        let c = Element::column(0.4, 0.4, 3.0, "col1");

        let name = &c.name;
        let guid = c.guid().to_string();
        let cstr = c.str();
        let crepr = c.repr();

        let ccopy = c.duplicate();
        let c2 = Element::column(0.4, 0.4, 3.0, "col1");
        let c3 = Element::column(0.5, 0.4, 3.0, "col1");

        MINI_CHECK!(name == "col1");
        MINI_CHECK!(!guid.is_empty());
        MINI_CHECK!(matches!(c.geometry(), ElementGeometry::Mesh(_)));
        MINI_CHECK!(c.width() == Some(0.4));
        MINI_CHECK!(c.depth() == Some(0.4));
        MINI_CHECK!(c.height() == Some(3.0));
        MINI_CHECK!(cstr == "ColumnElement(col1, 0.4, 0.4, 3)");
        MINI_CHECK!(crepr == format!("ColumnElement({}, col1, 0.4, 0.4, 3)", guid));
        MINI_CHECK!(ccopy == c && ccopy.guid() != c.guid());
        MINI_CHECK!(c == c2);
        MINI_CHECK!(c != c3);
    })
}

pub fn run_column_setters() -> TestResult {
    MINI_TEST!("Setters", {
        use crate::element::Element;

        let mut c = Element::column(0.4, 0.4, 3.0, "my_column");
        c.set_width(0.5);
        c.set_depth(0.6);
        c.set_height(4.0);

        MINI_CHECK!(c.width() == Some(0.5));
        MINI_CHECK!(c.depth() == Some(0.6));
        MINI_CHECK!(c.height() == Some(4.0));
        MINI_CHECK!(c.has_geometry());
    })
}

pub fn run_column_center_line() -> TestResult {
    MINI_TEST!("Center Line", {
        use crate::element::Element;

        let c = Element::column(0.4, 0.4, 5.0, "my_column");
        let cl = c.center_line().unwrap();

        MINI_CHECK!(TOLERANCE.is_close(cl.start()[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(cl.end()[2], 5.0));
    })
}

pub fn run_column_extend() -> TestResult {
    MINI_TEST!("Extend", {
        use crate::element::Element;

        let mut c = Element::column(0.4, 0.4, 3.0, "my_column");
        c.extend(0.5);

        MINI_CHECK!(TOLERANCE.is_close(c.height().unwrap(), 4.0));
    })
}

pub fn run_column_aabb() -> TestResult {
    MINI_TEST!("Aabb", {
        use crate::element::Element;

        let mut c = Element::column(0.4, 0.4, 3.0, "my_column");
        let aabb = c.aabb();

        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[0], 0.2));
        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[1], 0.2));
        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[2], 1.5));
    })
}

pub fn run_column_compute_point() -> TestResult {
    MINI_TEST!("Compute Point", {
        use crate::element::Element;

        let mut c = Element::column(0.4, 0.4, 3.0, "my_column");
        let pt = c.point();

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 1.5));
    })
}

pub fn run_column_session_geometry() -> TestResult {
    MINI_TEST!("Session Geometry", {
        use crate::element::{Element, ElementGeometry};
        use crate::Xform;

        let mut c = Element::column(0.4, 0.4, 3.0, "my_column");
        c.session_transformation = Xform::translation(10.0, 0.0, 0.0);
        let sg = c.session_geometry();

        MINI_CHECK!(matches!(&sg, ElementGeometry::Mesh(_)));
        if let ElementGeometry::Mesh(mesh) = &sg {
            let min_x = mesh.vertex.values().map(|v| v.x).fold(f64::INFINITY, f64::min);
            MINI_CHECK!(min_x > 9.0);
        }
    })
}

pub fn run_column_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::element::Element;
        use crate::Xform;

        let mut c = Element::column(0.5, 0.6, 4.0, "json_col");
        c.session_transformation = Xform::translation(1.0, 2.0, 3.0);

        let fname = "serialization/test_column_element.json";
        c.json_dump(fname);
        let loaded = Element::json_load(fname);

        MINI_CHECK!(loaded.name == "json_col");
        MINI_CHECK!(TOLERANCE.is_close(loaded.width().unwrap(), 0.5));
        MINI_CHECK!(TOLERANCE.is_close(loaded.depth().unwrap(), 0.6));
        MINI_CHECK!(TOLERANCE.is_close(loaded.height().unwrap(), 4.0));
    })
}

pub fn run_column_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::element::Element;
        use crate::Xform;

        let mut c = Element::column(0.5, 0.6, 4.0, "proto_col");
        c.session_transformation = Xform::translation(1.0, 2.0, 3.0);

        let path = "serialization/test_column_element.bin";
        c.pb_dump(path);
        let loaded = Element::pb_load(path).unwrap();

        MINI_CHECK!(loaded.name == "proto_col");
        MINI_CHECK!(TOLERANCE.is_close(loaded.width().unwrap(), 0.5));
        MINI_CHECK!(TOLERANCE.is_close(loaded.depth().unwrap(), 0.6));
        MINI_CHECK!(TOLERANCE.is_close(loaded.height().unwrap(), 4.0));
    })
}

///////////////////////////////////////////////////////////////////////////////////////////
// BeamElement
///////////////////////////////////////////////////////////////////////////////////////////

pub fn run_beam_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::element::{Element, ElementGeometry};

        let b = Element::beam(0.1, 0.2, 3.0, "beam1");

        let name = &b.name;
        let guid = b.guid().to_string();
        let bstr = b.str();
        let brepr = b.repr();

        let bcopy = b.duplicate();
        let b2 = Element::beam(0.1, 0.2, 3.0, "beam1");
        let b3 = Element::beam(0.1, 0.2, 5.0, "beam1");

        MINI_CHECK!(name == "beam1");
        MINI_CHECK!(!guid.is_empty());
        MINI_CHECK!(matches!(b.geometry(), ElementGeometry::Mesh(_)));
        MINI_CHECK!(b.width() == Some(0.1));
        MINI_CHECK!(b.depth() == Some(0.2));
        MINI_CHECK!(b.length() == Some(3.0));
        MINI_CHECK!(bstr == "BeamElement(beam1, 0.1, 0.2, 3)");
        MINI_CHECK!(brepr == format!("BeamElement({}, beam1, 0.1, 0.2, 3)", guid));
        MINI_CHECK!(bcopy == b && bcopy.guid() != b.guid());
        MINI_CHECK!(b == b2);
        MINI_CHECK!(b != b3);
    })
}

pub fn run_beam_setters() -> TestResult {
    MINI_TEST!("Setters", {
        use crate::element::Element;

        let mut b = Element::beam(0.1, 0.2, 3.0, "my_beam");
        b.set_width(0.15);
        b.set_depth(0.3);
        b.set_length(5.0);

        MINI_CHECK!(b.width() == Some(0.15));
        MINI_CHECK!(b.depth() == Some(0.3));
        MINI_CHECK!(b.length() == Some(5.0));
        MINI_CHECK!(b.has_geometry());
    })
}

pub fn run_beam_center_line() -> TestResult {
    MINI_TEST!("Center Line", {
        use crate::element::Element;

        let b = Element::beam(0.1, 0.2, 5.0, "my_beam");
        let cl = b.center_line().unwrap();

        MINI_CHECK!(TOLERANCE.is_close(cl.start()[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(cl.end()[2], 5.0));
    })
}

pub fn run_beam_extend() -> TestResult {
    MINI_TEST!("Extend", {
        use crate::element::Element;

        let mut b = Element::beam(0.1, 0.2, 3.0, "my_beam");
        b.extend(0.5);

        MINI_CHECK!(TOLERANCE.is_close(b.length().unwrap(), 4.0));
    })
}

pub fn run_beam_aabb() -> TestResult {
    MINI_TEST!("Aabb", {
        use crate::element::Element;

        let mut b = Element::beam(0.1, 0.2, 3.0, "my_beam");
        let aabb = b.aabb();

        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[0], 0.05));
        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[1], 0.1));
        MINI_CHECK!(TOLERANCE.is_close(aabb.half_size[2], 1.5));
    })
}

pub fn run_beam_compute_point() -> TestResult {
    MINI_TEST!("Compute Point", {
        use crate::element::Element;

        let mut b = Element::beam(0.1, 0.2, 3.0, "my_beam");
        let pt = b.point();

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 1.5));
    })
}

pub fn run_beam_session_geometry() -> TestResult {
    MINI_TEST!("Session Geometry", {
        use crate::element::{Element, ElementGeometry};
        use crate::Xform;

        let mut b = Element::beam(0.1, 0.2, 3.0, "my_beam");
        b.session_transformation = Xform::translation(10.0, 0.0, 0.0);
        let sg = b.session_geometry();

        MINI_CHECK!(matches!(&sg, ElementGeometry::Mesh(_)));
        if let ElementGeometry::Mesh(mesh) = &sg {
            let min_x = mesh.vertex.values().map(|v| v.x).fold(f64::INFINITY, f64::min);
            MINI_CHECK!(min_x > 9.0);
        }
    })
}

pub fn run_beam_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::element::Element;
        use crate::Xform;

        let mut b = Element::beam(0.15, 0.3, 5.0, "json_beam");
        b.session_transformation = Xform::translation(1.0, 2.0, 3.0);

        let fname = "serialization/test_beam_element.json";
        b.json_dump(fname);
        let loaded = Element::json_load(fname);

        MINI_CHECK!(loaded.name == "json_beam");
        MINI_CHECK!(TOLERANCE.is_close(loaded.width().unwrap(), 0.15));
        MINI_CHECK!(TOLERANCE.is_close(loaded.depth().unwrap(), 0.3));
        MINI_CHECK!(TOLERANCE.is_close(loaded.length().unwrap(), 5.0));
    })
}

pub fn run_beam_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::element::Element;
        use crate::Xform;

        let mut b = Element::beam(0.15, 0.3, 5.0, "proto_beam");
        b.session_transformation = Xform::translation(1.0, 2.0, 3.0);

        let path = "serialization/test_beam_element.bin";
        b.pb_dump(path);
        let loaded = Element::pb_load(path).unwrap();

        MINI_CHECK!(loaded.name == "proto_beam");
        MINI_CHECK!(TOLERANCE.is_close(loaded.width().unwrap(), 0.15));
        MINI_CHECK!(TOLERANCE.is_close(loaded.depth().unwrap(), 0.3));
        MINI_CHECK!(TOLERANCE.is_close(loaded.length().unwrap(), 5.0));
    })
}

///////////////////////////////////////////////////////////////////////////////////////////
// PlateElement
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
        MINI_CHECK!(pstr == "PlateElement(plate1, 4 pts, 0.2)");
        MINI_CHECK!(prepr == format!("PlateElement({}, plate1, 4 pts, 0.2)", guid));
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
    MINI_TEST!("Aabb", {
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
        use crate::Xform;

        let polygon = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
        ];
        let mut p = Element::plate(polygon, 0.3, "json_plate");
        p.session_transformation = Xform::translation(1.0, 2.0, 3.0);

        let fname = "serialization/test_plate_element.json";
        p.json_dump(fname);
        let loaded = Element::json_load(fname);

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
        use crate::Xform;

        let polygon = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
        ];
        let mut p = Element::plate(polygon, 0.3, "proto_plate");
        p.session_transformation = Xform::translation(1.0, 2.0, 3.0);

        let path = "serialization/test_plate_element.bin";
        p.pb_dump(path);
        let loaded = Element::pb_load(path).unwrap();

        MINI_CHECK!(loaded.name == "proto_plate");
        MINI_CHECK!(TOLERANCE.is_close(loaded.thickness().unwrap(), 0.3));
        MINI_CHECK!(loaded.polygon().unwrap().len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(loaded.polygon().unwrap()[1][0], 2.0));
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
// BeamElement - Polylines/Planes/Edge Vectors/Axis
///////////////////////////////////////////////////////////////////////////////////////////

pub fn run_beam_polylines() -> TestResult {
    MINI_TEST!("Polylines", {
        use crate::element::Element;
        let mut b = Element::beam(0.1, 0.2, 3.0, "my_beam");
        let pls = b.polylines();
        MINI_CHECK!(pls.len() == 6);
        for pl in &pls { MINI_CHECK!(pl.point_count() == 5); }
    })
}

pub fn run_beam_planes() -> TestResult {
    MINI_TEST!("Planes", {
        use crate::element::Element;
        let mut b = Element::beam(0.1, 0.2, 3.0, "my_beam");
        let pls = b.planes();
        MINI_CHECK!(pls.len() == 6);
        MINI_CHECK!(TOLERANCE.is_close(pls[0].z_axis()[2], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(pls[1].z_axis()[2], 1.0));
    })
}

pub fn run_beam_edge_vectors() -> TestResult {
    MINI_TEST!("Edge Vectors", {
        use crate::element::Element;
        let mut b = Element::beam(0.1, 0.2, 3.0, "my_beam");
        let evs = b.edge_vectors();
        MINI_CHECK!(evs.len() == 12);
    })
}

pub fn run_beam_axis() -> TestResult {
    MINI_TEST!("Axis", {
        use crate::element::Element;
        let mut b = Element::beam(0.1, 0.2, 5.0, "my_beam");
        let ax = b.axis().unwrap();
        MINI_CHECK!(TOLERANCE.is_close(ax.start()[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(ax.end()[2], 5.0));
    })
}

///////////////////////////////////////////////////////////////////////////////////////////
// ColumnElement - Polylines/Planes/Edge Vectors/Axis
///////////////////////////////////////////////////////////////////////////////////////////

pub fn run_column_polylines() -> TestResult {
    MINI_TEST!("Polylines", {
        use crate::element::Element;
        let mut c = Element::column(0.4, 0.4, 3.0, "my_column");
        let pls = c.polylines();
        MINI_CHECK!(pls.len() == 6);
        for pl in &pls { MINI_CHECK!(pl.point_count() == 5); }
    })
}

pub fn run_column_planes() -> TestResult {
    MINI_TEST!("Planes", {
        use crate::element::Element;
        let mut c = Element::column(0.4, 0.4, 3.0, "my_column");
        let pls = c.planes();
        MINI_CHECK!(pls.len() == 6);
        MINI_CHECK!(TOLERANCE.is_close(pls[0].z_axis()[2], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(pls[1].z_axis()[2], 1.0));
    })
}

pub fn run_column_edge_vectors() -> TestResult {
    MINI_TEST!("Edge Vectors", {
        use crate::element::Element;
        let mut c = Element::column(0.4, 0.4, 3.0, "my_column");
        let evs = c.edge_vectors();
        MINI_CHECK!(evs.len() == 12);
    })
}

pub fn run_column_axis() -> TestResult {
    MINI_TEST!("Axis", {
        use crate::element::Element;
        let mut c = Element::column(0.4, 0.4, 5.0, "my_column");
        let ax = c.axis().unwrap();
        MINI_CHECK!(TOLERANCE.is_close(ax.start()[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(ax.end()[2], 5.0));
    })
}

///////////////////////////////////////////////////////////////////////////////////////////
// PlateElement - Polylines/Planes/Edge Vectors/Axis/Joinery
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
        p.json_dump(fname);
        let loaded = Element::json_load(fname);

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

REGISTER_MINI_TEST!("ColumnElement", "Constructor", crate::element_test::run_column_constructor);
REGISTER_MINI_TEST!("ColumnElement", "Setters", crate::element_test::run_column_setters);
REGISTER_MINI_TEST!("ColumnElement", "Center Line", crate::element_test::run_column_center_line);
REGISTER_MINI_TEST!("ColumnElement", "Extend", crate::element_test::run_column_extend);
REGISTER_MINI_TEST!("ColumnElement", "Aabb", crate::element_test::run_column_aabb);
REGISTER_MINI_TEST!("ColumnElement", "Compute Point", crate::element_test::run_column_compute_point);
REGISTER_MINI_TEST!("ColumnElement", "Session Geometry", crate::element_test::run_column_session_geometry);
REGISTER_MINI_TEST!("ColumnElement", "Json Roundtrip", crate::element_test::run_column_json_roundtrip);
REGISTER_MINI_TEST!("ColumnElement", "Protobuf Roundtrip", crate::element_test::run_column_protobuf_roundtrip);
REGISTER_MINI_TEST!("ColumnElement", "Polylines", crate::element_test::run_column_polylines);
REGISTER_MINI_TEST!("ColumnElement", "Planes", crate::element_test::run_column_planes);
REGISTER_MINI_TEST!("ColumnElement", "Edge Vectors", crate::element_test::run_column_edge_vectors);
REGISTER_MINI_TEST!("ColumnElement", "Axis", crate::element_test::run_column_axis);

REGISTER_MINI_TEST!("BeamElement", "Constructor", crate::element_test::run_beam_constructor);
REGISTER_MINI_TEST!("BeamElement", "Setters", crate::element_test::run_beam_setters);
REGISTER_MINI_TEST!("BeamElement", "Center Line", crate::element_test::run_beam_center_line);
REGISTER_MINI_TEST!("BeamElement", "Extend", crate::element_test::run_beam_extend);
REGISTER_MINI_TEST!("BeamElement", "Aabb", crate::element_test::run_beam_aabb);
REGISTER_MINI_TEST!("BeamElement", "Compute Point", crate::element_test::run_beam_compute_point);
REGISTER_MINI_TEST!("BeamElement", "Session Geometry", crate::element_test::run_beam_session_geometry);
REGISTER_MINI_TEST!("BeamElement", "Json Roundtrip", crate::element_test::run_beam_json_roundtrip);
REGISTER_MINI_TEST!("BeamElement", "Protobuf Roundtrip", crate::element_test::run_beam_protobuf_roundtrip);
REGISTER_MINI_TEST!("BeamElement", "Polylines", crate::element_test::run_beam_polylines);
REGISTER_MINI_TEST!("BeamElement", "Planes", crate::element_test::run_beam_planes);
REGISTER_MINI_TEST!("BeamElement", "Edge Vectors", crate::element_test::run_beam_edge_vectors);
REGISTER_MINI_TEST!("BeamElement", "Axis", crate::element_test::run_beam_axis);

REGISTER_MINI_TEST!("PlateElement", "Constructor", crate::element_test::run_plate_constructor);
REGISTER_MINI_TEST!("PlateElement", "Default Polygon", crate::element_test::run_plate_default_polygon);
REGISTER_MINI_TEST!("PlateElement", "Setters", crate::element_test::run_plate_setters);
REGISTER_MINI_TEST!("PlateElement", "Mesh Topology", crate::element_test::run_plate_mesh_topology);
REGISTER_MINI_TEST!("PlateElement", "Aabb", crate::element_test::run_plate_aabb);
REGISTER_MINI_TEST!("PlateElement", "Compute Point", crate::element_test::run_plate_compute_point);
REGISTER_MINI_TEST!("PlateElement", "Triangle Polygon", crate::element_test::run_plate_triangle_polygon);
REGISTER_MINI_TEST!("PlateElement", "Json Roundtrip", crate::element_test::run_plate_json_roundtrip);
REGISTER_MINI_TEST!("PlateElement", "Protobuf Roundtrip", crate::element_test::run_plate_protobuf_roundtrip);
REGISTER_MINI_TEST!("PlateElement", "Polylines", crate::element_test::run_plate_polylines);
REGISTER_MINI_TEST!("PlateElement", "Planes", crate::element_test::run_plate_planes);
REGISTER_MINI_TEST!("PlateElement", "Edge Vectors", crate::element_test::run_plate_edge_vectors);
REGISTER_MINI_TEST!("PlateElement", "Axis", crate::element_test::run_plate_axis);
REGISTER_MINI_TEST!("PlateElement", "Joint Types", crate::element_test::run_plate_joint_types);
REGISTER_MINI_TEST!("PlateElement", "J Mf", crate::element_test::run_plate_j_mf);
REGISTER_MINI_TEST!("PlateElement", "Key", crate::element_test::run_plate_key);
REGISTER_MINI_TEST!("PlateElement", "Component Plane", crate::element_test::run_plate_component_plane);
REGISTER_MINI_TEST!("PlateElement", "Json Roundtrip Joinery", crate::element_test::run_plate_json_roundtrip_joinery);
REGISTER_MINI_TEST!("PlateElement", "Protobuf Roundtrip Joinery", crate::element_test::run_plate_protobuf_roundtrip_joinery);
