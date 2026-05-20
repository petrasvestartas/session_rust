use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

///////////////////////////////////////////////////////////////////////////////////////////
// ElementBeam
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
        MINI_CHECK!(bstr == "ElementBeam(beam1, 0.1, 0.2, 3)");
        MINI_CHECK!(brepr == format!("ElementBeam({}, beam1, 0.1, 0.2, 3)", guid));
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
    MINI_TEST!("AABB", {
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
            let min_x = mesh.vertex.values().map(|v| v.x).fold(f32::INFINITY, f32::min);
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
        b.file_json_dump(fname);
        let loaded = Element::file_json_load(fname);

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
// ElementBeam - Polylines/Planes/Edge Vectors/Axis
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
// Registration
///////////////////////////////////////////////////////////////////////////////////////////

REGISTER_MINI_TEST!("ElementBeam", "Constructor", crate::element_beam_test::run_beam_constructor);
REGISTER_MINI_TEST!("ElementBeam", "Setters", crate::element_beam_test::run_beam_setters);
REGISTER_MINI_TEST!("ElementBeam", "Center Line", crate::element_beam_test::run_beam_center_line);
REGISTER_MINI_TEST!("ElementBeam", "Extend", crate::element_beam_test::run_beam_extend);
REGISTER_MINI_TEST!("ElementBeam", "AABB", crate::element_beam_test::run_beam_aabb);
REGISTER_MINI_TEST!("ElementBeam", "Compute Point", crate::element_beam_test::run_beam_compute_point);
REGISTER_MINI_TEST!("ElementBeam", "Session Geometry", crate::element_beam_test::run_beam_session_geometry);
REGISTER_MINI_TEST!("ElementBeam", "Json Roundtrip", crate::element_beam_test::run_beam_json_roundtrip);
REGISTER_MINI_TEST!("ElementBeam", "Protobuf Roundtrip", crate::element_beam_test::run_beam_protobuf_roundtrip);
REGISTER_MINI_TEST!("ElementBeam", "Polylines", crate::element_beam_test::run_beam_polylines);
REGISTER_MINI_TEST!("ElementBeam", "Planes", crate::element_beam_test::run_beam_planes);
REGISTER_MINI_TEST!("ElementBeam", "Edge Vectors", crate::element_beam_test::run_beam_edge_vectors);
REGISTER_MINI_TEST!("ElementBeam", "Axis", crate::element_beam_test::run_beam_axis);
