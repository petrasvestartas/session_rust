use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

///////////////////////////////////////////////////////////////////////////////////////////
// ElementColumn
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
        MINI_CHECK!(cstr == "ElementColumn(col1, 0.4, 0.4, 3)");
        MINI_CHECK!(crepr == format!("ElementColumn({}, col1, 0.4, 0.4, 3)", guid));
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
    MINI_TEST!("AABB", {
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

        let c = Element::column(0.4, 0.4, 3.0, "my_column");
        let c_xf = Xform::translation(10.0, 0.0, 0.0);
        let sg = c.session_geometry(&c_xf);

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

        let c = Element::column(0.5, 0.6, 4.0, "json_col");

        let fname = "serialization/test_column_element.json";
        c.file_json_dump(fname);
        let loaded = Element::file_json_load(fname);

        MINI_CHECK!(loaded.name == "json_col");
        MINI_CHECK!(TOLERANCE.is_close(loaded.width().unwrap(), 0.5));
        MINI_CHECK!(TOLERANCE.is_close(loaded.depth().unwrap(), 0.6));
        MINI_CHECK!(TOLERANCE.is_close(loaded.height().unwrap(), 4.0));
    })
}

pub fn run_column_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::element::Element;

        let c = Element::column(0.5, 0.6, 4.0, "proto_col");

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
// ElementColumn - Polylines/Planes/Edge Vectors/Axis
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
// Registration
///////////////////////////////////////////////////////////////////////////////////////////

REGISTER_MINI_TEST!("ElementColumn", "Constructor", crate::element_column_test::run_column_constructor);
REGISTER_MINI_TEST!("ElementColumn", "Setters", crate::element_column_test::run_column_setters);
REGISTER_MINI_TEST!("ElementColumn", "Center Line", crate::element_column_test::run_column_center_line);
REGISTER_MINI_TEST!("ElementColumn", "Extend", crate::element_column_test::run_column_extend);
REGISTER_MINI_TEST!("ElementColumn", "AABB", crate::element_column_test::run_column_aabb);
REGISTER_MINI_TEST!("ElementColumn", "Compute Point", crate::element_column_test::run_column_compute_point);
REGISTER_MINI_TEST!("ElementColumn", "Session Geometry", crate::element_column_test::run_column_session_geometry);
REGISTER_MINI_TEST!("ElementColumn", "Json Roundtrip", crate::element_column_test::run_column_json_roundtrip);
REGISTER_MINI_TEST!("ElementColumn", "Protobuf Roundtrip", crate::element_column_test::run_column_protobuf_roundtrip);
REGISTER_MINI_TEST!("ElementColumn", "Polylines", crate::element_column_test::run_column_polylines);
REGISTER_MINI_TEST!("ElementColumn", "Planes", crate::element_column_test::run_column_planes);
REGISTER_MINI_TEST!("ElementColumn", "Edge Vectors", crate::element_column_test::run_column_edge_vectors);
REGISTER_MINI_TEST!("ElementColumn", "Axis", crate::element_column_test::run_column_axis);
