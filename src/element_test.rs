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

pub fn run_element_place() -> TestResult {
    MINI_TEST!("Place", {
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
        let xf = Xform::translation(10.0, 20.0, 30.0);
        e.place(&xf);

        MINI_CHECK!(e.is_dirty());
        if let ElementGeometry::Mesh(mesh) = e.geometry() {
            let min_x = mesh.vertex.values().map(|v| v.x).fold(f64::INFINITY, f64::min);
            MINI_CHECK!(min_x > 9.0);
        }
    })
}

pub fn run_element_add_feature() -> TestResult {
    MINI_TEST!("Add Geometry Op", {
        use crate::Mesh;
        use crate::BRep;
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

        fn my_feature(geo: Mesh) -> Mesh { geo }
        e.add_geometry_op(my_feature);

        // Features are Mesh -> Mesh, so BRep geometry passes through untouched
        fn empty_mesh(_geo: Mesh) -> Mesh { Mesh::new() }
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
        use crate::Mesh;
        use crate::Element;
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
        use crate::Element;
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
        let e = Element::from_mesh(m, "my_element");
        let e_xf = Xform::translation(10.0, 0.0, 0.0);
        let sg = e.session_geometry(&e_xf);

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
        use crate::Element;
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
        use crate::Element;
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
        use crate::Mesh;
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
        use crate::BRep;
        use crate::element::{Element, ElementGeometry};

        let b = BRep::create_box(2.0, 3.0, 4.0);
        let e = Element::from_brep(b, "proto_test");

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
        use crate::Element;
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
// Element - polymorphic registry
//
// Rust has no inheritance, so unlike C++/Python there is no factory returning a derived
// element. What Rust guarantees instead is that it never DESTROYS a derived element: the
// type name and payload a downstream package wrote survive a load/save untouched, so a Rust
// tool can round-trip a file whose domain type it has never heard of.
///////////////////////////////////////////////////////////////////////////////////////////

fn unit_quad() -> crate::Mesh {
    use crate::Point;
    crate::Mesh::from_vertices_and_faces(
        vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
             Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0)],
        vec![vec![0, 1, 2, 3]],
    )
}

pub fn run_element_registry_round_trip() -> TestResult {
    MINI_TEST!("RegistryRoundTrip", {
        use crate::element::{Element, ElementGeometry};

        // Stand-in for what a domain package writes: a type name the kernel does not know
        // and a payload it never parses.
        let mut plate = Element::from_mesh(unit_quad(), "plate_0");
        plate.element_type = "TestPlate".to_string();
        plate.element_data = b"12.5,30,11,20".to_vec();

        let loaded = Element::pb_loads(&plate.pb_dumps()).unwrap();

        // Identity, base state and the domain payload all survived.
        MINI_CHECK!(loaded.guid() == plate.guid());
        MINI_CHECK!(loaded.name == "plate_0");
        MINI_CHECK!(matches!(loaded.geometry(), ElementGeometry::Mesh(_)));
        MINI_CHECK!(loaded.element_type == "TestPlate");
        MINI_CHECK!(loaded.element_data == b"12.5,30,11,20".to_vec());
    })
}

pub fn run_element_registry_unknown_type_degrades() -> TestResult {
    MINI_TEST!("RegistryUnknownTypeDegrades", {
        use crate::element::{Element, ElementGeometry};

        // A file written by a package this binary does not have. It must still load, keeping
        // its geometry - and must carry the payload back out again unchanged, so saving does
        // not quietly strip data this build could not interpret.
        let mut mystery = Element::from_mesh(unit_quad(), "mystery");
        mystery.element_type = "NeverRegistered".to_string();
        mystery.element_data = b"whatever this package meant".to_vec();

        let loaded = Element::pb_loads(&mystery.pb_dumps()).unwrap();
        MINI_CHECK!(loaded.name == "mystery");
        MINI_CHECK!(matches!(loaded.geometry(), ElementGeometry::Mesh(_)));

        let again = Element::pb_loads(&loaded.pb_dumps()).unwrap();
        MINI_CHECK!(again.element_type == "NeverRegistered");
        MINI_CHECK!(again.element_data == b"whatever this package meant".to_vec());
    })
}

pub fn run_element_registry_leaves_base_bytes_unchanged() -> TestResult {
    MINI_TEST!("RegistryLeavesBaseBytesUnchanged", {
        use crate::element::Element;

        // proto3 omits empty scalars, so adding element_type/element_data must not have
        // changed one byte of a plain Element - the cross-language golden files depend on it.
        let e = Element::from_mesh(unit_quad(), "plain");
        let proto = e.to_proto();

        MINI_CHECK!(proto.element_type.is_empty());
        MINI_CHECK!(proto.element_data.is_empty());
    })
}

pub fn run_element_features_round_trip() -> TestResult {
    MINI_TEST!("FeaturesRoundTrip", {
        use crate::element::{Element, ElementFeature};
        use crate::{Point, Polyline, Vector};

        // insertion_vectors / dimensions / features are the general shape that replaced the
        // per-domain arrays (joint_types and friends) that used to sit on this message. All
        // three must survive a round trip or a domain reinvents its own fields.
        let mut e = Element::from_mesh(unit_quad(), "plate_0");
        e.insertion_vectors = vec![Vector::new(0.0, 0.0, 1.0), Vector::new(1.0, 0.0, 0.0)];
        e.dimensions = Some(Vector::new(120.0, 80.0, 12.5));
        e.features.push(ElementFeature::new("cut", 2, vec![Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 0.0, 0.0)])], "notch"));
        let feature_guid = e.features[0].guid().to_string();

        let loaded = Element::pb_loads(&e.pb_dumps()).unwrap();

        MINI_CHECK!(loaded.insertion_vectors.len() == 2);
        MINI_CHECK!(loaded.dimensions.is_some());
        // z is the thickness - the whole reason this is a vector rather than one f64.
        MINI_CHECK!((loaded.dimensions.as_ref().unwrap()[2] - 12.5).abs() < 1e-9);
        MINI_CHECK!(loaded.features.len() == 1);
        MINI_CHECK!(loaded.features[0].feature_type == "cut");
        MINI_CHECK!(loaded.features[0].face_index == 2);
        MINI_CHECK!(loaded.features[0].name == "notch");
        MINI_CHECK!(loaded.features[0].outlines.len() == 1);
        // The guid is the feature's handle: a package that wrote a joint has to find it again, and
        // the index in `features` moves the moment an earlier feature is removed.
        MINI_CHECK!(loaded.features[0].guid() == feature_guid);
    })
}

pub fn run_element_dimensions_are_nominal_not_measured() -> TestResult {
    MINI_TEST!("DimensionsAreNominalNotMeasured", {
        use crate::element::Element;
        use crate::Vector;

        // dimensions is AUTHORED intent; obb() MEASURES what exists. They are allowed to
        // disagree, and this pins that they are genuinely independent.
        let mut e = Element::from_mesh(unit_quad(), "plate");
        MINI_CHECK!(e.dimensions.is_none());              // never authored

        e.dimensions = Some(Vector::new(120.0, 80.0, 12.5)); // nothing like the unit quad
        let measured = e.obb();

        MINI_CHECK!((e.dimensions.as_ref().unwrap()[0] - 120.0).abs() < 1e-9);
        MINI_CHECK!(measured.half_size[0] < 1.0);         // the geometry is still a unit quad
    })
}

pub fn run_element_unknown_type_survives_resave() -> TestResult {
    MINI_TEST!("UnknownTypeSurvivesResave", {
        use crate::element::Element;

        // The whole point of element_type/element_data: a viewer WITHOUT the wood package opens
        // a wood file, edits something else, and saves. If the kernel does not carry these two
        // through, that save silently destroys the payload - the geometry still looks right, so
        // nothing announces the loss. This is the test that would have caught it.
        let mut plate = Element::from_mesh(unit_quad(), "plate");
        plate.element_type = "wood::Plate".to_string();
        plate.element_data = b"the package's own bytes".to_vec();
        let original = plate.pb_dumps();

        let loaded = Element::pb_loads(&original).unwrap();
        MINI_CHECK!(loaded.element_type_name() == "wood::Plate");
        MINI_CHECK!(loaded.element_data_dumps() == b"the package's own bytes");

        let resaved = Element::pb_loads(&loaded.pb_dumps()).unwrap();
        MINI_CHECK!(resaved.element_type == "wood::Plate");
        MINI_CHECK!(resaved.element_data == b"the package's own bytes".to_vec());
    })
}

pub fn run_element_duplicate_keeps_every_field() -> TestResult {
    MINI_TEST!("DuplicateKeepsEveryField", {
        use crate::element::{Element, ElementFeature};
        use crate::Vector;

        // A copy that drops fields is the same silent data loss as a save that drops them, and
        // a duplicate is what an assembly does to place the same part twice.
        let mut e = Element::from_mesh(unit_quad(), "original");
        e.insertion_vectors = vec![Vector::new(0.0, 0.0, 1.0)];
        e.dimensions = Some(Vector::new(120.0, 80.0, 12.5));
        e.features = vec![ElementFeature::new("cut", 2, vec![], "notch")];

        let copy = e.duplicate();

        MINI_CHECK!(copy == e);                   // every carried field compares equal
        MINI_CHECK!(copy.guid() != e.guid());     // but it is a different object
        MINI_CHECK!(copy.insertion_vectors.len() == 1);
        MINI_CHECK!(copy.dimensions.is_some());
        MINI_CHECK!(copy.features.len() == 1);
    })
}

pub fn run_element_equality_compares_carried_fields() -> TestResult {
    MINI_TEST!("EqualityComparesCarriedFields", {
        use crate::element::Element;
        use crate::Vector;

        // Equality that looks at name and geometry only makes every round-trip test above
        // vacuous: it would pass while the loader dropped all five of the other fields.
        let a = Element::from_mesh(unit_quad(), "same");
        let mut b = Element::from_mesh(unit_quad(), "same");
        MINI_CHECK!(a == b);

        b.dimensions = Some(Vector::new(1.0, 2.0, 3.0));
        MINI_CHECK!(a != b);
    })
}

///////////////////////////////////////////////////////////////////////////////////////////
// ElementFeature
///////////////////////////////////////////////////////////////////////////////////////////

pub fn run_element_feature_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::element::ElementFeature;
        use crate::{Point, Polyline};

        let outline = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
                                         Point::new(1.0, 1.0, 0.0), Point::new(0.0, 0.0, 0.0)]);
        let f = ElementFeature::new("cut", 2, vec![outline.clone()], "notch");

        MINI_CHECK!(f.feature_type == "cut");
        MINI_CHECK!(f.face_index == 2);
        MINI_CHECK!(f.name == "notch");
        MINI_CHECK!(f.outlines.len() == 1);

        let same = ElementFeature::new("cut", 2, vec![outline.clone()], "notch");
        MINI_CHECK!(f == same);
        MINI_CHECK!(!(f != same));
        // Data equality, not identity - the two guids differ and the features are still equal.
        MINI_CHECK!(f.guid() != same.guid());

        let other = ElementFeature::new("drill", 2, vec![outline], "notch");
        MINI_CHECK!(f != other);

        MINI_CHECK!(f.str() == "ElementFeature(cut, face 2, 1 outline(s))");
        MINI_CHECK!(format!("{}", f) == f.str());

        let empty = ElementFeature::default();
        MINI_CHECK!(empty.face_index == 0);
        MINI_CHECK!(empty.outlines.is_empty());
    })
}

pub fn run_element_feature_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::element::ElementFeature;
        use crate::{Point, Polyline};

        let f = ElementFeature::new("cut", 2,
            vec![Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
                                    Point::new(1.0, 1.0, 0.0), Point::new(0.0, 0.0, 0.0)])],
            "notch");
        let feature_guid = f.guid().to_string();

        let fname = "serialization/test_element_feature.json";
        f.file_json_dump(fname);
        let loaded = ElementFeature::file_json_load(fname);

        MINI_CHECK!(loaded == f);
        MINI_CHECK!(loaded.outlines.len() == 1);
        // Read back, not re-minted: whoever holds the guid must still find this feature.
        MINI_CHECK!(loaded.guid() == feature_guid);
    })
}

pub fn run_element_feature_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::element::ElementFeature;
        use crate::{Point, Polyline};

        let f = ElementFeature::new("drill", 5,
            vec![Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
                                    Point::new(1.0, 1.0, 0.0), Point::new(0.0, 0.0, 0.0)])],
            "hole");
        let feature_guid = f.guid().to_string();

        let path = "serialization/test_element_feature.bin";
        f.pb_dump(path);
        let loaded = ElementFeature::pb_load(path).unwrap();

        MINI_CHECK!(loaded == f);
        MINI_CHECK!(loaded.feature_type == "drill");
        MINI_CHECK!(loaded.face_index == 5);
        MINI_CHECK!(loaded.outlines.len() == 1);
        MINI_CHECK!(loaded.guid() == feature_guid);
    })
}

///////////////////////////////////////////////////////////////////////////////////////////
// Registration
///////////////////////////////////////////////////////////////////////////////////////////

REGISTER_MINI_TEST!("Element", "Constructor", crate::element_test::run_element_constructor);
REGISTER_MINI_TEST!("Element", "Place", crate::element_test::run_element_place);
REGISTER_MINI_TEST!("Element", "Add Geometry Op", crate::element_test::run_element_add_feature);
REGISTER_MINI_TEST!("Element", "AABB", crate::element_test::run_element_aabb);
REGISTER_MINI_TEST!("Element", "OBB", crate::element_test::run_element_obb);
REGISTER_MINI_TEST!("Element", "Session Geometry", crate::element_test::run_element_session_geometry);
REGISTER_MINI_TEST!("Element", "Reset", crate::element_test::run_element_reset);
REGISTER_MINI_TEST!("Element", "Compute Point", crate::element_test::run_element_compute_point);
REGISTER_MINI_TEST!("Element", "Brep Aabb", crate::element_test::run_element_brep_aabb);
REGISTER_MINI_TEST!("Element", "Json Roundtrip", crate::element_test::run_element_json_roundtrip);
REGISTER_MINI_TEST!("Element", "Protobuf Roundtrip", crate::element_test::run_element_protobuf_roundtrip);
REGISTER_MINI_TEST!("Element", "Polylines", crate::element_test::run_element_polylines);
REGISTER_MINI_TEST!("Element", "RegistryRoundTrip", crate::element_test::run_element_registry_round_trip);
REGISTER_MINI_TEST!("Element", "RegistryUnknownTypeDegrades", crate::element_test::run_element_registry_unknown_type_degrades);
REGISTER_MINI_TEST!("Element", "RegistryLeavesBaseBytesUnchanged", crate::element_test::run_element_registry_leaves_base_bytes_unchanged);
REGISTER_MINI_TEST!("Element", "FeaturesRoundTrip", crate::element_test::run_element_features_round_trip);
REGISTER_MINI_TEST!("Element", "DimensionsAreNominalNotMeasured", crate::element_test::run_element_dimensions_are_nominal_not_measured);
REGISTER_MINI_TEST!("Element", "UnknownTypeSurvivesResave", crate::element_test::run_element_unknown_type_survives_resave);
REGISTER_MINI_TEST!("Element", "DuplicateKeepsEveryField", crate::element_test::run_element_duplicate_keeps_every_field);
REGISTER_MINI_TEST!("Element", "EqualityComparesCarriedFields", crate::element_test::run_element_equality_compares_carried_fields);
REGISTER_MINI_TEST!("ElementFeature", "Constructor", crate::element_test::run_element_feature_constructor);
REGISTER_MINI_TEST!("ElementFeature", "Json Roundtrip", crate::element_test::run_element_feature_json_roundtrip);
REGISTER_MINI_TEST!("ElementFeature", "Protobuf Roundtrip", crate::element_test::run_element_feature_protobuf_roundtrip);
