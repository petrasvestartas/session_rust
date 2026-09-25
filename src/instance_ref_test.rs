use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_instance_ref_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::element::ElementFeature;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Polyline;
        use crate::Xform;

        let x = Xform::translation(10.0, 20.0, 30.0);
        let inst = InstanceRef::new("def-123", x.clone());

        let mut instset = inst.duplicate();
        instset[0] = 2.0;
        let m0 = instset[0];

        let istr = inst.str();
        let irepr = inst.repr();

        let instcopy = inst.duplicate();
        let instother = InstanceRef::new("def-123", x.clone());
        let named = InstanceRef::with_name("custom", "def-9", Xform::identity());

        let mut featured = InstanceRef::new("def-123", x);
        featured.features.push(ElementFeature::new(
            "contact",
            0,
            vec![Polyline::new(vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
            ])],
            "",
        ));
        let featuredcopy = featured.duplicate();

        MINI_CHECK!(inst.name == "my_instance_ref");
        MINI_CHECK!(inst.definition_guid == "def-123");
        MINI_CHECK!(!inst.guid().is_empty());
        MINI_CHECK!(m0 == 2.0);
        MINI_CHECK!(inst[12] == 10.0 && inst[13] == 20.0 && inst[14] == 30.0);
        MINI_CHECK!(istr.contains("def-123"));
        MINI_CHECK!(irepr.contains("InstanceRef"));
        MINI_CHECK!(irepr.contains("my_instance_ref"));
        MINI_CHECK!(instcopy.guid() != inst.guid());
        MINI_CHECK!(inst == instother);
        MINI_CHECK!(inst != named);
        MINI_CHECK!(named.name == "custom" && named.definition_guid == "def-9");
        MINI_CHECK!(inst.features.is_empty());
        MINI_CHECK!(inst != featured);
        MINI_CHECK!(featuredcopy == featured && featuredcopy.guid() != featured.guid());
        MINI_CHECK!(
            InstanceRef::FLAG_HIDDEN == 1
                && InstanceRef::FLAG_LOCKED == 2
                && InstanceRef::FLAG_COLOR == 4
        );
    })
}

pub fn run_instance_ref_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::InstanceRef;
        use crate::Xform;

        let mut inst = InstanceRef::new("def", Xform::translation(1.0, 0.0, 0.0));
        let moved = inst.transformed(&Xform::translation(5.0, 0.0, 0.0));
        inst.transform(&Xform::translation(5.0, 0.0, 0.0));

        MINI_CHECK!(TOLERANCE.is_close(moved[12], 6.0));
        MINI_CHECK!(TOLERANCE.is_close(inst[12], 6.0));
    })
}

pub fn run_instance_ref_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::element::ElementFeature;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Polyline;
        use crate::Xform;

        let mut inst = InstanceRef::new("def-abc", Xform::translation(1.0, 2.0, 3.0));
        inst.name = "test_ref".to_string();
        inst.flags = 7;
        inst.features.push(ElementFeature::new(
            "contact",
            0,
            vec![Polyline::new(vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
            ])],
            "",
        ));
        let feature = inst.features[0].guid().to_string();

        let j = inst.jsondump().unwrap();
        let loaded_j = InstanceRef::jsonload(&j).unwrap();
        let mut bare: serde_json::Value = serde_json::from_str(&inst.jsondump().unwrap()).unwrap();
        bare.as_object_mut().unwrap().remove("features");
        bare.as_object_mut().unwrap().remove("flags");
        let loaded_bare = InstanceRef::jsonload(&bare.to_string()).unwrap();

        MINI_CHECK!(loaded_j.name == "test_ref");
        MINI_CHECK!(loaded_j.definition_guid == "def-abc");
        MINI_CHECK!(loaded_j.flags == 7);
        MINI_CHECK!(loaded_j.features.len() == 1);
        MINI_CHECK!(loaded_j.features[0].guid() == feature);
        MINI_CHECK!(loaded_j == inst);
        MINI_CHECK!(loaded_bare.flags == 0 && loaded_bare.features.is_empty());
        MINI_CHECK!(TOLERANCE.is_close(loaded_j[12], 1.0));

        let s = inst.file_json_dumps();
        let loaded_s = InstanceRef::file_json_loads(&s);

        MINI_CHECK!(loaded_s.name == "test_ref");
        MINI_CHECK!(loaded_s.definition_guid == "def-abc");

        let filename = "serialization/test_instance_ref.json";
        inst.file_json_dump(filename).unwrap();
        let loaded = InstanceRef::file_json_load(filename).unwrap();

        MINI_CHECK!(loaded.name == "test_ref");
        MINI_CHECK!(loaded.definition_guid == "def-abc");
        MINI_CHECK!(loaded.flags == 7);
        MINI_CHECK!(TOLERANCE.is_close(loaded[12], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded[13], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded[14], 3.0));
    })
}

pub fn run_instance_ref_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::element::ElementFeature;
        use crate::Color;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Polyline;
        use crate::Xform;

        let fresh = InstanceRef::default();
        let fresh_proto = fresh.to_proto();
        let mut inst = InstanceRef::new("def-xyz", Xform::translation(1.0, 2.0, 3.0));
        inst.name = "test_ref".to_string();
        inst.flags = 5;
        inst.features.push(ElementFeature::new(
            "contact",
            0,
            vec![Polyline::new(vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
            ])],
            "",
        ));
        let feature = inst.features[0].guid().to_string();
        let mut plain = InstanceRef::new("def-xyz", Xform::identity());
        plain.color = Color::red();
        let plain_proto = plain.to_proto();
        let plain_loaded = InstanceRef::from_proto(plain_proto.clone());

        let guid = inst.guid().to_string();
        let b = inst.pb_dumps();
        let loaded_b = InstanceRef::pb_loads(&b).unwrap();
        let converted = InstanceRef::from_proto(inst.to_proto());

        MINI_CHECK!(!fresh.has_guid());
        MINI_CHECK!(fresh_proto.guid.is_empty());
        MINI_CHECK!(plain_proto.xform.is_none());
        MINI_CHECK!(plain_proto.color.is_none());
        MINI_CHECK!(plain_loaded.xform == Xform::identity());
        MINI_CHECK!(plain_loaded.color == Color::white());
        MINI_CHECK!(loaded_b.features.len() == 1);
        MINI_CHECK!(loaded_b.features[0].guid() == feature);
        MINI_CHECK!(loaded_b.name == "test_ref");
        MINI_CHECK!(loaded_b.definition_guid == "def-xyz");
        MINI_CHECK!(loaded_b.flags == 5);
        MINI_CHECK!(loaded_b.guid() == guid);
        MINI_CHECK!(TOLERANCE.is_close(loaded_b[14], 3.0));
        MINI_CHECK!(converted == inst);
        MINI_CHECK!(converted.guid() == guid);

        let filename = "serialization/test_instance_ref.bin";
        inst.pb_dump(filename).unwrap();
        let loaded = InstanceRef::pb_load(filename).unwrap();

        MINI_CHECK!(loaded.name == "test_ref");
        MINI_CHECK!(loaded.definition_guid == "def-xyz");
        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(TOLERANCE.is_close(loaded[12], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded[13], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded[14], 3.0));
    })
}

REGISTER_MINI_TEST!(
    "InstanceRef",
    "Constructor",
    crate::instance_ref_test::run_instance_ref_constructor
);
REGISTER_MINI_TEST!(
    "InstanceRef",
    "Transformation",
    crate::instance_ref_test::run_instance_ref_transformation
);
REGISTER_MINI_TEST!(
    "InstanceRef",
    "Json Roundtrip",
    crate::instance_ref_test::run_instance_ref_json_roundtrip
);
REGISTER_MINI_TEST!(
    "InstanceRef",
    "Protobuf Roundtrip",
    crate::instance_ref_test::run_instance_ref_protobuf_roundtrip
);
