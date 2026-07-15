use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_instance_ref_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::InstanceRef;
        use crate::Xform;

        // Constructor from a definition guid and a placement transform
        let x = Xform::translation(10.0, 20.0, 30.0);
        let r = InstanceRef::new("def-123", x.clone());

        // Setter on a copy (keep r pristine for the == check below)
        let mut rset = r.duplicate();
        rset[0] = 2.0;
        let m0 = rset[0];

        // Minimal and Full String Representation
        let rstr = r.str();
        let rrepr = r.repr();

        // Copy (duplicate everything but guid)
        let rcopy = r.duplicate();
        let rother = InstanceRef::new("def-123", x.clone());

        // with_name constructor
        let rwn = InstanceRef::with_name("custom", "def-9", Xform::identity());

        MINI_CHECK!(r.name == "my_instance_ref" && !r.guid().is_empty());
        MINI_CHECK!(r.definition_guid == "def-123");
        MINI_CHECK!(m0 == 2.0);
        MINI_CHECK!(r[12] == 10.0 && r[13] == 20.0 && r[14] == 30.0);
        MINI_CHECK!(rstr.contains("def-123"));
        MINI_CHECK!(rrepr.contains("InstanceRef") && rrepr.contains("my_instance_ref"));
        MINI_CHECK!(rcopy.guid() != r.guid());
        MINI_CHECK!(r == rother);
        MINI_CHECK!(r != rwn);
        MINI_CHECK!(rwn.name == "custom" && rwn.definition_guid == "def-9");
    })
}

pub fn run_instance_ref_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::InstanceRef;
        use crate::Xform;

        let mut r = InstanceRef::new("def", Xform::translation(1.0, 0.0, 0.0));
        let moved = r.transformed(&Xform::translation(5.0, 0.0, 0.0)); // Make a copy
        r.transform(&Xform::translation(5.0, 0.0, 0.0)); // compose in place

        // translation(5) * translation(1) => translation(6)
        MINI_CHECK!(TOLERANCE.is_close(moved[12], 6.0));
        MINI_CHECK!(TOLERANCE.is_close(r[12], 6.0));
    })
}

pub fn run_instance_ref_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::InstanceRef;
        use crate::Xform;

        let mut r = InstanceRef::new("def-abc", Xform::translation(1.0, 2.0, 3.0));
        r.name = "test_ref".to_string();
        r.flags = 7;

        // JSON object (string)
        let js = r.jsondump().unwrap();
        let loaded_j = InstanceRef::jsonload(&js).unwrap();

        MINI_CHECK!(loaded_j.name == "test_ref");
        MINI_CHECK!(loaded_j.definition_guid == "def-abc");
        MINI_CHECK!(loaded_j.flags == 7);
        MINI_CHECK!(TOLERANCE.is_close(loaded_j[12], 1.0));

        // String
        let s = r.file_json_dumps();
        let loaded_s = InstanceRef::file_json_loads(&s);
        MINI_CHECK!(loaded_s.name == "test_ref");
        MINI_CHECK!(loaded_s.definition_guid == "def-abc");

        // File
        let fname = "serialization/test_instance_ref.json";
        r.file_json_dump(fname).unwrap();
        let loaded = InstanceRef::file_json_load(fname).unwrap();

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
        use crate::InstanceRef;
        use crate::Xform;

        let mut r = InstanceRef::new("def-xyz", Xform::translation(1.0, 2.0, 3.0));
        r.name = "test_ref".to_string();
        r.flags = 5;

        // Bytes
        let b = r.pb_dumps();
        let loaded_s = InstanceRef::pb_loads(&b).unwrap();

        MINI_CHECK!(loaded_s.name == "test_ref");
        MINI_CHECK!(loaded_s.definition_guid == "def-xyz");
        MINI_CHECK!(loaded_s.flags == 5);
        MINI_CHECK!(loaded_s.guid() == r.guid());
        MINI_CHECK!(TOLERANCE.is_close(loaded_s[14], 3.0));

        // File
        let fname = "serialization/test_instance_ref.bin";
        r.pb_dump(fname);
        let loaded = InstanceRef::pb_load(fname);

        MINI_CHECK!(loaded.name == "test_ref");
        MINI_CHECK!(loaded.definition_guid == "def-xyz");
        MINI_CHECK!(loaded.guid() == r.guid());
        MINI_CHECK!(TOLERANCE.is_close(loaded[12], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded[13], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded[14], 3.0));
    })
}

// Register tests with the shared registry for run_all("rust")
REGISTER_MINI_TEST!("InstanceRef", "Constructor", crate::instance_ref_test::run_instance_ref_constructor);
REGISTER_MINI_TEST!("InstanceRef", "Transformation", crate::instance_ref_test::run_instance_ref_transformation);
REGISTER_MINI_TEST!("InstanceRef", "Json Roundtrip", crate::instance_ref_test::run_instance_ref_json_roundtrip);
REGISTER_MINI_TEST!("InstanceRef", "Protobuf Roundtrip", crate::instance_ref_test::run_instance_ref_protobuf_roundtrip);
