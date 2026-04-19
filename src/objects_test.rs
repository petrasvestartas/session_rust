use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_objects_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Objects;
        let obj = Objects::new();

        MINI_CHECK!(obj.name == "my_objects");
        MINI_CHECK!(!obj.guid().is_empty());
        MINI_CHECK!(!obj.to_string().is_empty());
    })
}

pub fn run_objects_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::{Objects, Point};
        use crate::encoders::{json_dump, json_load};
        let mut original = Objects::new();
        original.points.push(Point::new(1.0, 2.0, 3.0));
        original.points.push(Point::new(4.0, 5.0, 6.0));
        json_dump(&original, "serialization/test_objects.json", false).unwrap();
        let loaded = json_load::<Objects>("serialization/test_objects.json").unwrap();
        MINI_CHECK!(loaded.points.len() == original.points.len());
    })
}

pub fn run_objects_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::{Objects, Point};
        let mut original = Objects::new();
        original.points.push(Point::new(1.0, 2.0, 3.0));
        original.points.push(Point::new(4.0, 5.0, 6.0));
        original.pb_dump("serialization/test_objects.bin");
        let loaded = Objects::pb_load("serialization/test_objects.bin");
        MINI_CHECK!(loaded.points.len() == original.points.len());
    })
}

pub fn run_objects_component_constructor() -> TestResult {
    MINI_TEST!("Component Constructor", {
        // Component is a generic envelope for custom domain objects.
        // type_name identifies the class; extra holds all custom fields as JSON.
        use crate::Component;

        let mut extra = std::collections::HashMap::new();
        extra.insert("size".to_string(),   serde_json::json!(3000));
        extra.insert("height".to_string(), serde_json::json!(650));

        let c = Component {
            type_name: "FloorBuilder".to_string(),
            guid:       uuid::Uuid::new_v4().to_string(),
            name:       "floor_builder".to_string(),
            extra,
        };

        MINI_CHECK!(c.type_name == "FloorBuilder");
        MINI_CHECK!(c.name == "floor_builder");
        MINI_CHECK!(!c.guid.is_empty());
        MINI_CHECK!(c.extra["size"] == serde_json::json!(3000));
    })
}

pub fn run_objects_component_json_roundtrip() -> TestResult {
    MINI_TEST!("Component Json Roundtrip", {
        // Round-trip a Component through JSON: all custom fields must survive.
        // jsondump produces a flat dict (type/guid/name + extra fields).
        // jsonload reconstructs it; serde flatten handles the extra fields.
        use crate::{Objects, Component};
        use crate::encoders::{json_dump, json_load};

        let mut extra = std::collections::HashMap::new();
        extra.insert("size".to_string(),   serde_json::json!(3000));
        extra.insert("height".to_string(), serde_json::json!(650));
        extra.insert("rise".to_string(),   serde_json::json!(453));

        let original_guid = uuid::Uuid::new_v4().to_string();
        let c = Component {
            type_name: "FloorBuilder".to_string(),
            guid:       original_guid.clone(),
            name:       "floor_builder".to_string(),
            extra,
        };

        let mut original = Objects::new();
        original.components.push(c);

        json_dump(&original, "serialization/test_objects_component.json", false).unwrap();
        let loaded = json_load::<Objects>("serialization/test_objects_component.json").unwrap();

        MINI_CHECK!(loaded.components.len() == 1);
        MINI_CHECK!(loaded.components[0].type_name == "FloorBuilder");
        MINI_CHECK!(loaded.components[0].guid == original_guid);
        MINI_CHECK!(loaded.components[0].extra["size"] == serde_json::json!(3000));
        MINI_CHECK!(loaded.components[0].extra["rise"] == serde_json::json!(453));
    })
}

pub fn run_objects_component_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Component Protobuf Roundtrip", {
        // Custom fields are serialised as a JSON string inside the proto message
        // and reconstructed into a HashMap on load — all values survive.
        use crate::{Objects, Component};

        let mut extra = std::collections::HashMap::new();
        extra.insert("size".to_string(),   serde_json::json!(3000));
        extra.insert("height".to_string(), serde_json::json!(650));

        let original_guid = uuid::Uuid::new_v4().to_string();
        let c = Component {
            type_name: "FloorBuilder".to_string(),
            guid:       original_guid.clone(),
            name:       "floor_builder".to_string(),
            extra,
        };

        let mut original = Objects::new();
        original.components.push(c);

        original.pb_dump("serialization/test_objects_component.bin");
        let loaded = Objects::pb_load("serialization/test_objects_component.bin");

        MINI_CHECK!(loaded.components.len() == 1);
        MINI_CHECK!(loaded.components[0].type_name == "FloorBuilder");
        MINI_CHECK!(loaded.components[0].guid == original_guid);
        MINI_CHECK!(loaded.components[0].extra["size"] == serde_json::json!(3000));
    })
}

REGISTER_MINI_TEST!("Objects", "Constructor",                crate::objects_test::run_objects_constructor);
REGISTER_MINI_TEST!("Objects", "Json Roundtrip",             crate::objects_test::run_objects_json_roundtrip);
REGISTER_MINI_TEST!("Objects", "Protobuf Roundtrip",         crate::objects_test::run_objects_protobuf_roundtrip);
REGISTER_MINI_TEST!("Objects", "Component Constructor",      crate::objects_test::run_objects_component_constructor);
REGISTER_MINI_TEST!("Objects", "Component Json Roundtrip",   crate::objects_test::run_objects_component_json_roundtrip);
REGISTER_MINI_TEST!("Objects", "Component Protobuf Roundtrip", crate::objects_test::run_objects_component_protobuf_roundtrip);
