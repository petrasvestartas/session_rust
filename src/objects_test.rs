use crate::mini_test::TestResult;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_objects_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Objects;
        let obj = Objects::new();
        let mut named = Objects::new();
        named.name = "custom_objects".to_string();

        MINI_CHECK!(obj.name == "my_objects");
        MINI_CHECK!(!obj.guid().is_empty());
        MINI_CHECK!(!obj.to_string().is_empty());
        MINI_CHECK!(named.name == "custom_objects");
    })
}

pub fn run_objects_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::file_encoders::file_json_dump;
        use crate::file_encoders::file_json_load;
        use crate::InstanceRef;
        use crate::Objects;
        use crate::Point;
        use crate::Xform;
        use std::rc::Rc;
        let mut original = Objects::new();
        let point1 = Rc::new(Point::new(1.0, 2.0, 3.0));
        let point2 = Rc::new(Point::new(4.0, 5.0, 6.0));
        original.points.push(point1);
        original.points.push(point2);
        let instance = InstanceRef::new("def-abc", Xform::identity());
        let guid = instance.guid().to_string();
        original.instances.push(Rc::new(instance));

        let filename = "serialization/test_objects.json";
        file_json_dump(&original, filename, false).unwrap();
        let loaded = file_json_load::<Objects>(filename).unwrap();

        MINI_CHECK!(loaded.points.len() == original.points.len());
        MINI_CHECK!(loaded.instances.len() == 1);
        MINI_CHECK!(loaded.instances[0].guid() == guid);
        MINI_CHECK!(loaded.instances[0].definition_guid == "def-abc");
    })
}

pub fn run_objects_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::InstanceRef;
        use crate::Objects;
        use crate::Point;
        use crate::Xform;
        use std::rc::Rc;
        let mut original = Objects::new();
        let point1 = Rc::new(Point::new(1.0, 2.0, 3.0));
        let point2 = Rc::new(Point::new(4.0, 5.0, 6.0));
        original.points.push(point1);
        original.points.push(point2);
        let instance = InstanceRef::new("def-abc", Xform::identity());
        let guid = instance.guid().to_string();
        original.instances.push(Rc::new(instance));

        let filename = "serialization/test_objects.bin";
        original.pb_dump(filename);
        let loaded = Objects::pb_load(filename);

        MINI_CHECK!(loaded.points.len() == original.points.len());
        MINI_CHECK!(loaded.instances.len() == 1);
        MINI_CHECK!(loaded.instances[0].guid() == guid);
        MINI_CHECK!(loaded.instances[0].definition_guid == "def-abc");
    })
}

pub fn run_objects_component_constructor() -> TestResult {
    MINI_TEST!("Component Constructor", {
        use crate::Component;
        let mut c = Component {
            type_name: "FloorBuilder".to_string(),
            guid: uuid::Uuid::new_v4().to_string(),
            name: "floor_builder".to_string(),
            extra: std::collections::HashMap::new(),
        };
        c.extra.insert("size".to_string(), serde_json::json!(3000));
        c.extra.insert("height".to_string(), serde_json::json!(650));

        MINI_CHECK!(c.type_name == "FloorBuilder");
        MINI_CHECK!(c.name == "floor_builder");
        MINI_CHECK!(!c.guid().is_empty());
        MINI_CHECK!(c.extra["size"] == serde_json::json!(3000));
    })
}

pub fn run_objects_component_json_roundtrip() -> TestResult {
    MINI_TEST!("Component Json Roundtrip", {
        use crate::Component;
        let mut original = Component {
            type_name: "FloorBuilder".to_string(),
            guid: uuid::Uuid::new_v4().to_string(),
            name: "floor_builder".to_string(),
            extra: std::collections::HashMap::new(),
        };
        original
            .extra
            .insert("size".to_string(), serde_json::json!(3000));
        original
            .extra
            .insert("height".to_string(), serde_json::json!(650));
        original
            .extra
            .insert("rise".to_string(), serde_json::json!(453));
        let original_guid = original.guid().to_string();

        let j = original.jsondump().unwrap();

        MINI_CHECK!(j["type"] == "FloorBuilder");
        MINI_CHECK!(j["guid"] == original_guid);
        MINI_CHECK!(j["size"] == 3000);
        MINI_CHECK!(j["height"] == 650);

        let loaded = Component::jsonload(&j).unwrap();

        MINI_CHECK!(loaded.type_name == "FloorBuilder");
        MINI_CHECK!(loaded.guid() == original_guid);
        MINI_CHECK!(loaded.extra["size"] == serde_json::json!(3000));
        MINI_CHECK!(loaded.extra["rise"] == serde_json::json!(453));
    })
}

pub fn run_objects_objects_component_json_roundtrip() -> TestResult {
    MINI_TEST!("Objects Component Json Roundtrip", {
        use crate::file_encoders::file_json_dump;
        use crate::file_encoders::file_json_load;
        use crate::Component;
        use crate::Objects;
        let mut original = Objects::new();
        let mut c = Component {
            type_name: "FloorBuilder".to_string(),
            guid: uuid::Uuid::new_v4().to_string(),
            name: "floor_builder".to_string(),
            extra: std::collections::HashMap::new(),
        };
        c.extra.insert("size".to_string(), serde_json::json!(3000));
        c.extra.insert("height".to_string(), serde_json::json!(650));
        let expected_guid = c.guid().to_string();
        original.components.push(c);

        let filename = "serialization/test_objects_component.json";
        file_json_dump(&original, filename, false).unwrap();
        let loaded = file_json_load::<Objects>(filename).unwrap();

        MINI_CHECK!(loaded.components.len() == 1);
        MINI_CHECK!(loaded.components[0].type_name == "FloorBuilder");
        MINI_CHECK!(loaded.components[0].extra["size"] == serde_json::json!(3000));
        MINI_CHECK!(loaded.components[0].guid() == expected_guid);
    })
}

pub fn run_objects_component_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Component Protobuf Roundtrip", {
        use crate::Component;
        use crate::Objects;
        let mut original = Objects::new();
        let mut c = Component {
            type_name: "FloorBuilder".to_string(),
            guid: uuid::Uuid::new_v4().to_string(),
            name: "floor_builder".to_string(),
            extra: std::collections::HashMap::new(),
        };
        c.extra.insert("size".to_string(), serde_json::json!(3000));
        c.extra.insert("height".to_string(), serde_json::json!(650));
        let expected_guid = c.guid().to_string();
        original.components.push(c);

        let filename = "serialization/test_objects_component.bin";
        original.pb_dump(filename);
        let loaded = Objects::pb_load(filename);

        MINI_CHECK!(loaded.components.len() == 1);
        MINI_CHECK!(loaded.components[0].type_name == "FloorBuilder");
        MINI_CHECK!(loaded.components[0].guid() == expected_guid);
        MINI_CHECK!(loaded.components[0].extra["size"] == serde_json::json!(3000));
    })
}

REGISTER_MINI_TEST!(
    "Objects",
    "Constructor",
    crate::objects_test::run_objects_constructor
);
REGISTER_MINI_TEST!(
    "Objects",
    "Json Roundtrip",
    crate::objects_test::run_objects_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Objects",
    "Protobuf Roundtrip",
    crate::objects_test::run_objects_protobuf_roundtrip
);
REGISTER_MINI_TEST!(
    "Objects",
    "Component Constructor",
    crate::objects_test::run_objects_component_constructor
);
REGISTER_MINI_TEST!(
    "Objects",
    "Component Json Roundtrip",
    crate::objects_test::run_objects_component_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Objects",
    "Objects Component Json Roundtrip",
    crate::objects_test::run_objects_objects_component_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Objects",
    "Component Protobuf Roundtrip",
    crate::objects_test::run_objects_component_protobuf_roundtrip
);
