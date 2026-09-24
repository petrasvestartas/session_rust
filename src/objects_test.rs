use crate::mini_test::TestResult;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_objects_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Objects;

        let objects = Objects::new();
        let named = Objects::with_name("custom_objects");

        MINI_CHECK!(objects.name == "my_objects");
        MINI_CHECK!(!objects.guid().is_empty());
        MINI_CHECK!(objects.points.is_empty());
        MINI_CHECK!(objects.instances.is_empty());
        MINI_CHECK!(
            objects.str()
                == "Objects(name=my_objects, guid=".to_string() + objects.guid() + ", points=0)"
        );
        MINI_CHECK!(objects.repr() == objects.str());
        MINI_CHECK!(named.name == "custom_objects");
    })
}

pub fn run_objects_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::InstanceRef;
        use crate::Line;
        use crate::Mesh;
        use crate::Objects;
        use crate::Plane;
        use crate::Point;
        use crate::Xform;
        use std::rc::Rc;

        let mut original = Objects::new();
        original.points.push(Rc::new(Point::new(1.0, 2.0, 3.0)));
        original.points.push(Rc::new(Point::new(4.0, 5.0, 6.0)));
        original
            .lines
            .push(Rc::new(Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0)));
        original.planes.push(Rc::new(Plane::xy_plane()));
        original
            .meshes
            .push(Rc::new(Mesh::create_box(1.0, 1.0, 1.0)));
        let instance = InstanceRef::new("def-abc", Xform::identity());
        let guid = instance.guid().to_string();
        original.instances.push(Rc::new(instance));

        let filename = "serialization/test_objects.json";
        original.file_json_dump(filename).unwrap();
        let loaded = Objects::file_json_load(filename).unwrap();
        let parsed = Objects::file_json_loads(&original.file_json_dumps());

        MINI_CHECK!(loaded.guid() == original.guid());
        MINI_CHECK!(parsed.guid() == original.guid());
        MINI_CHECK!(loaded.points.len() == 2);
        MINI_CHECK!(loaded.points[1][0] == 4.0);
        MINI_CHECK!(loaded.lines.len() == 1);
        MINI_CHECK!(loaded.lines[0].end()[0] == 1.0);
        MINI_CHECK!(loaded.planes.len() == 1);
        MINI_CHECK!(loaded.planes[0].z_axis()[2] == 1.0);
        MINI_CHECK!(loaded.meshes.len() == 1);
        MINI_CHECK!(loaded.meshes[0].number_of_faces() == 6);
        MINI_CHECK!(loaded.instances.len() == 1);
        MINI_CHECK!(loaded.instances[0].guid() == guid);
        MINI_CHECK!(loaded.instances[0].definition_guid == "def-abc");
    })
}

pub fn run_objects_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::InstanceRef;
        use crate::Line;
        use crate::Mesh;
        use crate::Objects;
        use crate::Plane;
        use crate::Point;
        use crate::Xform;
        use std::rc::Rc;

        let mut original = Objects::new();
        original.points.push(Rc::new(Point::new(1.0, 2.0, 3.0)));
        original.points.push(Rc::new(Point::new(4.0, 5.0, 6.0)));
        original
            .lines
            .push(Rc::new(Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0)));
        original.planes.push(Rc::new(Plane::xy_plane()));
        original
            .meshes
            .push(Rc::new(Mesh::create_box(1.0, 1.0, 1.0)));
        let instance = InstanceRef::new("def-abc", Xform::identity());
        let guid = instance.guid().to_string();
        original.instances.push(Rc::new(instance));

        let filename = "serialization/test_objects.bin";
        original.pb_dump(filename);
        let loaded = Objects::pb_load(filename);
        let parsed = Objects::pb_loads(&original.pb_dumps()).unwrap();

        MINI_CHECK!(parsed.points.len() == 2);
        MINI_CHECK!(loaded.points.len() == 2);
        MINI_CHECK!(loaded.points[1][0] == 4.0);
        MINI_CHECK!(loaded.lines.len() == 1);
        MINI_CHECK!(loaded.lines[0].end()[0] == 1.0);
        MINI_CHECK!(loaded.planes.len() == 1);
        MINI_CHECK!(loaded.planes[0].z_axis()[2] == 1.0);
        MINI_CHECK!(loaded.meshes.len() == 1);
        MINI_CHECK!(loaded.meshes[0].number_of_faces() == 6);
        MINI_CHECK!(loaded.instances.len() == 1);
        MINI_CHECK!(loaded.instances[0].guid() == guid);
        MINI_CHECK!(loaded.instances[0].definition_guid == "def-abc");
    })
}

pub fn run_objects_component_constructor() -> TestResult {
    MINI_TEST!("Component Constructor", {
        use crate::Component;

        let mut component = Component::new();
        component.type_name = "FloorBuilder".to_string();
        component.name = "floor_builder".to_string();
        component
            .extra
            .insert("size".to_string(), serde_json::json!(3000));
        component
            .extra
            .insert("height".to_string(), serde_json::json!(650));

        MINI_CHECK!(Component::new().name == "my_component");
        MINI_CHECK!(component.type_name == "FloorBuilder");
        MINI_CHECK!(component.name == "floor_builder");
        MINI_CHECK!(!component.guid().is_empty());
        MINI_CHECK!(component.extra["size"] == serde_json::json!(3000));
    })
}

pub fn run_objects_component_json_roundtrip() -> TestResult {
    MINI_TEST!("Component Json Roundtrip", {
        use crate::Component;

        let mut original = Component::new();
        original.type_name = "FloorBuilder".to_string();
        original.name = "floor_builder".to_string();
        original
            .extra
            .insert("size".to_string(), serde_json::json!(3000));
        original
            .extra
            .insert("height".to_string(), serde_json::json!(650));
        original
            .extra
            .insert("rise".to_string(), serde_json::json!(453));
        let guid = original.guid().to_string();

        let data = original.jsondump().unwrap();

        MINI_CHECK!(data["type"] == "FloorBuilder");
        MINI_CHECK!(data["guid"] == guid);
        MINI_CHECK!(data["size"] == 3000);
        MINI_CHECK!(data["height"] == 650);

        let loaded = Component::jsonload(&data).unwrap();

        MINI_CHECK!(loaded.type_name == "FloorBuilder");
        MINI_CHECK!(loaded.guid() == guid);
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
        let mut component = Component::new();
        component.type_name = "FloorBuilder".to_string();
        component.name = "floor_builder".to_string();
        component
            .extra
            .insert("size".to_string(), serde_json::json!(3000));
        component
            .extra
            .insert("height".to_string(), serde_json::json!(650));
        let guid = component.guid().to_string();
        original.components.push(component);

        let filename = "serialization/test_objects_component.json";
        file_json_dump(&original, filename, false).unwrap();
        let loaded = file_json_load::<Objects>(filename).unwrap();

        MINI_CHECK!(loaded.components.len() == 1);
        MINI_CHECK!(loaded.components[0].type_name == "FloorBuilder");
        MINI_CHECK!(loaded.components[0].extra["size"] == serde_json::json!(3000));
        MINI_CHECK!(loaded.components[0].guid() == guid);
    })
}

pub fn run_objects_component_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Component Protobuf Roundtrip", {
        use crate::Component;
        use crate::Objects;

        let mut original = Objects::new();
        let mut component = Component::new();
        component.type_name = "FloorBuilder".to_string();
        component.name = "floor_builder".to_string();
        component
            .extra
            .insert("size".to_string(), serde_json::json!(3000));
        component
            .extra
            .insert("height".to_string(), serde_json::json!(650));
        let guid = component.guid().to_string();
        original.components.push(component);

        let filename = "serialization/test_objects_component.bin";
        original.pb_dump(filename);
        let loaded = Objects::pb_load(filename);

        MINI_CHECK!(loaded.components.len() == 1);
        MINI_CHECK!(loaded.components[0].type_name == "FloorBuilder");
        MINI_CHECK!(loaded.components[0].guid() == guid);
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
