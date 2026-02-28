use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_encoders_json_dump_load() -> TestResult {
    MINI_TEST!("Json Dump Load", {
        use crate::Point;
        use crate::encoders::{json_dump, json_load};
        use std::fs;

        let mut original = Point::new(1.5, 2.5, 3.5);
        original.name = "test_point".to_string();
        let filepath = "serialization/test_encoders_point.json";
        json_dump(&original, filepath, false).unwrap();
        let loaded: Point = json_load(filepath).unwrap();

        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded[0], original[0]));
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded[1], original[1]));
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded[2], original[2]));
        MINI_CHECK!(loaded.name == original.name);

        fs::remove_file(filepath).ok();
    })
}

pub fn run_encoders_json_dumps_loads() -> TestResult {
    MINI_TEST!("Json Dumps Loads", {
        use crate::Vector;
        use crate::encoders::{json_dumps, json_loads};

        let mut original = Vector::new(42.1, 84.2, 126.3);
        original.name = "test_vector".to_string();
        let json_str = json_dumps(&original, false).unwrap();

        MINI_CHECK!(!json_str.is_empty());
        MINI_CHECK!(json_str.contains("Vector"));

        let loaded: Vector = json_loads(&json_str).unwrap();

        MINI_CHECK!(loaded.name == original.name);
    })
}

pub fn run_encoders_encode_collection_values() -> TestResult {
    MINI_TEST!("Encode Collection Values", {
        use crate::Point;
        use crate::encoders::json_dumps;

        let points = vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0), Point::new(7.0, 8.0, 9.0)];
        let json_str = json_dumps(&points, false).unwrap();
        let json_arr: serde_json::Value = serde_json::from_str(&json_str).unwrap();

        MINI_CHECK!(json_arr.is_array());
        MINI_CHECK!(json_arr.as_array().unwrap().len() == 3);
        MINI_CHECK!(json_arr[0]["type"] == "Point");
        MINI_CHECK!(json_arr[1]["x"] == 4.0);
        MINI_CHECK!(json_arr[2]["z"] == 9.0);
    })
}

pub fn run_encoders_encode_collection_shared_ptr() -> TestResult {
    MINI_TEST!("Encode Collection Shared Ptr", {
        use crate::Line;
        use crate::encoders::json_dumps;

        let lines = vec![Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0), Line::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0)];
        let json_str = json_dumps(&lines, false).unwrap();
        let json_arr: serde_json::Value = serde_json::from_str(&json_str).unwrap();

        MINI_CHECK!(json_arr.is_array());
        MINI_CHECK!(json_arr.as_array().unwrap().len() == 2);
        MINI_CHECK!(json_arr[0]["type"] == "Line");
        MINI_CHECK!(json_arr[1]["type"] == "Line");
    })
}

pub fn run_encoders_decode_collection() -> TestResult {
    MINI_TEST!("Decode Collection", {
        use crate::Point;
        use crate::encoders::{json_dumps, json_loads};

        let original_points = vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)];
        let json_str = json_dumps(&original_points, false).unwrap();
        let decoded: Vec<Point> = json_loads(&json_str).unwrap();

        MINI_CHECK!(decoded.len() == 2);
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(decoded[0][0], 1.0));
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(decoded[1][1], 5.0));
    })
}

pub fn run_encoders_decode_collection_ptr() -> TestResult {
    MINI_TEST!("Decode Collection Ptr", {
        use crate::Vector;
        use crate::encoders::{json_dumps, json_loads};

        let original_vectors = vec![Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0)];
        let json_str = json_dumps(&original_vectors, false).unwrap();
        let decoded: Vec<Vector> = json_loads(&json_str).unwrap();

        MINI_CHECK!(decoded.len() == 2);
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(decoded[0][0], 1.0));
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(decoded[1][1], 1.0));
    })
}

pub fn run_encoders_nested_collections() -> TestResult {
    MINI_TEST!("Nested Collections", {
        use crate::Line;
        use crate::encoders::{json_dumps, json_loads};

        let lines = vec![Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0), Line::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0)];
        let json_str = json_dumps(&lines, false).unwrap();
        let loaded: Vec<Line> = json_loads(&json_str).unwrap();

        MINI_CHECK!(loaded.len() == 2);
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded[0].end()[0], 1.0));
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded[1].end()[1], 1.0));
    })
}

pub fn run_encoders_roundtrip_file_io() -> TestResult {
    MINI_TEST!("Roundtrip File Io", {
        use crate::Vector;
        use crate::encoders::{json_dump, json_load};
        use std::fs;

        let vectors = vec![Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0), Vector::new(0.0, 0.0, 1.0)];
        let filepath = "serialization/test_encoders_collection.json";
        json_dump(&vectors, filepath, false).unwrap();
        let decoded: Vec<Vector> = json_load(filepath).unwrap();

        MINI_CHECK!(decoded.len() == 3);

        fs::remove_file(filepath).ok();
    })
}

pub fn run_encoders_pretty_vs_compact() -> TestResult {
    MINI_TEST!("Pretty Vs Compact", {
        use crate::Point;
        use crate::encoders::{json_dumps, json_loads};

        let point = Point::new(1.0, 2.0, 3.0);
        let pretty = json_dumps(&point, true).unwrap();
        let compact = json_dumps(&point, false).unwrap();

        MINI_CHECK!(pretty.len() > compact.len());
        MINI_CHECK!(pretty.contains('\n'));
        MINI_CHECK!(!compact.contains('\n'));

        let loaded_pretty: Point = json_loads(&pretty).unwrap();
        let loaded_compact: Point = json_loads(&compact).unwrap();

        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded_pretty[0], 1.0));
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded_compact[0], 1.0));
    })
}

pub fn run_encoders_decode_primitives() -> TestResult {
    MINI_TEST!("Decode Primitives", {
        use crate::encoders::{json_dumps, json_loads};

        let json_str = json_dumps(&42i32, false).unwrap();
        let loaded: i32 = json_loads(&json_str).unwrap();
        MINI_CHECK!(loaded == 42);

        let json_str = json_dumps(&3.14f64, false).unwrap();
        let loaded: f64 = json_loads(&json_str).unwrap();
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded, 3.14));

        let json_str = json_dumps(&"hello", false).unwrap();
        let loaded: String = json_loads(&json_str).unwrap();
        MINI_CHECK!(loaded == "hello");

        let json_str = json_dumps(&true, false).unwrap();
        let loaded: bool = json_loads(&json_str).unwrap();
        MINI_CHECK!(loaded);
    })
}

pub fn run_encoders_decode_list() -> TestResult {
    MINI_TEST!("Decode List", {
        use crate::Point;
        use crate::encoders::{json_dumps, json_loads};

        let data = vec![1i32, 2, 3];
        let json_str = json_dumps(&data, false).unwrap();
        let loaded: Vec<i32> = json_loads(&json_str).unwrap();

        MINI_CHECK!(loaded.len() == 3);
        MINI_CHECK!(loaded[0] == 1);
        MINI_CHECK!(loaded[2] == 3);

        let points = vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)];
        let json_str = json_dumps(&points, false).unwrap();
        let decoded: Vec<Point> = json_loads(&json_str).unwrap();

        MINI_CHECK!(decoded.len() == 2);
    })
}

pub fn run_encoders_decode_dict() -> TestResult {
    MINI_TEST!("Decode Dict", {
        use crate::Vector;
        use crate::encoders::{json_dumps, json_loads};
        use std::collections::HashMap;

        let mut data = HashMap::new();
        data.insert("a".to_string(), 1i32);
        data.insert("b".to_string(), 2i32);
        let json_str = json_dumps(&data, false).unwrap();
        let loaded: HashMap<String, i32> = json_loads(&json_str).unwrap();

        MINI_CHECK!(loaded.get("a") == Some(&1));
        MINI_CHECK!(loaded.get("b") == Some(&2));

        let vec = Vector::new(1.0, 2.0, 3.0);
        let json_str = json_dumps(&vec, false).unwrap();
        let loaded_vec: Vector = json_loads(&json_str).unwrap();

        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded_vec[0], 1.0));
    })
}

pub fn run_encoders_list_in_list_in_list() -> TestResult {
    MINI_TEST!("List In List In List", {
        use crate::encoders::{json_dumps, json_loads};

        let data = vec![vec![vec![1i32, 2], vec![3, 4]], vec![vec![5, 6], vec![7, 8]]];
        let json_str = json_dumps(&data, false).unwrap();
        let loaded: Vec<Vec<Vec<i32>>> = json_loads(&json_str).unwrap();

        MINI_CHECK!(loaded[0][0][0] == 1);
        MINI_CHECK!(loaded[1][1][1] == 8);
        MINI_CHECK!(loaded.len() == 2);
    })
}

pub fn run_encoders_dict_of_lists() -> TestResult {
    MINI_TEST!("Dict Of Lists", {
        use crate::Point;
        use crate::encoders::json_loads;

        let points = vec![Point::new(1.0, 0.0, 0.0), Point::new(0.0, 1.0, 0.0)];
        let data = serde_json::json!({
            "numbers": [1, 2, 3],
            "letters": ["a", "b", "c"],
            "points": points
        });
        let json_str = data.to_string();
        let loaded: serde_json::Value = json_loads(&json_str).unwrap();

        MINI_CHECK!(loaded["numbers"].as_array().unwrap().len() == 3);
        MINI_CHECK!(loaded["letters"][0] == "a");

        let loaded_points: Vec<Point> = serde_json::from_value(loaded["points"].clone()).unwrap();
        MINI_CHECK!(loaded_points.len() == 2);
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded_points[0][0], 1.0));
    })
}

pub fn run_encoders_list_of_dict() -> TestResult {
    MINI_TEST!("List Of Dict", {
        use crate::Point;
        use crate::encoders::json_loads;

        let point = Point::new(1.0, 2.0, 3.0);
        let data = serde_json::json!([
            {"name": "point1", "value": 10},
            {"name": "point2", "value": 20},
            {"geometry": point}
        ]);
        let json_str = data.to_string();
        let loaded: serde_json::Value = json_loads(&json_str).unwrap();

        MINI_CHECK!(loaded[0]["name"] == "point1");
        MINI_CHECK!(loaded[1]["value"] == 20);

        let loaded_point: Point = serde_json::from_value(loaded[2]["geometry"].clone()).unwrap();
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded_point[2], 3.0));
    })
}

pub fn run_encoders_dict_of_dicts() -> TestResult {
    MINI_TEST!("Dict Of Dicts", {
        use crate::Point;
        use crate::Vector;
        use crate::encoders::json_loads;

        let point = Point::new(1.0, 2.0, 3.0);
        let vec = Vector::new(0.0, 0.0, 1.0);
        let data = serde_json::json!({
            "config": {"tolerance": 0.001, "scale": 1000},
            "geometry": {"point": point, "vector": vec}
        });
        let json_str = data.to_string();
        let loaded: serde_json::Value = json_loads(&json_str).unwrap();

        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded["config"]["tolerance"].as_f64().unwrap(), 0.001));
        MINI_CHECK!(loaded["config"]["scale"] == 1000);

        let loaded_point: Point = serde_json::from_value(loaded["geometry"]["point"].clone()).unwrap();
        let loaded_vec: Vector = serde_json::from_value(loaded["geometry"]["vector"].clone()).unwrap();
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded_point[0], 1.0));
        MINI_CHECK!(crate::tolerance::TOLERANCE.is_close(loaded_vec[2], 1.0));
    })
}

REGISTER_MINI_TEST!("Encoders", "Json Dump Load", crate::encoders_test::run_encoders_json_dump_load);
REGISTER_MINI_TEST!("Encoders", "Json Dumps Loads", crate::encoders_test::run_encoders_json_dumps_loads);
REGISTER_MINI_TEST!("Encoders", "Encode Collection Values", crate::encoders_test::run_encoders_encode_collection_values);
REGISTER_MINI_TEST!("Encoders", "Encode Collection Shared Ptr", crate::encoders_test::run_encoders_encode_collection_shared_ptr);
REGISTER_MINI_TEST!("Encoders", "Decode Collection", crate::encoders_test::run_encoders_decode_collection);
REGISTER_MINI_TEST!("Encoders", "Decode Collection Ptr", crate::encoders_test::run_encoders_decode_collection_ptr);
REGISTER_MINI_TEST!("Encoders", "Nested Collections", crate::encoders_test::run_encoders_nested_collections);
REGISTER_MINI_TEST!("Encoders", "Roundtrip File Io", crate::encoders_test::run_encoders_roundtrip_file_io);
REGISTER_MINI_TEST!("Encoders", "Pretty Vs Compact", crate::encoders_test::run_encoders_pretty_vs_compact);
REGISTER_MINI_TEST!("Encoders", "Decode Primitives", crate::encoders_test::run_encoders_decode_primitives);
REGISTER_MINI_TEST!("Encoders", "Decode List", crate::encoders_test::run_encoders_decode_list);
REGISTER_MINI_TEST!("Encoders", "Decode Dict", crate::encoders_test::run_encoders_decode_dict);
REGISTER_MINI_TEST!("Encoders", "List In List In List", crate::encoders_test::run_encoders_list_in_list_in_list);
REGISTER_MINI_TEST!("Encoders", "Dict Of Lists", crate::encoders_test::run_encoders_dict_of_lists);
REGISTER_MINI_TEST!("Encoders", "List Of Dict", crate::encoders_test::run_encoders_list_of_dict);
REGISTER_MINI_TEST!("Encoders", "Dict Of Dicts", crate::encoders_test::run_encoders_dict_of_dicts);
