use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_encoders_file_json_dump_load() -> TestResult {
    MINI_TEST!("Json Dump Load", {
        use crate::file_encoders::file_json_dump;
        use crate::file_encoders::file_json_load;
        use crate::Point;
        use std::fs;

        let mut original = Point::new(1.5, 2.5, 3.5);
        original.name = "test_point".to_string();

        let filepath = "serialization/test_encoders_point.json";
        file_json_dump(&original, filepath, true).unwrap();

        let loaded: Point = file_json_load(filepath).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(loaded[0], original[0]));
        MINI_CHECK!(TOLERANCE.is_close(loaded[1], original[1]));
        MINI_CHECK!(TOLERANCE.is_close(loaded[2], original[2]));
        MINI_CHECK!(loaded.name == original.name);

        fs::remove_file(filepath).ok();
    })
}

pub fn run_encoders_file_json_dumps_loads() -> TestResult {
    MINI_TEST!("Json Dumps Loads", {
        use crate::file_encoders::file_json_dumps;
        use crate::file_encoders::file_json_loads;
        use crate::Vector;

        let mut original = Vector::new(42.1, 84.2, 126.3);
        original.name = "test_vector".to_string();

        let json_str = file_json_dumps(&original, true).unwrap();

        MINI_CHECK!(!json_str.is_empty());
        MINI_CHECK!(json_str.contains("Vector"));

        let loaded: Vector = file_json_loads(&json_str).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(loaded[0], original[0]));
        MINI_CHECK!(TOLERANCE.is_close(loaded[1], original[1]));
        MINI_CHECK!(TOLERANCE.is_close(loaded[2], original[2]));
        MINI_CHECK!(loaded.name == original.name);
    })
}

pub fn run_encoders_file_encode_collection_values() -> TestResult {
    MINI_TEST!("Encode Collection Values", {
        use crate::file_encoders::file_encode_collection;
        use crate::Point;

        let points = vec![
            Point::new(1.0, 2.0, 3.0),
            Point::new(4.0, 5.0, 6.0),
            Point::new(7.0, 8.0, 9.0),
        ];

        let json_arr = file_encode_collection(&points).unwrap();

        MINI_CHECK!(json_arr.is_array());
        MINI_CHECK!(json_arr.as_array().unwrap().len() == 3);
        MINI_CHECK!(json_arr[0]["type"] == "Point");
        MINI_CHECK!(json_arr[1]["x"] == 4.0);
        MINI_CHECK!(json_arr[2]["z"] == 9.0);
    })
}

pub fn run_encoders_file_encode_collection_shared_ptr() -> TestResult {
    MINI_TEST!("Encode Collection Shared Ptr", {
        use crate::file_encoders::file_encode_collection;
        use crate::Line;
        use std::rc::Rc;

        let lines = vec![
            Rc::new(Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0)),
            Rc::new(Line::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0)),
        ];

        let json_arr = file_encode_collection(&lines).unwrap();

        MINI_CHECK!(json_arr.is_array());
        MINI_CHECK!(json_arr.as_array().unwrap().len() == 2);
        MINI_CHECK!(json_arr[0]["type"] == "Line");
        MINI_CHECK!(json_arr[1]["type"] == "Line");
    })
}

pub fn run_encoders_file_decode_collection() -> TestResult {
    MINI_TEST!("Decode Collection", {
        use crate::file_encoders::file_decode_collection;
        use crate::file_encoders::file_encode_collection;
        use crate::Point;

        let original_points = vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)];

        let json_arr = file_encode_collection(&original_points).unwrap();
        let decoded_points: Vec<Point> = file_decode_collection(&json_arr).unwrap();

        MINI_CHECK!(decoded_points.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(decoded_points[0][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(decoded_points[1][1], 5.0));
    })
}

pub fn run_encoders_file_decode_collection_ptr() -> TestResult {
    MINI_TEST!("Decode Collection Ptr", {
        use crate::file_encoders::file_decode_collection_ptr;
        use crate::file_encoders::file_encode_collection;
        use crate::Vector;
        use std::rc::Rc;

        let original_vectors = vec![
            Rc::new(Vector::new(1.0, 0.0, 0.0)),
            Rc::new(Vector::new(0.0, 1.0, 0.0)),
        ];

        let json_arr = file_encode_collection(&original_vectors).unwrap();
        let decoded_vectors: Vec<Rc<Vector>> = file_decode_collection_ptr(&json_arr).unwrap();

        MINI_CHECK!(decoded_vectors.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(decoded_vectors[0][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(decoded_vectors[1][1], 1.0));
    })
}

pub fn run_encoders_nested_collections() -> TestResult {
    MINI_TEST!("Nested Collections", {
        use crate::file_encoders::file_decode_collection;
        use crate::file_encoders::file_encode_collection;
        use crate::Line;

        let lines = vec![
            Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
            Line::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        ];

        let json_arr = file_encode_collection(&lines).unwrap();
        let json_str = json_arr.to_string();

        MINI_CHECK!(!json_str.is_empty());

        let loaded_json: serde_json::Value = serde_json::from_str(&json_str).unwrap();
        let loaded: Vec<Line> = file_decode_collection(&loaded_json).unwrap();

        MINI_CHECK!(loaded.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(loaded[0].end()[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded[1].end()[1], 1.0));
    })
}

pub fn run_encoders_roundtrip_file_io() -> TestResult {
    MINI_TEST!("Roundtrip File Io", {
        use crate::file_encoders::file_decode_collection;
        use crate::file_encoders::file_encode_collection;
        use crate::file_encoders::file_json_dump;
        use crate::file_encoders::file_json_load_data;
        use crate::Vector;
        use std::fs;

        let vectors = vec![
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
        ];

        let filepath = "serialization/test_encoders_collection.json";
        let json_arr = file_encode_collection(&vectors).unwrap();
        file_json_dump(&json_arr, filepath, true).unwrap();

        let loaded_json = file_json_load_data(filepath).unwrap();
        let decoded_vectors: Vec<Vector> = file_decode_collection(&loaded_json).unwrap();

        MINI_CHECK!(decoded_vectors.len() == 3);
        MINI_CHECK!(TOLERANCE.is_close(decoded_vectors[0][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(decoded_vectors[1][1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(decoded_vectors[2][2], 1.0));

        fs::remove_file(filepath).ok();
    })
}

pub fn run_encoders_pretty_vs_compact() -> TestResult {
    MINI_TEST!("Pretty Vs Compact", {
        use crate::file_encoders::file_json_dumps;
        use crate::file_encoders::file_json_loads;
        use crate::Point;

        let point = Point::new(1.0, 2.0, 3.0);

        let pretty = file_json_dumps(&point, true).unwrap();
        let compact = file_json_dumps(&point, false).unwrap();

        MINI_CHECK!(pretty.len() > compact.len());
        MINI_CHECK!(pretty.contains('\n'));
        MINI_CHECK!(!compact.contains('\n'));

        let loaded_pretty: Point = file_json_loads(&pretty).unwrap();
        let loaded_compact: Point = file_json_loads(&compact).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(loaded_pretty[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded_compact[0], 1.0));
    })
}

#[allow(clippy::approx_constant)]
pub fn run_encoders_decode_primitives() -> TestResult {
    MINI_TEST!("Decode Primitives", {
        let num: i32 = 42;
        let json_str = serde_json::to_string(&num).unwrap();
        let loaded: i32 = serde_json::from_str(&json_str).unwrap();

        MINI_CHECK!(loaded == 42);

        let float_val: f64 = 3.14;
        let json_str = serde_json::to_string(&float_val).unwrap();
        let loaded: f64 = serde_json::from_str(&json_str).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(loaded, 3.14));

        let text = "hello";
        let json_str = serde_json::to_string(&text).unwrap();
        let loaded: String = serde_json::from_str(&json_str).unwrap();

        MINI_CHECK!(loaded == "hello");

        let flag = true;
        let json_str = serde_json::to_string(&flag).unwrap();
        let loaded: bool = serde_json::from_str(&json_str).unwrap();

        MINI_CHECK!(loaded);
    })
}

pub fn run_encoders_decode_list() -> TestResult {
    MINI_TEST!("Decode List", {
        use crate::file_encoders::file_decode_collection;
        use crate::file_encoders::file_encode_collection;
        use crate::Point;

        let data = vec![1, 2, 3];
        let json_str = serde_json::to_string(&data).unwrap();
        let loaded_vec: Vec<i32> = serde_json::from_str(&json_str).unwrap();

        MINI_CHECK!(loaded_vec.len() == 3);
        MINI_CHECK!(loaded_vec[0] == 1);
        MINI_CHECK!(loaded_vec[2] == 3);

        let points = vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)];

        let json_arr = file_encode_collection(&points).unwrap();
        let decoded: Vec<Point> = file_decode_collection(&json_arr).unwrap();

        MINI_CHECK!(decoded.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(decoded[0][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(decoded[1][0], 4.0));
    })
}

pub fn run_encoders_decode_dict() -> TestResult {
    MINI_TEST!("Decode Dict", {
        use crate::file_encoders::file_json_dumps;
        use crate::file_encoders::file_json_loads;
        use crate::Vector;
        use std::collections::BTreeMap;

        let mut data = BTreeMap::new();
        data.insert("a".to_string(), 1);
        data.insert("b".to_string(), 2);

        let json_str = serde_json::to_string(&data).unwrap();
        let loaded: BTreeMap<String, i32> = serde_json::from_str(&json_str).unwrap();

        MINI_CHECK!(loaded["a"] == 1);
        MINI_CHECK!(loaded["b"] == 2);

        let vec = Vector::new(1.0, 2.0, 3.0);
        let vec_json = file_json_dumps(&vec, true).unwrap();
        let loaded_vec: Vector = file_json_loads(&vec_json).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(loaded_vec[0], 1.0));
    })
}

pub fn run_encoders_list_in_list_in_list() -> TestResult {
    MINI_TEST!("List In List In List", {
        let data = vec![vec![vec![1, 2], vec![3, 4]], vec![vec![5, 6], vec![7, 8]]];
        let json_str = serde_json::to_string(&data).unwrap();
        let loaded: Vec<Vec<Vec<i32>>> = serde_json::from_str(&json_str).unwrap();

        MINI_CHECK!(loaded[0][0][0] == 1);
        MINI_CHECK!(loaded[1][1][1] == 8);
        MINI_CHECK!(loaded.len() == 2);
    })
}

pub fn run_encoders_dict_of_lists() -> TestResult {
    MINI_TEST!("Dict Of Lists", {
        use crate::file_encoders::file_decode_collection;
        use crate::file_encoders::file_encode_collection;
        use crate::Point;

        let points = vec![Point::new(1.0, 0.0, 0.0), Point::new(0.0, 1.0, 0.0)];

        let mut data = serde_json::json!({});
        data["numbers"] = serde_json::json!([1, 2, 3]);
        data["letters"] = serde_json::json!(["a", "b", "c"]);
        data["points"] = file_encode_collection(&points).unwrap();

        let json_str = data.to_string();
        let loaded: serde_json::Value = serde_json::from_str(&json_str).unwrap();

        MINI_CHECK!(loaded["numbers"].as_array().unwrap().len() == 3);
        MINI_CHECK!(loaded["letters"][0] == "a");

        let loaded_points: Vec<Point> = file_decode_collection(&loaded["points"]).unwrap();

        MINI_CHECK!(loaded_points.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(loaded_points[0][0], 1.0));
    })
}

pub fn run_encoders_list_of_dict() -> TestResult {
    MINI_TEST!("List Of Dict", {
        use crate::Point;

        let point = Point::new(1.0, 2.0, 3.0);

        let data = vec![
            serde_json::json!({"name": "point1", "value": 10}),
            serde_json::json!({"name": "point2", "value": 20}),
            serde_json::json!({"geometry": point}),
        ];

        let json_str = serde_json::to_string(&data).unwrap();
        let loaded: serde_json::Value = serde_json::from_str(&json_str).unwrap();

        MINI_CHECK!(loaded.as_array().unwrap().len() == 3);
        MINI_CHECK!(loaded[0]["name"] == "point1");
        MINI_CHECK!(loaded[1]["value"] == 20);

        let loaded_point: Point = serde_json::from_value(loaded[2]["geometry"].clone()).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(loaded_point[2], 3.0));
    })
}

pub fn run_encoders_dict_of_dicts() -> TestResult {
    MINI_TEST!("Dict Of Dicts", {
        use crate::Point;
        use crate::Vector;

        let point = Point::new(1.0, 2.0, 3.0);
        let vec = Vector::new(0.0, 0.0, 1.0);

        let mut data = serde_json::json!({"config": {}, "geometry": {}});
        data["config"]["tolerance"] = serde_json::json!(0.001);
        data["config"]["scale"] = serde_json::json!(1000);
        data["geometry"]["point"] = serde_json::to_value(&point).unwrap();
        data["geometry"]["vector"] = serde_json::to_value(&vec).unwrap();

        let json_str = data.to_string();
        let loaded: serde_json::Value = serde_json::from_str(&json_str).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(loaded["config"]["tolerance"].as_f64().unwrap(), 0.001));
        MINI_CHECK!(loaded["config"]["scale"] == 1000);

        let loaded_point: Point =
            serde_json::from_value(loaded["geometry"]["point"].clone()).unwrap();
        let loaded_vec: Vector =
            serde_json::from_value(loaded["geometry"]["vector"].clone()).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(loaded_point[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded_vec[2], 1.0));
    })
}

pub fn run_encoders_write_error() -> TestResult {
    MINI_TEST!("Write Error", {
        use crate::file_encoders::file_json_dump;
        use crate::Point;

        let point = Point::new(1.0, 2.0, 3.0);
        let result = file_json_dump(&point, "serialization/missing-directory/test.json", true);

        MINI_CHECK!(result.is_err());
    })
}

REGISTER_MINI_TEST!(
    "FileEncoders",
    "Json Dump Load",
    crate::file_encoders_test::run_encoders_file_json_dump_load
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Json Dumps Loads",
    crate::file_encoders_test::run_encoders_file_json_dumps_loads
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Encode Collection Values",
    crate::file_encoders_test::run_encoders_file_encode_collection_values
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Encode Collection Shared Ptr",
    crate::file_encoders_test::run_encoders_file_encode_collection_shared_ptr
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Decode Collection",
    crate::file_encoders_test::run_encoders_file_decode_collection
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Decode Collection Ptr",
    crate::file_encoders_test::run_encoders_file_decode_collection_ptr
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Nested Collections",
    crate::file_encoders_test::run_encoders_nested_collections
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Roundtrip File Io",
    crate::file_encoders_test::run_encoders_roundtrip_file_io
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Pretty Vs Compact",
    crate::file_encoders_test::run_encoders_pretty_vs_compact
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Decode Primitives",
    crate::file_encoders_test::run_encoders_decode_primitives
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Decode List",
    crate::file_encoders_test::run_encoders_decode_list
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Decode Dict",
    crate::file_encoders_test::run_encoders_decode_dict
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "List In List In List",
    crate::file_encoders_test::run_encoders_list_in_list_in_list
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Dict Of Lists",
    crate::file_encoders_test::run_encoders_dict_of_lists
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "List Of Dict",
    crate::file_encoders_test::run_encoders_list_of_dict
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Dict Of Dicts",
    crate::file_encoders_test::run_encoders_dict_of_dicts
);
REGISTER_MINI_TEST!(
    "FileEncoders",
    "Write Error",
    crate::file_encoders_test::run_encoders_write_error
);
