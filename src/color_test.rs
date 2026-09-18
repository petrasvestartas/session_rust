use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_color_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Color;

        let cdefault = Color::default();
        let clamped = Color::new(-1.0, 2.0, 0.5, 3.0);
        let mut c = Color::with_name(1.0, 0.0, 0.0, 1.0, "red");
        let fresh = !c.has_guid();

        c[0] = 1.0;
        c[1] = 0.0;
        c[2] = 0.0;
        c[3] = 1.0;

        let r = c[0];
        let g = c[1];
        let b = c[2];
        let a = c[3];

        let cstr = c.str();
        let crepr = c.repr();

        let ccopy = c.duplicate();
        let cother = Color::with_name(1.0, 0.0, 0.0, 1.0, "red");

        MINI_CHECK!(cdefault == Color::default());
        MINI_CHECK!(clamped == Color::new(0.0, 1.0, 0.5, 1.0));
        MINI_CHECK!(fresh);
        MINI_CHECK!(c.name == "red");
        MINI_CHECK!(!c.guid().is_empty());
        MINI_CHECK!(c[0] == 1.0 && c[1] == 0.0 && c[2] == 0.0 && c[3] == 1.0);
        MINI_CHECK!(r == 1.0 && g == 0.0 && b == 0.0 && a == 1.0);
        MINI_CHECK!(cstr == "1.0, 0.0, 0.0, 1.0");
        MINI_CHECK!(crepr == "Color(red, 1.0, 0.0, 0.0, 1.0)");
        MINI_CHECK!(ccopy == cother);
        MINI_CHECK!(c != Color::blue());
        MINI_CHECK!(ccopy.guid() != c.guid());
    })
}

pub fn run_color_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Color;

        let c = Color::with_name(1.0, 0.5, 0.25, 1.0, "test_color");

        let guid = c.guid().to_string();
        let filename = "serialization/test_color.json";
        c.file_json_dump(filename).unwrap();
        let loaded = Color::file_json_load(filename).unwrap();
        let parsed = Color::file_json_loads(&c.file_json_dumps());

        MINI_CHECK!(loaded.name == "test_color");
        MINI_CHECK!(loaded[0] == 1.0);
        MINI_CHECK!(loaded[1] == 0.5);
        MINI_CHECK!(loaded[2] == 0.25);
        MINI_CHECK!(loaded[3] == 1.0);
        MINI_CHECK!(parsed == c);
        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(parsed.guid() == guid);
    })
}

pub fn run_color_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Color;

        let fresh = Color::default();
        let fresh_proto = fresh.to_proto();
        let c = Color::with_name(1.0, 0.5, 0.25, 1.0, "test_color");

        let guid = c.guid().to_string();
        let filename = "serialization/test_color.bin";
        c.pb_dump(filename);
        let loaded = Color::pb_load(filename);
        let parsed = Color::pb_loads(&c.pb_dumps()).unwrap();
        let converted = Color::from_proto(c.to_proto());

        MINI_CHECK!(!fresh.has_guid());
        MINI_CHECK!(fresh_proto.guid.is_empty());
        MINI_CHECK!(loaded.name == "test_color");
        MINI_CHECK!(loaded[0] == 1.0);
        MINI_CHECK!(loaded[1] == 0.5);
        MINI_CHECK!(loaded[2] == 0.25);
        MINI_CHECK!(loaded[3] == 1.0);
        MINI_CHECK!(parsed == c);
        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(parsed.guid() == guid);
        MINI_CHECK!(converted == c);
        MINI_CHECK!(converted.guid() == guid);
    })
}

pub fn run_color_conversion() -> TestResult {
    MINI_TEST!("Conversion", {
        use crate::Color;

        let c = Color::new(1.0, 0.5, 0.25, 1.0);
        let flts = c.to_unified_array();
        let back = Color::from_unified_array(flts);

        MINI_CHECK!(TOLERANCE.is_close(flts[0] as f64, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(flts[1] as f64, 0.5));
        MINI_CHECK!(TOLERANCE.is_close(flts[2] as f64, 0.25));
        MINI_CHECK!(TOLERANCE.is_close(flts[3] as f64, 1.0));
        MINI_CHECK!(back == c);
    })
}

pub fn run_color_presets() -> TestResult {
    MINI_TEST!("Presets", {
        use crate::Color;

        let white = Color::white();
        let black = Color::black();
        let grey = Color::grey();
        let red = Color::red();
        let orange = Color::orange();
        let yellow = Color::yellow();
        let lime = Color::lime();
        let green = Color::green();
        let mint = Color::mint();
        let cyan = Color::cyan();
        let azure = Color::azure();
        let blue = Color::blue();
        let violet = Color::violet();
        let magenta = Color::magenta();
        let pink = Color::pink();
        let maroon = Color::maroon();
        let brown = Color::brown();
        let olive = Color::olive();
        let teal = Color::teal();
        let navy = Color::navy();
        let purple = Color::purple();
        let silver = Color::silver();
        let lightgrey = Color::lightgrey();
        let palette = Color::palette();

        MINI_CHECK!(white == Color::with_name(1.0, 1.0, 1.0, 1.0, "white"));
        MINI_CHECK!(black == Color::with_name(0.0, 0.0, 0.0, 1.0, "black"));
        MINI_CHECK!(grey == Color::with_name(0.5, 0.5, 0.5, 1.0, "grey"));
        MINI_CHECK!(red == Color::with_name(1.0, 0.0, 0.0, 1.0, "red"));
        MINI_CHECK!(orange == Color::with_name(1.0, 0.5, 0.0, 1.0, "orange"));
        MINI_CHECK!(yellow == Color::with_name(1.0, 1.0, 0.0, 1.0, "yellow"));
        MINI_CHECK!(lime == Color::with_name(0.5, 1.0, 0.0, 1.0, "lime"));
        MINI_CHECK!(green == Color::with_name(0.0, 1.0, 0.0, 1.0, "green"));
        MINI_CHECK!(mint == Color::with_name(0.0, 1.0, 0.5, 1.0, "mint"));
        MINI_CHECK!(cyan == Color::with_name(0.0, 1.0, 1.0, 1.0, "cyan"));
        MINI_CHECK!(azure == Color::with_name(0.0, 0.5, 1.0, 1.0, "azure"));
        MINI_CHECK!(blue == Color::with_name(0.0, 0.0, 1.0, 1.0, "blue"));
        MINI_CHECK!(violet == Color::with_name(0.5, 0.0, 1.0, 1.0, "violet"));
        MINI_CHECK!(magenta == Color::with_name(1.0, 0.0, 1.0, 1.0, "magenta"));
        MINI_CHECK!(pink == Color::with_name(1.0, 0.0, 0.5, 1.0, "pink"));
        MINI_CHECK!(maroon == Color::with_name(0.5, 0.0, 0.0, 1.0, "maroon"));
        MINI_CHECK!(brown == Color::with_name(0.5, 0.25, 0.0, 1.0, "brown"));
        MINI_CHECK!(olive == Color::with_name(0.5, 0.5, 0.0, 1.0, "olive"));
        MINI_CHECK!(teal == Color::with_name(0.0, 0.5, 0.5, 1.0, "teal"));
        MINI_CHECK!(navy == Color::with_name(0.0, 0.0, 0.5, 1.0, "navy"));
        MINI_CHECK!(purple == Color::with_name(0.5, 0.0, 0.5, 1.0, "purple"));
        MINI_CHECK!(silver == Color::with_name(0.75, 0.75, 0.75, 1.0, "silver"));
        MINI_CHECK!(lightgrey == Color::with_name(0.9, 0.9, 0.9, 1.0, "lightgrey"));
        MINI_CHECK!(
            palette
                == vec![
                    red, orange, yellow, lime, green, mint, cyan, azure, blue, violet, magenta,
                    pink,
                ]
        );
    })
}

pub fn run_color_serialization_errors() -> TestResult {
    MINI_TEST!("Serialization Errors", {
        use crate::Color;

        let color = Color::default();
        let malformed_json = Color::jsonload("{}").is_err();
        let malformed_pb = Color::pb_loads(&[0xff]).is_err();
        let json_write_failed = color.file_json_dump("").is_err();

        MINI_CHECK!(malformed_json);
        MINI_CHECK!(malformed_pb);
        MINI_CHECK!(json_write_failed);

        #[cfg(panic = "unwind")]
        {
            let pb_write_failed = std::panic::catch_unwind(|| color.pb_dump("")).is_err();
            MINI_CHECK!(pb_write_failed);
        }
    })
}

REGISTER_MINI_TEST!(
    "Color",
    "Constructor",
    crate::color_test::run_color_constructor
);
REGISTER_MINI_TEST!(
    "Color",
    "Json Roundtrip",
    crate::color_test::run_color_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Color",
    "Protobuf Roundtrip",
    crate::color_test::run_color_protobuf_roundtrip
);
REGISTER_MINI_TEST!(
    "Color",
    "Conversion",
    crate::color_test::run_color_conversion
);
REGISTER_MINI_TEST!("Color", "Presets", crate::color_test::run_color_presets);
REGISTER_MINI_TEST!(
    "Color",
    "Serialization Errors",
    crate::color_test::run_color_serialization_errors
);
