#[cfg(test)]
mod tests {
    use crate::file_encoders::{file_json_dump, file_json_load};
    use crate::Color;

    #[test]
    fn test_color_constructor() {
        let mut red = Color::new(1.0, 0.0, 0.0, 1.0);
        red.name = "red".to_string();
        assert_eq!(red.name, "red");
        assert!(!red.guid().is_empty());
        assert_eq!(red.r, 1.0);
        assert_eq!(red.g, 0.0);
        assert_eq!(red.b, 0.0);
        assert_eq!(red.a, 1.0);
    }

    #[test]
    fn test_color_equality() {
        let c1 = Color::new(0.0, 0.5, 0.25, 1.0);
        let c2 = Color::new(0.0, 0.5, 0.25, 1.0);
        assert_eq!(c1.r, c2.r);
        assert_eq!(c1.g, c2.g);
        assert_eq!(c1.b, c2.b);
        assert_eq!(c1.a, c2.a);

        let c3 = Color::new(0.0, 0.5, 0.25, 1.0);
        let c4 = Color::new(1.0, 0.5, 0.25, 1.0);
        assert_ne!(c3.r, c4.r);
    }

    #[test]
    fn test_color_to_json_data() {
        let mut color = Color::new(0.5, 0.25, 0.75, 1.0);
        color.name = "purple".to_string();

        let json_string = color.jsondump().unwrap();
        let data: serde_json::Value = serde_json::from_str(&json_string).unwrap();

        assert_eq!(data["type"], "Color");
        assert_eq!(data["name"], "purple");
        assert!(data["r"].is_number());
        assert!(data["g"].is_number());
        assert!(data["b"].is_number());
        assert!(data["a"].is_number());
        assert!(data["guid"].is_string());
    }

    #[test]
    fn test_color_from_json_data() {
        let mut original_color = Color::new(0.5, 0.25, 1.0, 1.0);
        original_color.name = "bronze".to_string();

        let json_string = original_color.jsondump().unwrap();
        let restored_color = Color::jsonload(&json_string).unwrap();

        assert_eq!(restored_color.r, original_color.r);
        assert_eq!(restored_color.g, original_color.g);
        assert_eq!(restored_color.b, original_color.b);
        assert_eq!(restored_color.a, original_color.a);
        assert_eq!(restored_color.name, "bronze");
        assert_eq!(restored_color.guid(), original_color.guid());
    }

    #[test]
    fn test_color_to_json_from_json() {
        let mut original = Color::new(1.0, 0.5, 0.25, 1.0);
        original.name = "sunset_orange".to_string();
        let filename = "serialization/test_color_roundtrip.json";

        file_json_dump(&original, filename, true).unwrap();
        let loaded = file_json_load::<Color>(filename).unwrap();

        assert_eq!(loaded.r, original.r);
        assert_eq!(loaded.g, original.g);
        assert_eq!(loaded.b, original.b);
        assert_eq!(loaded.a, original.a);
        assert_eq!(loaded.name, original.name);
        assert_eq!(loaded.guid(), original.guid());
    }

    #[test]
    fn test_color_white() {
        let white = Color::white();
        assert_eq!(white.name, "white");
        assert_eq!(white.r, 1.0);
        assert_eq!(white.g, 1.0);
        assert_eq!(white.b, 1.0);
        assert_eq!(white.a, 1.0);
    }

    #[test]
    fn test_color_black() {
        let black = Color::black();
        assert_eq!(black.name, "black");
        assert_eq!(black.r, 0.0);
        assert_eq!(black.g, 0.0);
        assert_eq!(black.b, 0.0);
        assert_eq!(black.a, 1.0);
    }

    #[test]
    fn test_color_to_float_array() {
        let color = Color::new(1.0, 0.5, 0.25, 1.0);
        let float_array = color.to_float_array();
        assert_eq!(float_array, [1.0, 0.5, 0.25, 1.0]);
    }

    #[test]
    fn test_color_from_float() {
        let color = Color::from_float(1.0, 0.5, 0.25, 1.0);
        assert_eq!(color.r, 1.0);
        assert_eq!(color.g, 0.5);
        assert_eq!(color.b, 0.25);
        assert_eq!(color.a, 1.0);
    }

    #[test]
    fn test_color_red() {
        let red = Color::red();
        assert_eq!(red.name, "red");
        assert_eq!(red.r, 1.0);
        assert_eq!(red.g, 0.0);
        assert_eq!(red.b, 0.0);
        assert_eq!(red.a, 1.0);
    }

    #[test]
    fn test_color_green() {
        let green = Color::green();
        assert_eq!(green.name, "green");
        assert_eq!(green.r, 0.0);
        assert_eq!(green.g, 1.0);
        assert_eq!(green.b, 0.0);
        assert_eq!(green.a, 1.0);
    }

    #[test]
    fn test_color_blue() {
        let blue = Color::blue();
        assert_eq!(blue.name, "blue");
        assert_eq!(blue.r, 0.0);
        assert_eq!(blue.g, 0.0);
        assert_eq!(blue.b, 1.0);
        assert_eq!(blue.a, 1.0);
    }

    #[test]
    fn test_color_grey() {
        let grey = Color::grey();
        assert_eq!(grey.name, "grey");
        assert_eq!(grey.r, 0.5);
        assert_eq!(grey.g, 0.5);
        assert_eq!(grey.b, 0.5);
        assert_eq!(grey.a, 1.0);
    }
}

// ============================================================================
// Mini-test framework tests (for cross-language test comparison)
// ============================================================================

use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_color_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Color;

        // Constructor
        let mut red = Color::new(1.0, 0.0, 0.0, 1.0);
        red.name = "red".to_string();

        // Setters
        red.r = 1.0;
        red.g = 0.0;
        red.b = 0.0;
        red.a = 1.0;

        // Getters
        let r = red.r;
        let g = red.g;
        let b = red.b;
        let a = red.a;

        // Minimal and Full String Representation
        let cstr = red.str();
        let crepr = red.repr();

        // Copy (duplicates everything except guid)
        let ccopy = red.duplicate();
        let mut cother = Color::new(1.0, 0.0, 0.0, 1.0);
        cother.name = "red".to_string();

        MINI_CHECK!(red.name == "red");
        MINI_CHECK!(!red.guid().is_empty());
        MINI_CHECK!(red.r == 1.0 && red.g == 0.0 && red.b == 0.0 && red.a == 1.0);
        MINI_CHECK!(r == 1.0 && g == 0.0 && b == 0.0 && a == 1.0);
        MINI_CHECK!(cstr == "1.0, 0.0, 0.0, 1.0");
        MINI_CHECK!(crepr == "Color(red, 1.0, 0.0, 0.0, 1.0)");
        MINI_CHECK!(ccopy == cother);
        MINI_CHECK!(ccopy.guid() != red.guid());
    })
}

pub fn run_color_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Color;

        let color = Color::with_name(1.0, 0.5, 0.25, 1.0, "test_color");

        //   file_json_dumps()    │ String       │ to JSON string
        //   file_json_loads(s)   │ String       │ from JSON string
        //   file_json_dump(path) │ file         │ write to file
        //   file_json_load(path) │ file         │ read from file

        let filename = "serialization/test_color.json";
        color.file_json_dump(filename).unwrap();
        let loaded = Color::file_json_load(filename).unwrap();

        MINI_CHECK!(loaded.name == "test_color");
        MINI_CHECK!(loaded.r == 1.0);
        MINI_CHECK!(loaded.g == 0.5);
        MINI_CHECK!(loaded.b == 0.25);
        MINI_CHECK!(loaded.a == 1.0);
    })
}

pub fn run_color_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Color;

        let mut color = Color::new(1.0, 0.5, 0.25, 1.0);
        color.name = "test_color".to_string();

        let filename = "serialization/test_color.bin";
        color.pb_dump(filename);
        let loaded = Color::pb_load(filename);

        MINI_CHECK!(loaded.name == "test_color");
        MINI_CHECK!(loaded.r == 1.0);
        MINI_CHECK!(loaded.g == 0.5);
        MINI_CHECK!(loaded.b == 0.25);
        MINI_CHECK!(loaded.a == 1.0);
    })
}

pub fn run_color_conversion() -> TestResult {
    MINI_TEST!("Conversion", {
        use crate::Color;

        let color = Color::new(1.0, 0.5, 0.25, 1.0);
        let flts = color.to_float_array();
        let color2 = Color::from_float(flts[0], flts[1], flts[2], flts[3]);

        MINI_CHECK!(TOLERANCE.is_close(flts[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(flts[1], 0.5));
        MINI_CHECK!(TOLERANCE.is_close(flts[2], 0.25));
        MINI_CHECK!(TOLERANCE.is_close(flts[3], 1.0));
        MINI_CHECK!(color2 == color);
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

        MINI_CHECK!(white.r == 1.0 && white.g == 1.0 && white.b == 1.0);
        MINI_CHECK!(white.name == "white");
        MINI_CHECK!(black.r == 0.0 && black.g == 0.0 && black.b == 0.0);
        MINI_CHECK!(black.name == "black");
        MINI_CHECK!(grey.r == 0.5 && grey.g == 0.5 && grey.b == 0.5);
        MINI_CHECK!(grey.name == "grey");
        MINI_CHECK!(red.r == 1.0 && red.g == 0.0 && red.b == 0.0);
        MINI_CHECK!(red.name == "red");
        MINI_CHECK!(orange.r == 1.0 && orange.g == 0.5 && orange.b == 0.0);
        MINI_CHECK!(orange.name == "orange");
        MINI_CHECK!(yellow.r == 1.0 && yellow.g == 1.0 && yellow.b == 0.0);
        MINI_CHECK!(yellow.name == "yellow");
        MINI_CHECK!(lime.r == 0.5 && lime.g == 1.0 && lime.b == 0.0);
        MINI_CHECK!(lime.name == "lime");
        MINI_CHECK!(green.r == 0.0 && green.g == 1.0 && green.b == 0.0);
        MINI_CHECK!(green.name == "green");
        MINI_CHECK!(mint.r == 0.0 && mint.g == 1.0 && mint.b == 0.5);
        MINI_CHECK!(mint.name == "mint");
        MINI_CHECK!(cyan.r == 0.0 && cyan.g == 1.0 && cyan.b == 1.0);
        MINI_CHECK!(cyan.name == "cyan");
        MINI_CHECK!(azure.r == 0.0 && azure.g == 0.5 && azure.b == 1.0);
        MINI_CHECK!(azure.name == "azure");
        MINI_CHECK!(blue.r == 0.0 && blue.g == 0.0 && blue.b == 1.0);
        MINI_CHECK!(blue.name == "blue");
        MINI_CHECK!(violet.r == 0.5 && violet.g == 0.0 && violet.b == 1.0);
        MINI_CHECK!(violet.name == "violet");
        MINI_CHECK!(magenta.r == 1.0 && magenta.g == 0.0 && magenta.b == 1.0);
        MINI_CHECK!(magenta.name == "magenta");
        MINI_CHECK!(pink.r == 1.0 && pink.g == 0.0 && pink.b == 0.5);
        MINI_CHECK!(pink.name == "pink");
        MINI_CHECK!(maroon.r == 0.5 && maroon.g == 0.0 && maroon.b == 0.0);
        MINI_CHECK!(maroon.name == "maroon");
        MINI_CHECK!(brown.r == 0.5 && brown.g == 0.25 && brown.b == 0.0);
        MINI_CHECK!(brown.name == "brown");
        MINI_CHECK!(olive.r == 0.5 && olive.g == 0.5 && olive.b == 0.0);
        MINI_CHECK!(olive.name == "olive");
        MINI_CHECK!(teal.r == 0.0 && teal.g == 0.5 && teal.b == 0.5);
        MINI_CHECK!(teal.name == "teal");
        MINI_CHECK!(navy.r == 0.0 && navy.g == 0.0 && navy.b == 0.5);
        MINI_CHECK!(navy.name == "navy");
        MINI_CHECK!(purple.r == 0.5 && purple.g == 0.0 && purple.b == 0.5);
        MINI_CHECK!(purple.name == "purple");
        MINI_CHECK!(silver.r == 0.75 && silver.g == 0.75 && silver.b == 0.75);
        MINI_CHECK!(silver.name == "silver");
    })
}

// Register tests with the shared registry for run_all("rust")
REGISTER_MINI_TEST!("Color", "Constructor", crate::color_test::run_color_constructor);
REGISTER_MINI_TEST!("Color", "Json Roundtrip", crate::color_test::run_color_json_roundtrip);
REGISTER_MINI_TEST!("Color", "Protobuf Roundtrip", crate::color_test::run_color_protobuf_roundtrip);
REGISTER_MINI_TEST!("Color", "Conversion", crate::color_test::run_color_conversion);
REGISTER_MINI_TEST!("Color", "Presets", crate::color_test::run_color_presets);
