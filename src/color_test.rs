#[cfg(test)]
mod tests {
    use crate::encoders::{json_dump, json_load};
    use crate::Color;

    #[test]
    fn test_color_constructor() {
        let mut red = Color::new(255, 0, 0, 255);
        red.name = "red".to_string();
        assert_eq!(red.name, "red");
        assert!(!red.guid.to_string().is_empty());
        assert_eq!(red.r, 255);
        assert_eq!(red.g, 0);
        assert_eq!(red.b, 0);
        assert_eq!(red.a, 255);
    }

    #[test]
    fn test_color_equality() {
        let c1 = Color::new(0, 100, 50, 200);
        let c2 = Color::new(0, 100, 50, 200);
        // Colors have different GUIDs, so they won't be equal by default
        // We'll compare the RGBA values instead
        assert_eq!(c1.r, c2.r);
        assert_eq!(c1.g, c2.g);
        assert_eq!(c1.b, c2.b);
        assert_eq!(c1.a, c2.a);

        let c3 = Color::new(0, 100, 50, 200);
        let c4 = Color::new(1, 100, 50, 200);
        assert_ne!(c3.r, c4.r);
    }

    #[test]
    fn test_color_to_json_data() {
        let mut color = Color::new(128, 64, 192, 255);
        color.name = "purple".to_string();

        let json_string = color.jsondump().unwrap();
        let data: serde_json::Value = serde_json::from_str(&json_string).unwrap();

        assert_eq!(data["type"], "Color");
        assert_eq!(data["name"], "purple");
        assert_eq!(data["r"], 128);
        assert_eq!(data["g"], 64);
        assert_eq!(data["b"], 192);
        assert_eq!(data["a"], 255);
        assert!(data["guid"].is_string());
    }

    #[test]
    fn test_color_from_json_data() {
        let mut original_color = Color::new(200, 150, 100, 255);
        original_color.name = "bronze".to_string();

        let json_string = original_color.jsondump().unwrap();
        let restored_color = Color::jsonload(&json_string).unwrap();

        assert_eq!(restored_color.r, 200);
        assert_eq!(restored_color.g, 150);
        assert_eq!(restored_color.b, 100);
        assert_eq!(restored_color.a, 255);
        assert_eq!(restored_color.name, "bronze");
        assert_eq!(restored_color.guid, original_color.guid);
    }

    #[test]
    fn test_color_to_json_from_json() {
        let mut original = Color::new(255, 128, 64, 255);
        original.name = "sunset_orange".to_string();
        let filename = "test_color.json";

        json_dump(&original, filename, true).unwrap();
        let loaded = json_load::<Color>(filename).unwrap();

        assert_eq!(loaded.r, original.r);
        assert_eq!(loaded.g, original.g);
        assert_eq!(loaded.b, original.b);
        assert_eq!(loaded.a, original.a);
        assert_eq!(loaded.name, original.name);
        assert_eq!(loaded.guid, original.guid);
    }

    #[test]
    fn test_color_white() {
        let white = Color::white();
        assert_eq!(white.name, "white");
        assert_eq!(white.r, 255);
        assert_eq!(white.g, 255);
        assert_eq!(white.b, 255);
        assert_eq!(white.a, 255);
    }

    #[test]
    fn test_color_black() {
        let black = Color::black();
        assert_eq!(black.name, "black");
        assert_eq!(black.r, 0);
        assert_eq!(black.g, 0);
        assert_eq!(black.b, 0);
        assert_eq!(black.a, 255);
    }

    #[test]
    fn test_color_to_float_array() {
        let color = Color::new(255, 128, 64, 255);
        let float_array = color.to_float_array();
        assert_eq!(
            float_array,
            [1.0, 0.5019607843137255, 0.25098039215686274, 1.0]
        );
    }

    #[test]
    fn test_color_from_float() {
        let color = Color::from_float(1.0, 0.5, 0.25, 1.0);
        assert_eq!(color.r, 255);
        assert_eq!(color.g, 128); // 0.5 * 255 = 127.5, rounded to 128 in Rust
        assert_eq!(color.b, 64); // 0.25 * 255 = 63.75, rounded to 64
        assert_eq!(color.a, 255);
    }

    #[test]
    fn test_color_red() {
        let red = Color::red();
        assert_eq!(red.name, "red");
        assert_eq!(red.r, 255);
        assert_eq!(red.g, 0);
        assert_eq!(red.b, 0);
        assert_eq!(red.a, 255);
    }

    #[test]
    fn test_color_green() {
        let green = Color::green();
        assert_eq!(green.name, "green");
        assert_eq!(green.r, 0);
        assert_eq!(green.g, 255);
        assert_eq!(green.b, 0);
        assert_eq!(green.a, 255);
    }

    #[test]
    fn test_color_blue() {
        let blue = Color::blue();
        assert_eq!(blue.name, "blue");
        assert_eq!(blue.r, 0);
        assert_eq!(blue.g, 0);
        assert_eq!(blue.b, 255);
        assert_eq!(blue.a, 255);
    }

    #[test]
    fn test_color_grey() {
        let grey = Color::grey();
        assert_eq!(grey.name, "grey");
        assert_eq!(grey.r, 128);
        assert_eq!(grey.g, 128);
        assert_eq!(grey.b, 128);
        assert_eq!(grey.a, 255);
    }
}

// ============================================================================
// Mini-test framework tests (for cross-language test comparison)
// ============================================================================

use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_color_constructor() -> TestResult {
    MINI_TEST!("constructor", {
        use crate::Color;

        // Constructor
        let mut red = Color::new(255, 0, 0, 255);
        red.name = "red".to_string();

        // Setters
        red.r = 255;
        red.g = 0;
        red.b = 0;
        red.a = 255;

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
        let mut cother = Color::new(255, 0, 0, 255);
        cother.name = "red".to_string();

        MINI_CHECK!(red.name == "red" && !red.guid.is_empty() && red.r == 255 && red.g == 0 && red.b == 0 && red.a == 255);
        MINI_CHECK!(r == 255 && g == 0 && b == 0 && a == 255);
        MINI_CHECK!(cstr == "255, 0, 0, 255");
        MINI_CHECK!(crepr == "Color(red, 255, 0, 0, 255)");
        MINI_CHECK!(ccopy == cother);
        MINI_CHECK!(ccopy.guid != red.guid);
    })
}

pub fn run_color_conversion() -> TestResult {
    MINI_TEST!("conversion", {
        use crate::Color;
        use crate::Tolerance;

        let color = Color::new(255, 128, 64, 255);
        let flts = color.to_float_array();
        let ints = Color::from_float(flts[0], flts[1], flts[2], flts[3]);

        MINI_CHECK!(Tolerance::round_to(flts[0], Tolerance::ROUNDING) == 1.0);
        MINI_CHECK!(Tolerance::round_to(flts[1], Tolerance::ROUNDING) == 0.501961);
        MINI_CHECK!(Tolerance::round_to(flts[2], Tolerance::ROUNDING) == 0.25098);
        MINI_CHECK!(Tolerance::round_to(flts[3], Tolerance::ROUNDING) == 1.0);
        MINI_CHECK!(ints == color);
    })
}

pub fn run_color_presets() -> TestResult {
    MINI_TEST!("presets", {
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

        MINI_CHECK!(white.r == 255 && white.g == 255 && white.b == 255 && white.name == "white");
        MINI_CHECK!(black.r == 0 && black.g == 0 && black.b == 0 && black.name == "black");
        MINI_CHECK!(grey.r == 128 && grey.g == 128 && grey.b == 128 && grey.name == "grey");
        MINI_CHECK!(red.r == 255 && red.g == 0 && red.b == 0 && red.name == "red");
        MINI_CHECK!(orange.r == 255 && orange.g == 128 && orange.b == 0 && orange.name == "orange");
        MINI_CHECK!(yellow.r == 255 && yellow.g == 255 && yellow.b == 0 && yellow.name == "yellow");
        MINI_CHECK!(lime.r == 128 && lime.g == 255 && lime.b == 0 && lime.name == "lime");
        MINI_CHECK!(green.r == 0 && green.g == 255 && green.b == 0 && green.name == "green");
        MINI_CHECK!(mint.r == 0 && mint.g == 255 && mint.b == 128 && mint.name == "mint");
        MINI_CHECK!(cyan.r == 0 && cyan.g == 255 && cyan.b == 255 && cyan.name == "cyan");
        MINI_CHECK!(azure.r == 0 && azure.g == 128 && azure.b == 255 && azure.name == "azure");
        MINI_CHECK!(blue.r == 0 && blue.g == 0 && blue.b == 255 && blue.name == "blue");
        MINI_CHECK!(violet.r == 128 && violet.g == 0 && violet.b == 255 && violet.name == "violet");
        MINI_CHECK!(magenta.r == 255 && magenta.g == 0 && magenta.b == 255 && magenta.name == "magenta");
        MINI_CHECK!(pink.r == 255 && pink.g == 0 && pink.b == 128 && pink.name == "pink");
        MINI_CHECK!(maroon.r == 128 && maroon.g == 0 && maroon.b == 0 && maroon.name == "maroon");
        MINI_CHECK!(brown.r == 128 && brown.g == 64 && brown.b == 0 && brown.name == "brown");
        MINI_CHECK!(olive.r == 128 && olive.g == 128 && olive.b == 0 && olive.name == "olive");
        MINI_CHECK!(teal.r == 0 && teal.g == 128 && teal.b == 128 && teal.name == "teal");
        MINI_CHECK!(navy.r == 0 && navy.g == 0 && navy.b == 128 && navy.name == "navy");
        MINI_CHECK!(purple.r == 128 && purple.g == 0 && purple.b == 128 && purple.name == "purple");
        MINI_CHECK!(silver.r == 192 && silver.g == 192 && silver.b == 192 && silver.name == "silver");
    })
}

pub fn run_color_json_roundtrip() -> TestResult {
    MINI_TEST!("json_roundtrip", {
        use crate::Color;

        let color = Color::with_name(255, 128, 64, 255, "test_color");

        let filename = "test_color.json";
        color.to_json(filename).unwrap();
        let loaded = Color::from_json(filename).unwrap();

        MINI_CHECK!(loaded.name == "test_color");
        MINI_CHECK!(loaded.r == 255);
        MINI_CHECK!(loaded.g == 128);
        MINI_CHECK!(loaded.b == 64);
        MINI_CHECK!(loaded.a == 255);
    })
}

#[cfg(feature = "protobuf")]
pub fn run_color_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("protobuf_roundtrip", {
        use crate::Color;

        let mut color = Color::new(255, 128, 64, 255);
        color.name = "test_color".to_string();

        let filename = "test_color.bin";
        color.protobuf_dump(filename);
        let loaded = Color::protobuf_load(filename);

        MINI_CHECK!(loaded.name == "test_color");
        MINI_CHECK!(loaded.r == 255);
        MINI_CHECK!(loaded.g == 128);
        MINI_CHECK!(loaded.b == 64);
        MINI_CHECK!(loaded.a == 255);
    })
}

// Register tests with the shared registry for run_all("rust")
REGISTER_MINI_TEST!("Color", "constructor", crate::color_test::run_color_constructor);
REGISTER_MINI_TEST!("Color", "conversion", crate::color_test::run_color_conversion);
REGISTER_MINI_TEST!("Color", "presets", crate::color_test::run_color_presets);
REGISTER_MINI_TEST!("Color", "json_roundtrip", crate::color_test::run_color_json_roundtrip);
#[cfg(feature = "protobuf")]
REGISTER_MINI_TEST!("Color", "protobuf_roundtrip", crate::color_test::run_color_protobuf_roundtrip);
