use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;
use crate::mini_test::TestResult;

pub fn run_text_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Color;
        use crate::Point;
        use crate::Text;

        let mut text = Text::new("n_0", Point::new(1.0, 2.0, 3.0));
        text.name = "n0".to_string();
        text.math = true;
        text.spatial = true;
        text.outline = -1.5;
        let copy = text.duplicate();
        let other = Text::default();
        let spatial = Text::from_tex("n_0", Point::new(1.0, 2.0, 3.0), true);
        let flat = Text::from_tex("x_0", Point::new(0.0, 0.0, 0.0), false);
        let string = text.str();
        let representation = text.repr();

        MINI_CHECK!(text.name == "n0" && !text.guid().is_empty());
        MINI_CHECK!(text.at == Point::new(1.0, 2.0, 3.0));
        MINI_CHECK!(text.height == 28.0 && text.color == Color::white());
        MINI_CHECK!(text.math && text.spatial && !text.serif);
        MINI_CHECK!(copy == text && copy.guid() != text.guid());
        MINI_CHECK!(other != text);
        MINI_CHECK!(
            spatial.math && spatial.height == 18.0 && spatial.outline == -1.5 && !spatial.serif
        );
        MINI_CHECK!(
            flat.math && !flat.spatial && flat.height == 28.0 && flat.outline == 0.0 && flat.serif
        );
        MINI_CHECK!(string == "n_0");
        MINI_CHECK!(representation.contains("Text("));
    })
}

pub fn run_text_json() -> TestResult {
    MINI_TEST!("Json", {
        use crate::Point;
        use crate::Text;

        let mut text = Text::new("q=\\frac{f}{l}", Point::new(4.0, 5.0, 6.0));
        text.math = true;
        text.serif = true;
        let filename = "text_test.json";
        let loaded = Text::file_json_loads(&text.file_json_dumps());
        text.file_json_dump(filename).unwrap();
        let from_file = Text::file_json_load(filename).unwrap();
        std::fs::remove_file(filename).unwrap();

        MINI_CHECK!(loaded == text);
        MINI_CHECK!(from_file == text);
    })
}

pub fn run_text_protobuf() -> TestResult {
    MINI_TEST!("Protobuf", {
        use crate::Point;
        use crate::Text;

        let mut text = Text::new("x_0", Point::new(7.0, 8.0, 9.0));
        text.math = true;
        text.spatial = true;
        text.outline = -1.5;
        let filename = "text_test.pb";
        let loaded = Text::pb_loads(&text.pb_dumps()).unwrap();
        text.pb_dump(filename).unwrap();
        let from_file = Text::pb_load(filename).unwrap();
        std::fs::remove_file(filename).unwrap();

        MINI_CHECK!(loaded == text);
        MINI_CHECK!(from_file == text);
    })
}

pub fn run_text_component() -> TestResult {
    MINI_TEST!("Component", {
        use crate::Point;
        use crate::Session;
        use crate::Text;

        let mut text = Text::new("C[0,1]", Point::new(10.0, 11.0, 0.0));
        text.math = true;
        let component = text.to_component();
        let loaded = Text::from_component(&component).unwrap();
        let mut session = Session::new("text");
        session.add_text(&text, None);
        let restored = Session::pb_loads(&session.pb_dumps()).unwrap();

        MINI_CHECK!(component.type_name == "Text");
        MINI_CHECK!(component.guid == text.guid());
        MINI_CHECK!(loaded == text);
        MINI_CHECK!(session.objects.components.len() == 1);
        MINI_CHECK!(session.objects.components[0].type_name == "Text");
        MINI_CHECK!(Text::from_component(&restored.objects.components[0]).unwrap() == text);
    })
}

REGISTER_MINI_TEST!(
    "Text",
    "Constructor",
    crate::text_test::run_text_constructor
);
REGISTER_MINI_TEST!("Text", "Json", crate::text_test::run_text_json);
REGISTER_MINI_TEST!("Text", "Protobuf", crate::text_test::run_text_protobuf);
REGISTER_MINI_TEST!("Text", "Component", crate::text_test::run_text_component);
