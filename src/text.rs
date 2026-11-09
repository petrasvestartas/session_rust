use crate::Color;
use crate::Point;
use crate::objects::Component;
use serde::Deserialize;
use serde::Serialize;
use serde::Serializer;
use std::fmt;
use std::sync::OnceLock;

/// Text or TeX with a position, pixel height, colour, outline and projection mode.
#[derive(Clone)]
pub struct Text {
    guid: OnceLock<String>, // Lazily minted GUID.
    pub name: String,       // Text name.
    pub text: String,       // Plain text or TeX source.
    pub at: Point,          // Position in the text coordinate system.
    pub height: f64,        // Letter height in CSS pixels.
    pub color: Color,       // Text colour.
    pub outline: f64,       // Glyph outline width in CSS pixels; negative is black, positive white.
    pub math: bool,         // Whether text contains TeX.
    pub spatial: bool,      // Whether the text belongs to the navigable 3D region.
    pub serif: bool,        // Whether to use the serif mathematics face.
}

impl Default for Text {
    fn default() -> Self {
        Self::new("", Point::default())
    }
}

impl Text {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct text at a position.
    pub fn new(text: &str, at: Point) -> Self {
        Self {
            guid: OnceLock::new(),
            name: "my_text".to_string(),
            text: text.to_string(),
            at,
            height: 28.0,
            color: Color::white(),
            outline: 0.0,
            math: false,
            spatial: false,
            serif: false,
        }
    }

    /// Construct TeX with flat-panel or fixed-size spatial defaults.
    pub fn from_tex(text: &str, at: Point, spatial: bool) -> Self {
        let mut result = Self::new(text, at);
        result.math = true;
        result.spatial = spatial;
        result.height = if spatial { 18.0 } else { 28.0 };
        result.outline = if spatial { -1.5 } else { 0.0 };
        result.serif = !spatial;

        result
    }

    /// Copy with a new guid and the same data.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = OnceLock::new();

        copy
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the guid, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid.
    pub fn set_guid(&mut self, guid: String) {
        self.guid = OnceLock::from(guid);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a sorted JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::file_json_dumps(self, false)
    }

    /// Deserialize from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().expect("Failed to serialize Text JSON")
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).expect("Failed to parse Text JSON")
    }

    /// Write JSON to a file.
    pub fn file_json_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        crate::file_encoders::file_json_dump(self, filename, true)
    }

    /// Read JSON from a file.
    pub fn file_json_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filename)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the generic component protobuf message.
    pub fn to_proto(&self) -> crate::proto::Component {
        self.to_component().to_proto()
    }

    /// Construct from a generic component protobuf message.
    pub fn from_proto(proto: crate::proto::Component) -> Result<Self, Box<dyn std::error::Error>> {
        Self::from_component(&Component::from_proto(proto))
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Self::from_proto(crate::proto::Component::decode(data)?)
    }

    /// Write protobuf bytes to a file.
    pub fn pb_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filename, self.pb_dumps())?;

        Ok(())
    }

    /// Read protobuf bytes from a file.
    pub fn pb_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::pb_loads(&std::fs::read(filename)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Component
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a Session component of type Text with this guid and name.
    pub fn to_component(&self) -> Component {
        let mut extra: std::collections::HashMap<String, serde_json::Value> =
            serde_json::from_value(serde_json::to_value(self).unwrap_or_default())
                .unwrap_or_default();
        extra.remove("guid");
        extra.remove("name");
        extra.remove("type");

        Component {
            guid: self.guid().to_string(),
            type_name: "Text".to_string(),
            name: self.name.clone(),
            extra,
        }
    }

    /// Construct from a component made by to_component.
    pub fn from_component(component: &Component) -> Result<Self, Box<dyn std::error::Error>> {
        let mut data = serde_json::Map::new();

        for (key, value) in &component.extra {
            data.insert(key.clone(), value.clone());
        }

        data.insert("guid".to_string(), serde_json::json!(component.guid));
        data.insert("name".to_string(), serde_json::json!(component.name));

        Ok(serde_json::from_value(serde_json::Value::Object(data))?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the text content.
    pub fn str(&self) -> String {
        self.text.clone()
    }

    /// Return a representation with every property.
    pub fn repr(&self) -> String {
        format!("Text({})", self.file_json_dumps())
    }
}

impl PartialEq for Text {
    /// Compare every text property, ignoring the guid.
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && self.text == other.text
            && self.at == other.at
            && self.height == other.height
            && self.color == other.color
            && self.outline == other.outline
            && self.math == other.math
            && self.spatial == other.spatial
            && self.serif == other.serif
    }
}

impl fmt::Display for Text {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

impl fmt::Debug for Text {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.repr())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Serde
// ═══════════════════════════════════════════════════════════════════════════
impl Serialize for Text {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        use serde::ser::SerializeMap;

        let mut map = serializer.serialize_map(Some(11))?;
        map.serialize_entry("at", &self.at)?;
        map.serialize_entry("color", &self.color)?;
        map.serialize_entry("guid", self.guid())?;
        map.serialize_entry("height", &self.height)?;
        map.serialize_entry("math", &self.math)?;
        map.serialize_entry("name", &self.name)?;
        map.serialize_entry("outline", &self.outline)?;
        map.serialize_entry("serif", &self.serif)?;
        map.serialize_entry("spatial", &self.spatial)?;
        map.serialize_entry("text", &self.text)?;
        map.serialize_entry("type", "Text")?;
        map.end()
    }
}

impl<'de> Deserialize<'de> for Text {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct TextData {
            at: Option<Point>,
            color: Option<Color>,
            guid: String,
            height: Option<f64>,
            math: Option<bool>,
            name: Option<String>,
            outline: Option<f64>,
            serif: Option<bool>,
            spatial: Option<bool>,
            text: Option<String>,
        }

        let d = TextData::deserialize(deserializer)?;
        let mut text = Text::new(&d.text.unwrap_or_default(), d.at.unwrap_or_default());

        if !d.guid.is_empty() {
            text.set_guid(d.guid);
        }

        if let Some(name) = d.name {
            text.name = name;
        }

        if let Some(height) = d.height {
            text.height = height;
        }

        if let Some(color) = d.color {
            text.color = color;
        }

        if let Some(outline) = d.outline {
            text.outline = outline;
        }

        if let Some(math) = d.math {
            text.math = math;
        }

        if let Some(spatial) = d.spatial {
            text.spatial = spatial;
        }

        if let Some(serif) = d.serif {
            text.serif = serif;
        }

        Ok(text)
    }
}
