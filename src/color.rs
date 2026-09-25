use serde::ser::SerializeMap;
use serde::Deserialize;
use serde::Deserializer;
use serde::Serializer;
use std::fmt;
use std::ops::Index;
use std::ops::IndexMut;
use std::sync::OnceLock;

/// A named color with RGBA components in [0.0, 1.0].
#[derive(Debug, Clone)]
pub struct Color {
    guid: OnceLock<String>, // Lazily minted GUID.
    pub name: String,       // Color name.
    pub r: f32,             // Red component.
    pub g: f32,             // Green component.
    pub b: f32,             // Blue component.
    pub a: f32,             // Alpha component.
}

impl Color {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from RGBA components, each clamped to [0.0, 1.0].
    pub fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self::with_name(r, g, b, a, "my_color")
    }

    /// Construct from RGBA components and a name.
    pub fn with_name(r: f32, g: f32, b: f32, a: f32, name: &str) -> Self {
        Color {
            guid: OnceLock::new(),
            name: name.to_string(),
            r: r.clamp(0.0, 1.0),
            g: g.clamp(0.0, 1.0),
            b: b.clamp(0.0, 1.0),
            a: a.clamp(0.0, 1.0),
        }
    }

    /// Copy (new guid, same data).
    pub fn duplicate(&self) -> Self {
        Self::with_name(self.r, self.g, self.b, self.a, &self.name)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy GUID has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the GUID, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid.
    pub fn set_guid(&mut self, guid: String) {
        self.guid = OnceLock::from(guid);
    }
}

impl Default for Color {
    /// Construct the default light grey color.
    fn default() -> Self {
        Self::new(0.94, 0.94, 0.94, 1.0)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
impl Index<usize> for Color {
    type Output = f32;

    /// Component by index (0=r, 1=g, 2=b, 3=a).
    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.r,
            1 => &self.g,
            2 => &self.b,
            3 => &self.a,
            _ => panic!("Index out of range"),
        }
    }
}

impl IndexMut<usize> for Color {
    /// Component by index, mutable.
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.r,
            1 => &mut self.g,
            2 => &mut self.b,
            3 => &mut self.a,
            _ => panic!("Index out of range"),
        }
    }
}

impl PartialEq for Color {
    /// Compare names and RGBA components.
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && self.r == other.r
            && self.g == other.g
            && self.b == other.b
            && self.a == other.a
    }
}

impl Color {
    // ═══════════════════════════════════════════════════════════════════════════
    // Presets
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return opaque white.
    pub fn white() -> Self {
        Self::with_name(1.0, 1.0, 1.0, 1.0, "white")
    }

    /// Return opaque black.
    pub fn black() -> Self {
        Self::with_name(0.0, 0.0, 0.0, 1.0, "black")
    }

    /// Return opaque grey.
    pub fn grey() -> Self {
        Self::with_name(0.5, 0.5, 0.5, 1.0, "grey")
    }

    /// Return opaque red.
    pub fn red() -> Self {
        Self::with_name(1.0, 0.0, 0.0, 1.0, "red")
    }

    /// Return opaque orange.
    pub fn orange() -> Self {
        Self::with_name(1.0, 0.5, 0.0, 1.0, "orange")
    }

    /// Return opaque yellow.
    pub fn yellow() -> Self {
        Self::with_name(1.0, 1.0, 0.0, 1.0, "yellow")
    }

    /// Return opaque lime.
    pub fn lime() -> Self {
        Self::with_name(0.5, 1.0, 0.0, 1.0, "lime")
    }

    /// Return opaque green.
    pub fn green() -> Self {
        Self::with_name(0.0, 1.0, 0.0, 1.0, "green")
    }

    /// Return opaque mint.
    pub fn mint() -> Self {
        Self::with_name(0.0, 1.0, 0.5, 1.0, "mint")
    }

    /// Return opaque cyan.
    pub fn cyan() -> Self {
        Self::with_name(0.0, 1.0, 1.0, 1.0, "cyan")
    }

    /// Return opaque azure.
    pub fn azure() -> Self {
        Self::with_name(0.0, 0.5, 1.0, 1.0, "azure")
    }

    /// Return opaque blue.
    pub fn blue() -> Self {
        Self::with_name(0.0, 0.0, 1.0, 1.0, "blue")
    }

    /// Return opaque violet.
    pub fn violet() -> Self {
        Self::with_name(0.5, 0.0, 1.0, 1.0, "violet")
    }

    /// Return opaque magenta.
    pub fn magenta() -> Self {
        Self::with_name(1.0, 0.0, 1.0, 1.0, "magenta")
    }

    /// Return opaque pink.
    pub fn pink() -> Self {
        Self::with_name(1.0, 0.0, 0.5, 1.0, "pink")
    }

    /// Return opaque maroon.
    pub fn maroon() -> Self {
        Self::with_name(0.5, 0.0, 0.0, 1.0, "maroon")
    }

    /// Return opaque brown.
    pub fn brown() -> Self {
        Self::with_name(0.5, 0.25, 0.0, 1.0, "brown")
    }

    /// Return opaque olive.
    pub fn olive() -> Self {
        Self::with_name(0.5, 0.5, 0.0, 1.0, "olive")
    }

    /// Return opaque teal.
    pub fn teal() -> Self {
        Self::with_name(0.0, 0.5, 0.5, 1.0, "teal")
    }

    /// Return opaque navy.
    pub fn navy() -> Self {
        Self::with_name(0.0, 0.0, 0.5, 1.0, "navy")
    }

    /// Return opaque purple.
    pub fn purple() -> Self {
        Self::with_name(0.5, 0.0, 0.5, 1.0, "purple")
    }

    /// Return opaque silver.
    pub fn silver() -> Self {
        Self::with_name(0.75, 0.75, 0.75, 1.0, "silver")
    }

    /// Return opaque light grey, the default surface color of meshes, breps and surfaces.
    pub fn lightgrey() -> Self {
        Self::with_name(0.94, 0.94, 0.94, 1.0, "lightgrey")
    }

    /// Return the 12 spectral colors in order.
    pub fn palette() -> Vec<Color> {
        vec![
            Self::red(),
            Self::orange(),
            Self::yellow(),
            Self::lime(),
            Self::green(),
            Self::mint(),
            Self::cyan(),
            Self::azure(),
            Self::blue(),
            Self::violet(),
            Self::magenta(),
            Self::pink(),
        ]
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Conversion
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the components as [r, g, b, a].
    pub fn to_unified_array(&self) -> [f32; 4] {
        [self.r, self.g, self.b, self.a]
    }

    /// Construct from [r, g, b, a].
    pub fn from_unified_array(arr: [f32; 4]) -> Self {
        Self::new(arr[0], arr[1], arr[2], arr[3])
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a sorted JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().expect("Failed to serialize Color JSON")
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).expect("Failed to parse Color JSON")
    }

    /// Write JSON to a file.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.jsondump()?)?;

        Ok(())
    }

    /// Read JSON from a file.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::Color {
        crate::proto::Color {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            r: self.r,
            g: self.g,
            b: self.b,
            a: self.a,
        }
    }

    /// Construct from the protobuf message.
    pub fn from_proto(proto: crate::proto::Color) -> Self {
        let mut color = Self::with_name(proto.r, proto.g, proto.b, proto.a, &proto.name);

        if !proto.guid.is_empty() {
            color.set_guid(proto.guid);
        }

        color
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Ok(Self::from_proto(crate::proto::Color::decode(data)?))
    }

    /// Write protobuf bytes to a file.
    pub fn pb_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.pb_dumps())?;

        Ok(())
    }

    /// Read protobuf bytes from a file.
    pub fn pb_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::pb_loads(&std::fs::read(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "r, g, b, a".
    pub fn str(&self) -> String {
        format!("{:.1}, {:.1}, {:.1}, {:.1}", self.r, self.g, self.b, self.a)
    }

    /// Return "Color(name, r, g, b, a)".
    pub fn repr(&self) -> String {
        format!(
            "Color({}, {:.1}, {:.1}, {:.1}, {:.1})",
            self.name, self.r, self.g, self.b, self.a
        )
    }
}

impl fmt::Display for Color {
    /// Write the string representation to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Serde
// ═══════════════════════════════════════════════════════════════════════════
impl serde::Serialize for Color {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut map = serializer.serialize_map(Some(7))?;
        map.serialize_entry("a", &self.a)?;
        map.serialize_entry("b", &self.b)?;
        map.serialize_entry("g", &self.g)?;
        map.serialize_entry("guid", self.guid())?;
        map.serialize_entry("name", &self.name)?;
        map.serialize_entry("r", &self.r)?;
        map.serialize_entry("type", "Color")?;

        map.end()
    }
}

impl<'de> Deserialize<'de> for Color {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct ColorData {
            a: f32,
            b: f32,
            g: f32,
            guid: String,
            name: String,
            r: f32,
        }

        let data = ColorData::deserialize(deserializer)?;
        let mut color = Color::with_name(data.r, data.g, data.b, data.a, &data.name);
        color.set_guid(data.guid);

        Ok(color)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SESSION_VIEWER
// ═══════════════════════════════════════════════════════════════════════════
impl Color {
    /// GPU-ready `[r, g, b, a]` for wgpu upload; mirrors [`crate::Xform::to_f32`]
    pub fn to_f32(&self) -> [f32; 4] {
        [self.r, self.g, self.b, self.a]
    }

    /// GPU-ready `[r, g, b]` for shaders that light a `vec3` base color
    pub fn to_rgb(&self) -> [f32; 3] {
        [self.r, self.g, self.b]
    }
}
