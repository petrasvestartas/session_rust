use serde::ser::SerializeMap;
use serde::{Deserialize, Deserializer, Serializer};
use std::fmt;
use std::ops::{Index, IndexMut};

/// A color with RGBA values in the range [0.0, 1.0].
#[derive(Debug, Clone)]
pub struct Color {
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

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
            #[serde(default)]
            guid: Option<String>,
            #[serde(default)]
            name: Option<String>,
            #[serde(default)]
            r: f32,
            #[serde(default)]
            g: f32,
            #[serde(default)]
            b: f32,
            #[serde(default = "default_alpha")]
            a: f32,
        }
        fn default_alpha() -> f32 {
            1.0
        }
        let data = ColorData::deserialize(deserializer)?;
        let c = Color {
            guid: std::sync::OnceLock::new(),
            name: data.name.unwrap_or_else(|| "my_color".to_string()),
            r: data.r.clamp(0.0, 1.0),
            g: data.g.clamp(0.0, 1.0),
            b: data.b.clamp(0.0, 1.0),
            a: data.a.clamp(0.0, 1.0),
        };
        if let Some(g) = data.guid {
            c.set_guid(g);
        }
        Ok(c)
    }
}

impl Color {
    /// Create a new color with RGBA values in range [0.0, 1.0].
    pub fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Color {
            guid: std::sync::OnceLock::new(),
            name: "my_color".to_string(),
            r: r.clamp(0.0, 1.0),
            g: g.clamp(0.0, 1.0),
            b: b.clamp(0.0, 1.0),
            a: a.clamp(0.0, 1.0),
        }
    }

    /// P6: a bulk colour list as packed floats, 4 per colour (r, g, b, a).
    ///
    /// ONE implementation for every type that carries bulk colours (Mesh, Line, NurbsCurve,
    /// NurbsSurface). Do not copy this into a geometry module: the whole point of P6 is that
    /// serialization stops being per-type mapping code.
    ///
    /// No guid (equality ignores it; it is lazily minted, not identity) and no name (bulk
    /// colours always carry the default). A single nameable colour — a Line's — stores its
    /// name in a sibling string field that costs nothing when empty.
    pub fn pack(colors: &[Color]) -> Vec<f32> {
        let mut packed = Vec::with_capacity(colors.len() * 4);
        for c in colors {
            packed.extend_from_slice(&[c.r, c.g, c.b, c.a]);
        }
        packed
    }

    /// P6: rebuild a bulk colour list from the packed array.
    pub fn unpack(packed: &[f32]) -> Vec<Color> {
        packed
            .chunks_exact(4)
            .map(|c| Color::new(c[0], c[1], c[2], c[3]))
            .collect()
    }

    /// Create a new color with RGBA values and custom name.
    pub fn with_name(r: f32, g: f32, b: f32, a: f32, name: &str) -> Self {
        Color {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            r: r.clamp(0.0, 1.0),
            g: g.clamp(0.0, 1.0),
            b: b.clamp(0.0, 1.0),
            a: a.clamp(0.0, 1.0),
        }
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Duplicate the color (creates a copy with new GUID).
    pub fn duplicate(&self) -> Self {
        Color {
            guid: std::sync::OnceLock::new(),
            name: self.name.clone(),
            r: self.r,
            g: self.g,
            b: self.b,
            a: self.a,
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Presets
    // ═══════════════════════════════════════════════════════════════════════════

    /// Create white color.
    pub fn white() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(1.0, 1.0, 1.0, 1.0);
            c.name = "white".to_string();
            c
        })
        .clone()
    }

    /// Create black color.
    pub fn black() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.0, 0.0, 0.0, 1.0);
            c.name = "black".to_string();
            c
        })
        .clone()
    }

    /// Create grey color.
    pub fn grey() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.5, 0.5, 0.5, 1.0);
            c.name = "grey".to_string();
            c
        })
        .clone()
    }

    /// Create red color.
    pub fn red() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(1.0, 0.0, 0.0, 1.0);
            c.name = "red".to_string();
            c
        })
        .clone()
    }

    /// Create orange color.
    pub fn orange() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(1.0, 0.5, 0.0, 1.0);
            c.name = "orange".to_string();
            c
        })
        .clone()
    }

    /// Create yellow color.
    pub fn yellow() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(1.0, 1.0, 0.0, 1.0);
            c.name = "yellow".to_string();
            c
        })
        .clone()
    }

    /// Create lime color.
    pub fn lime() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.5, 1.0, 0.0, 1.0);
            c.name = "lime".to_string();
            c
        })
        .clone()
    }

    /// Create green color.
    pub fn green() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.0, 1.0, 0.0, 1.0);
            c.name = "green".to_string();
            c
        })
        .clone()
    }

    /// Create mint color.
    pub fn mint() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.0, 1.0, 0.5, 1.0);
            c.name = "mint".to_string();
            c
        })
        .clone()
    }

    /// Create cyan color.
    pub fn cyan() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.0, 1.0, 1.0, 1.0);
            c.name = "cyan".to_string();
            c
        })
        .clone()
    }

    /// Create azure color.
    pub fn azure() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.0, 0.5, 1.0, 1.0);
            c.name = "azure".to_string();
            c
        })
        .clone()
    }

    /// Create blue color.
    pub fn blue() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.0, 0.0, 1.0, 1.0);
            c.name = "blue".to_string();
            c
        })
        .clone()
    }

    /// Create violet color.
    pub fn violet() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.5, 0.0, 1.0, 1.0);
            c.name = "violet".to_string();
            c
        })
        .clone()
    }

    /// Create magenta color.
    pub fn magenta() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(1.0, 0.0, 1.0, 1.0);
            c.name = "magenta".to_string();
            c
        })
        .clone()
    }

    /// Create pink color.
    pub fn pink() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(1.0, 0.0, 0.5, 1.0);
            c.name = "pink".to_string();
            c
        })
        .clone()
    }

    /// Create maroon color.
    pub fn maroon() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.5, 0.0, 0.0, 1.0);
            c.name = "maroon".to_string();
            c
        })
        .clone()
    }

    /// Create brown color.
    pub fn brown() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.5, 0.25, 0.0, 1.0);
            c.name = "brown".to_string();
            c
        })
        .clone()
    }

    /// Create olive color.
    pub fn olive() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.5, 0.5, 0.0, 1.0);
            c.name = "olive".to_string();
            c
        })
        .clone()
    }

    /// Create teal color.
    pub fn teal() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.0, 0.5, 0.5, 1.0);
            c.name = "teal".to_string();
            c
        })
        .clone()
    }

    /// Create navy color.
    pub fn navy() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.0, 0.0, 0.5, 1.0);
            c.name = "navy".to_string();
            c
        })
        .clone()
    }

    /// Create purple color.
    pub fn purple() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.5, 0.0, 0.5, 1.0);
            c.name = "purple".to_string();
            c
        })
        .clone()
    }

    /// Create silver color.
    pub fn silver() -> Self {
        use std::sync::OnceLock;
        static C: OnceLock<Color> = OnceLock::new();
        C.get_or_init(|| {
            let mut c = Color::new(0.75, 0.75, 0.75, 1.0);
            c.name = "silver".to_string();
            c
        })
        .clone()
    }

    /// Return a palette of 12 spectral colors in order.
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
    // Protobuf Serialization
    // ═══════════════════════════════════════════════════════════════════════════

    /// Convert to protobuf binary format.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        let proto = crate::proto::Color {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            r: self.r,
            g: self.g,
            b: self.b,
            a: self.a,
        };
        proto.encode_to_vec()
    }

    /// Create Color from protobuf binary data.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Color::decode(data)?;
        let mut color = Self::new(proto.r, proto.g, proto.b, proto.a);
        color.set_guid(proto.guid);
        color.name = proto.name;
        Ok(color)
    }

    /// Write protobuf to file.
    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    /// Read protobuf from file.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════

    /// Serialize to JSON string (for cross-language compatibility).
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from JSON string (for cross-language compatibility).
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_else(|_| Self::default())
    }

    /// Serialize to JSON file.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    /// Deserialize from JSON file.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json)
    }

    /// Simple string representation: "r, g, b, a" with one decimal place.
    pub fn str(&self) -> String {
        format!("{:.1}, {:.1}, {:.1}, {:.1}", self.r, self.g, self.b, self.a)
    }

    /// Detailed representation: "Color(name, r, g, b, a)".
    pub fn repr(&self) -> String {
        format!(
            "Color({}, {:.1}, {:.1}, {:.1}, {:.1})",
            self.name, self.r, self.g, self.b, self.a
        )
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Details
    // ═══════════════════════════════════════════════════════════════════════════

    /// Return RGBA as [r, g, b, a] float array (values already in [0.0, 1.0]).
    pub fn to_float_array(&self) -> [f32; 4] {
        [self.r, self.g, self.b, self.a]
    }

    /// Create color from float values [0.0, 1.0].
    pub fn from_float(r: f32, g: f32, b: f32, a: f32) -> Self {
        Color::new(r, g, b, a)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // WGPU
    // ═══════════════════════════════════════════════════════════════════════════

    /// GPU-ready `[r, g, b, a]` as f32 — the color→GPU boundary for wgpu upload
    /// (`bytemuck::cast_slice`, instance/segment rows). Mirrors [`crate::Xform::to_f32`].
    /// Components are already f32, so this just drops the guid/name and packs the four.
    pub fn to_f32(&self) -> [f32; 4] {
        [self.r, self.g, self.b, self.a]
    }

    /// GPU-ready `[r, g, b]` as f32 — the opaque RGB prefix (drops alpha), for shaders
    /// that light a `vec3` base color.
    pub fn to_rgb(&self) -> [f32; 3] {
        [self.r, self.g, self.b]
    }
}

impl Default for Color {
    fn default() -> Self {
        Self::white()
    }
}

impl fmt::Display for Color {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:.1}, {:.1}, {:.1}, {:.1}",
            self.r, self.g, self.b, self.a
        )
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Index Access (like Python __getitem__ / __setitem__)
// ═══════════════════════════════════════════════════════════════════════════

impl Index<usize> for Color {
    type Output = f32;

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
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && self.r == other.r
            && self.g == other.g
            && self.b == other.b
            && self.a == other.a
    }
}
