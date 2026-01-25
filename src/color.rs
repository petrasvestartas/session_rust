use serde::{ser::Serialize as SerTrait, Deserialize, Serialize};
use std::fmt;
use std::ops::{Index, IndexMut};
use uuid::Uuid;

/// A color with RGBA values and JSON serialization support.
///
/// The Color struct represents a color using 8-bit RGBA components (0-255).
/// It includes preset colors and conversion utilities for normalized float values.
///
/// # Attributes
///
/// * `guid` - Unique identifier for the color.
/// * `name` - Name of the color.
/// * `r` - Red component (0-255).
/// * `g` - Green component (0-255).
/// * `b` - Blue component (0-255).
/// * `a` - Alpha component (0-255).
///
/// # Examples
///
/// ```
/// # use session_rust::Color;
/// let red = Color::red();
/// let custom = Color::new(128, 64, 32, 255);
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Color")]
pub struct Color {
    pub guid: String,
    pub name: String,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

impl Color {
    /// Create a new color with RGBA values.
    ///
    /// # Arguments
    ///
    /// * `r` - Red component (0-255).
    /// * `g` - Green component (0-255).
    /// * `b` - Blue component (0-255).
    /// * `a` - Alpha component (0-255).
    ///
    /// # Returns
    ///
    /// A new Color with the specified RGBA values and a unique GUID.
    pub fn new(r: u8, g: u8, b: u8, a: u8) -> Self {
        Color {
            guid: Uuid::new_v4().to_string(),
            name: "my_color".to_string(),
            r,
            g,
            b,
            a,
        }
    }

    /// Create a new color with RGBA values and custom name.
    ///
    /// # Arguments
    ///
    /// * `r` - Red component (0-255).
    /// * `g` - Green component (0-255).
    /// * `b` - Blue component (0-255).
    /// * `a` - Alpha component (0-255).
    /// * `name` - Name for the color.
    ///
    /// # Returns
    ///
    /// A new Color with the specified RGBA values, name, and a unique GUID.
    pub fn with_name(r: u8, g: u8, b: u8, a: u8, name: &str) -> Self {
        Color {
            guid: Uuid::new_v4().to_string(),
            name: name.to_string(),
            r,
            g,
            b,
            a,
        }
    }

    /// Duplicate the color (creates a copy with new GUID).
    ///
    /// # Returns
    ///
    /// A new Color with identical RGBA values but a different GUID.
    pub fn duplicate(&self) -> Self {
        Color {
            guid: Uuid::new_v4().to_string(),
            name: self.name.clone(),
            r: self.r,
            g: self.g,
            b: self.b,
            a: self.a,
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Presets
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Create white color.
    pub fn white() -> Self {
        let mut color = Color::new(255, 255, 255, 255);
        color.name = "white".to_string();
        color
    }

    /// Create black color.
    pub fn black() -> Self {
        let mut color = Color::new(0, 0, 0, 255);
        color.name = "black".to_string();
        color
    }

    /// Create grey color.
    pub fn grey() -> Self {
        let mut color = Color::new(128, 128, 128, 255);
        color.name = "grey".to_string();
        color
    }

    /// Create red color.
    pub fn red() -> Self {
        let mut color = Color::new(255, 0, 0, 255);
        color.name = "red".to_string();
        color
    }

    /// Create orange color.
    pub fn orange() -> Self {
        let mut color = Color::new(255, 128, 0, 255);
        color.name = "orange".to_string();
        color
    }

    /// Create yellow color.
    pub fn yellow() -> Self {
        let mut color = Color::new(255, 255, 0, 255);
        color.name = "yellow".to_string();
        color
    }

    /// Create lime color.
    pub fn lime() -> Self {
        let mut color = Color::new(128, 255, 0, 255);
        color.name = "lime".to_string();
        color
    }

    /// Create green color.
    pub fn green() -> Self {
        let mut color = Color::new(0, 255, 0, 255);
        color.name = "green".to_string();
        color
    }

    /// Create mint color.
    pub fn mint() -> Self {
        let mut color = Color::new(0, 255, 128, 255);
        color.name = "mint".to_string();
        color
    }

    /// Create cyan color.
    pub fn cyan() -> Self {
        let mut color = Color::new(0, 255, 255, 255);
        color.name = "cyan".to_string();
        color
    }

    /// Create azure color.
    pub fn azure() -> Self {
        let mut color = Color::new(0, 128, 255, 255);
        color.name = "azure".to_string();
        color
    }

    /// Create blue color.
    pub fn blue() -> Self {
        let mut color = Color::new(0, 0, 255, 255);
        color.name = "blue".to_string();
        color
    }

    /// Create violet color.
    pub fn violet() -> Self {
        let mut color = Color::new(128, 0, 255, 255);
        color.name = "violet".to_string();
        color
    }

    /// Create magenta color.
    pub fn magenta() -> Self {
        let mut color = Color::new(255, 0, 255, 255);
        color.name = "magenta".to_string();
        color
    }

    /// Create pink color.
    pub fn pink() -> Self {
        let mut color = Color::new(255, 0, 128, 255);
        color.name = "pink".to_string();
        color
    }

    /// Create maroon color.
    pub fn maroon() -> Self {
        let mut color = Color::new(128, 0, 0, 255);
        color.name = "maroon".to_string();
        color
    }

    /// Create brown color.
    pub fn brown() -> Self {
        let mut color = Color::new(128, 64, 0, 255);
        color.name = "brown".to_string();
        color
    }

    /// Create olive color.
    pub fn olive() -> Self {
        let mut color = Color::new(128, 128, 0, 255);
        color.name = "olive".to_string();
        color
    }

    /// Create teal color.
    pub fn teal() -> Self {
        let mut color = Color::new(0, 128, 128, 255);
        color.name = "teal".to_string();
        color
    }

    /// Create navy color.
    pub fn navy() -> Self {
        let mut color = Color::new(0, 0, 128, 255);
        color.name = "navy".to_string();
        color
    }

    /// Create purple color.
    pub fn purple() -> Self {
        let mut color = Color::new(128, 0, 128, 255);
        color.name = "purple".to_string();
        color
    }

    /// Create silver color.
    pub fn silver() -> Self {
        let mut color = Color::new(192, 192, 192, 255);
        color.name = "silver".to_string();
        color
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to protobuf binary format.
    ///
    /// # Returns
    ///
    /// A Vec<u8> containing the serialized protobuf data.
    pub fn to_protobuf(&self) -> Vec<u8> {
        use prost::Message;
        
        let proto = crate::proto::Color {
            guid: self.guid.clone(),
            name: self.name.clone(),
            r: self.r as i32,
            g: self.g as i32,
            b: self.b as i32,
            a: self.a as i32,
        };
        proto.encode_to_vec()
    }

    /// Create Color from protobuf binary data.
    ///
    /// # Arguments
    ///
    /// * `data` - Byte slice containing protobuf-encoded color data.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized Color or an error.
    pub fn from_protobuf(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        
        let proto = crate::proto::Color::decode(data)?;
        
        let mut color = Self::new(proto.r as u8, proto.g as u8, proto.b as u8, proto.a as u8);
        color.guid = proto.guid;
        color.name = proto.name;
        Ok(color)
    }

    /// Write protobuf to file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the output file.
    pub fn protobuf_dump(&self, filepath: &str) {
        let data = self.to_protobuf();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    /// Read protobuf from file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the protobuf file.
    ///
    /// # Returns
    ///
    /// The deserialized Color.
    pub fn protobuf_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::from_protobuf(&data).expect("Failed to parse protobuf")
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serialize to JSON string (for cross-language compatibility).
    ///
    /// # Returns
    ///
    /// A Result containing the pretty-printed JSON string or an error.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let mut buf = Vec::new();
        let formatter = serde_json::ser::PrettyFormatter::with_indent(b"    ");
        let mut ser = serde_json::Serializer::with_formatter(&mut buf, formatter);
        SerTrait::serialize(self, &mut ser)?;
        Ok(String::from_utf8(buf)?)
    }

    /// Deserialize from JSON string (for cross-language compatibility).
    ///
    /// # Arguments
    ///
    /// * `json_data` - JSON string containing color data.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized Color or an error.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serialize to JSON file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the output file.
    ///
    /// # Returns
    ///
    /// A Result indicating success or an error.
    pub fn to_json(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    /// Deserialize from JSON file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the JSON file.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized Color or an error.
    pub fn from_json(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json)
    }

    /// Simple string representation (like Python __str__).
    ///
    /// # Returns
    ///
    /// A string in the format "r, g, b, a".
    pub fn str(&self) -> String {
        format!("{}, {}, {}, {}", self.r, self.g, self.b, self.a)
    }

    /// Detailed representation (like Python __repr__).
    ///
    /// # Returns
    ///
    /// A string in the format "Color(name, r, g, b, a)".
    pub fn repr(&self) -> String {
        format!("Color({}, {}, {}, {}, {})", self.name, self.r, self.g, self.b, self.a)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Details
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to normalized float array [0-1].
    ///
    /// # Returns
    ///
    /// An array [r, g, b, a] with values normalized to the range [0.0, 1.0].
    pub fn to_float_array(&self) -> [f64; 4] {
        [
            self.r as f64 / 255.0,
            self.g as f64 / 255.0,
            self.b as f64 / 255.0,
            self.a as f64 / 255.0,
        ]
    }

    /// Create color from normalized float values [0-1].
    ///
    /// # Arguments
    ///
    /// * `r` - Red component (0.0-1.0).
    /// * `g` - Green component (0.0-1.0).
    /// * `b` - Blue component (0.0-1.0).
    /// * `a` - Alpha component (0.0-1.0).
    ///
    /// # Returns
    ///
    /// A new Color with values converted to 0-255 range.
    pub fn from_float(r: f64, g: f64, b: f64, a: f64) -> Self {
        Color::new(
            (r * 255.0).round() as u8,
            (g * 255.0).round() as u8,
            (b * 255.0).round() as u8,
            (a * 255.0).round() as u8,
        )
    }
}

impl Default for Color {
    fn default() -> Self {
        Self::white()
    }
}

impl fmt::Display for Color {
    /// Display format matches Python __str__: "r, g, b, a"
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}, {}, {}, {}", self.r, self.g, self.b, self.a)
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// Index Access (like Python __getitem__ / __setitem__)
///////////////////////////////////////////////////////////////////////////////////////////

impl Index<usize> for Color {
    type Output = u8;

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

#[cfg(test)]
#[path = "color_test.rs"]
mod color_test;
