use crate::tolerance::Tolerance;
use crate::tolerance::TOLERANCE;
use crate::{Color, Xform};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Index, IndexMut};
use std::sync::OnceLock;

/// A block reference: places a definition (by guid) at a transform.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "InstanceRef")]
pub struct InstanceRef {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: OnceLock<String>,
    pub name: String,            // Instance name.
    pub definition_guid: String, // Guid of the referenced definition.
    pub xform: Xform,            // Placement transform.
    pub color: Color,            // Display color.
    pub flags: u32,              // Instance flags.
}

impl Default for InstanceRef {
    fn default() -> Self {
        Self {
            guid: OnceLock::new(),
            name: "my_instance_ref".to_string(),
            definition_guid: String::new(),
            xform: Xform::identity(),
            color: Color::white(),
            flags: 0,
        }
    }
}

impl InstanceRef {
    /// Constructs from a definition guid and a placement.
    pub fn new(definition_guid: &str, xform: Xform) -> Self {
        Self {
            definition_guid: definition_guid.to_string(),
            xform,
            ..Default::default()
        }
    }

    /// Copies with a new guid and the same data.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = OnceLock::new();

        copy
    }

    /// Returns whether the lazy guid has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Returns the guid, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Sets the guid if it has not already been created.
    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Constructs from a name, a definition guid and a placement.
    pub fn with_name(name: &str, definition_guid: &str, xform: Xform) -> Self {
        let mut ref_ = Self::new(definition_guid, xform);
        ref_.name = name.to_string();

        ref_
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Composes in place: xform = t * xform.
    pub fn transform(&mut self, t: &Xform) {
        self.xform = t * &self.xform;
    }

    /// Returns a composed copy.
    pub fn transformed(&self, t: &Xform) -> Self {
        let mut result = self.duplicate();
        result.transform(t);

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════

    /// Serializes to a JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserializes from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serializes to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    /// Deserializes from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_default()
    }

    /// Writes to a JSON file.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.jsondump()?)?;

        Ok(())
    }

    /// Reads from a JSON file.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════

    /// Serializes to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        let proto = crate::proto::InstanceRef {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            definition_guid: self.definition_guid.clone(),
            xform: Some(crate::proto::Xform {
                guid: String::new(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
            color: Some(crate::proto::Color {
                guid: String::new(),
                name: self.color.name.clone(),
                r: self.color.r,
                g: self.color.g,
                b: self.color.b,
                a: self.color.a,
            }),
            flags: self.flags,
        };

        proto.encode_to_vec()
    }

    /// Deserializes from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::InstanceRef::decode(data)?;
        let mut ref_ = Self::default();

        if !proto.guid.is_empty() {
            ref_.set_guid(proto.guid);
        }

        ref_.name = proto.name;
        ref_.definition_guid = proto.definition_guid;

        if let Some(proto_xform) = proto.xform {
            ref_.xform.name = proto_xform.name;

            for i in 0..proto_xform.matrix.len().min(16) {
                ref_.xform.m[i] = proto_xform.matrix[i];
            }
        }

        if let Some(proto_color) = proto.color {
            ref_.color.name = proto_color.name;
            ref_.color.r = proto_color.r;
            ref_.color.g = proto_color.g;
            ref_.color.b = proto_color.b;
            ref_.color.a = proto_color.a;
        }

        ref_.flags = proto.flags;

        Ok(ref_)
    }

    /// Writes to a protobuf file.
    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    /// Reads from a protobuf file.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");

        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════

    /// "definition_guid @ [tx, ty, tz]"
    pub fn str(&self) -> String {
        let prec = Tolerance::ROUNDING;
        format!(
            "{} @ [{}, {}, {}]",
            self.definition_guid,
            TOLERANCE.format_number(self.xform.m[12], prec),
            TOLERANCE.format_number(self.xform.m[13], prec),
            TOLERANCE.format_number(self.xform.m[14], prec)
        )
    }

    /// Returns "InstanceRef(name, definition_guid, Color(...), flags)".
    pub fn repr(&self) -> String {
        format!(
            "InstanceRef({}, {}, {}, {})",
            self.name,
            self.definition_guid,
            self.color.repr(),
            self.flags
        )
    }
}

impl fmt::Display for InstanceRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

impl Index<usize> for InstanceRef {
    type Output = f64;

    /// Placement matrix entry by index (0..15, column-major)
    fn index(&self, index: usize) -> &Self::Output {
        if index >= 16 {
            panic!("Index out of bounds");
        }

        &self.xform.m[index]
    }
}

impl IndexMut<usize> for InstanceRef {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        if index >= 16 {
            panic!("Index out of bounds");
        }

        &mut self.xform.m[index]
    }
}

impl PartialEq for InstanceRef {
    /// Same definition, placement, color and flags; guid and name ignored
    fn eq(&self, other: &Self) -> bool {
        self.definition_guid == other.definition_guid
            && self.xform == other.xform
            && self.color == other.color
            && self.flags == other.flags
    }
}
