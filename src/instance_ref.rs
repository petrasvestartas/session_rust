use crate::element::ElementFeature;
use crate::tolerance::Tolerance;
use crate::tolerance::TOLERANCE;
use crate::Color;
use crate::Xform;
use serde::Deserialize;
use serde::Serialize;
use std::fmt;
use std::ops::Index;
use std::ops::IndexMut;
use std::sync::OnceLock;

/// Write features as their JSON objects.
fn serialize_features<S: serde::Serializer>(
    features: &[ElementFeature],
    serializer: S,
) -> Result<S::Ok, S::Error> {
    serializer.collect_seq(features.iter().map(ElementFeature::jsondump))
}

/// Read features from their JSON objects.
fn deserialize_features<'de, D: serde::Deserializer<'de>>(
    deserializer: D,
) -> Result<Vec<ElementFeature>, D::Error> {
    let values: Vec<serde_json::Value> = Deserialize::deserialize(deserializer)?;

    Ok(values.iter().map(ElementFeature::jsonload).collect())
}

/// A block reference: places a definition (by guid) at a transform.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "InstanceRef")]
pub struct InstanceRef {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: OnceLock<String>, // Lazily minted GUID.
    pub name: String,            // Instance name.
    pub definition_guid: String, // Guid of the referenced definition.
    pub xform: Xform, // Placement outside a Session; inside one Session::xforms places the instance and this stays identity.
    pub color: Color, // Display color, used when flags has FLAG_COLOR.
    #[serde(default)]
    pub flags: u32, // FLAG_* bits.
    #[serde(
        default,
        serialize_with = "serialize_features",
        deserialize_with = "deserialize_features"
    )]
    pub features: Vec<ElementFeature>, // Per-instance features in the definition frame, drawn after the definition's own.
}

impl InstanceRef {
    pub const FLAG_HIDDEN: u32 = 1; // Not drawn.
    pub const FLAG_LOCKED: u32 = 2; // Not selectable.
    pub const FLAG_COLOR: u32 = 4; // color overrides the definition's.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from a definition guid and a placement.
    pub fn new(definition_guid: &str, xform: Xform) -> Self {
        Self {
            definition_guid: definition_guid.to_string(),
            xform,
            ..Default::default()
        }
    }

    /// Copy with a new guid and the same data, each feature with a new guid too.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = OnceLock::new();

        for feature in &mut copy.features {
            feature.refresh_guid();
        }

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
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from a name, a definition guid and a placement.
    pub fn with_name(name: &str, definition_guid: &str, xform: Xform) -> Self {
        let mut ref_ = Self::new(definition_guid, xform);
        ref_.name = name.to_string();

        ref_
    }
}

impl Default for InstanceRef {
    /// Construct an empty reference.
    fn default() -> Self {
        Self {
            guid: OnceLock::new(),
            name: "my_instance_ref".to_string(),
            definition_guid: String::new(),
            xform: Xform::identity(),
            color: Color::white(),
            flags: 0,
            features: Vec::new(),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
impl Index<usize> for InstanceRef {
    type Output = f64;

    /// Return the placement matrix entry by index (0..15, column-major).
    fn index(&self, index: usize) -> &Self::Output {
        if index >= 16 {
            panic!("Index out of bounds");
        }

        &self.xform.m[index]
    }
}

impl IndexMut<usize> for InstanceRef {
    /// Return the mutable placement matrix entry by index (0..15, column-major).
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        if index >= 16 {
            panic!("Index out of bounds");
        }

        &mut self.xform.m[index]
    }
}

impl PartialEq for InstanceRef {
    /// Compare definition guid, placement, color, flags and features.
    fn eq(&self, other: &Self) -> bool {
        self.definition_guid == other.definition_guid
            && self.xform == other.xform
            && self.color == other.color
            && self.flags == other.flags
            && self.features == other.features
    }
}

impl InstanceRef {
    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compose in place: xform = t * xform.
    pub fn transform(&mut self, t: &Xform) {
        self.xform = t * &self.xform;
    }

    /// Return a composed copy.
    pub fn transformed(&self, t: &Xform) -> Self {
        let mut result = self.duplicate();
        result.transform(t);

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a sorted JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from a JSON string; missing flags and features default to none.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump()
            .expect("Failed to serialize InstanceRef JSON")
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).expect("Failed to parse InstanceRef JSON")
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filename, self.jsondump()?)?;

        Ok(())
    }

    /// Read from a JSON file.
    pub fn file_json_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filename)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message; an identity xform, and color without FLAG_COLOR, are not written.
    pub fn to_proto(&self) -> crate::proto::InstanceRef {
        let mut features = Vec::with_capacity(self.features.len());

        for feature in &self.features {
            features.push(feature.to_proto());
        }

        crate::proto::InstanceRef {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            definition_guid: self.definition_guid.clone(),
            xform: (!self.xform.is_identity()).then(|| self.xform.to_proto()),
            color: (self.flags & Self::FLAG_COLOR != 0).then(|| self.color.to_proto()),
            flags: self.flags,
            features,
        }
    }

    /// Construct from the protobuf message; an absent xform is identity and an absent color white.
    pub fn from_proto(proto: crate::proto::InstanceRef) -> Self {
        let mut ref_ = Self::default();

        if !proto.guid.is_empty() {
            ref_.set_guid(proto.guid);
        }

        ref_.name = proto.name;
        ref_.definition_guid = proto.definition_guid;

        if let Some(xform) = proto.xform {
            ref_.xform = Xform::from_proto(xform);
        }

        if let Some(color) = proto.color {
            ref_.color = Color::from_proto(color);
        }

        ref_.flags = proto.flags;

        for feature in proto.features {
            ref_.features.push(ElementFeature::from_proto(feature));
        }

        ref_
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Ok(Self::from_proto(crate::proto::InstanceRef::decode(data)?))
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filename, self.pb_dumps())?;

        Ok(())
    }

    /// Read from a protobuf file.
    pub fn pb_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::pb_loads(&std::fs::read(filename)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "definition_guid @ [tx, ty, tz]".
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

    /// Return "InstanceRef(name, definition_guid, Color(...), flags)".
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
    /// Write the instance string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}
