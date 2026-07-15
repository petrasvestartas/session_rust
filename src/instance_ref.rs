use crate::{Color, Xform};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Index, IndexMut};

/// A block reference: places a definition (by guid) at a transform.
///
/// The only per-instance data is the placement `xform`; the geometry lives once
/// in the definition the `definition_guid` points to. Mirrors the Rhino block model.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "InstanceRef")]
pub struct InstanceRef {
    #[serde(serialize_with = "crate::guid_serde::serialize", deserialize_with = "crate::guid_serde::deserialize")]
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub definition_guid: String,
    #[serde(default = "Xform::identity")]
    pub xform: Xform,
    pub color: Color,
    pub flags: u32,
}

impl Default for InstanceRef {
    fn default() -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: "my_instance_ref".to_string(),
            definition_guid: String::new(),
            xform: Xform::identity(),
            color: Color::white(),
            flags: 0,
        }
    }
}

impl InstanceRef {
    pub fn new(definition_guid: &str, xform: Xform) -> Self {
        Self {
            definition_guid: definition_guid.to_string(),
            xform,
            ..Default::default()
        }
    }

    pub fn with_name(name: &str, definition_guid: &str, xform: Xform) -> Self {
        Self {
            name: name.to_string(),
            definition_guid: definition_guid.to_string(),
            xform,
            ..Default::default()
        }
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Create a duplicate with a new GUID.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = std::sync::OnceLock::new();
        copy
    }

    /// Compose an extra transform onto the placement (in-place): xform = t * xform.
    pub fn transform(&mut self, t: &Xform) {
        self.xform = t * &self.xform;
    }

    /// Return a copy with an extra transform composed onto the placement.
    pub fn transformed(&self, t: &Xform) -> Self {
        let mut result = self.clone();
        result.transform(t);
        result
    }

    /// Short string representation (definition + placement translation).
    pub fn str(&self) -> String {
        use crate::tolerance::TOLERANCE;
        let prec = crate::tolerance::Tolerance::ROUNDING;
        format!(
            "{} @ [{}, {}, {}]",
            self.definition_guid,
            TOLERANCE.format_number(self.xform.m[12], prec),
            TOLERANCE.format_number(self.xform.m[13], prec),
            TOLERANCE.format_number(self.xform.m[14], prec),
        )
    }

    /// Detailed string representation (like Python __repr__).
    pub fn repr(&self) -> String {
        format!(
            "InstanceRef({}, {}, Color({}, {}, {}, {}), {})",
            self.name,
            self.definition_guid,
            self.color.r,
            self.color.g,
            self.color.b,
            self.color.a,
            self.flags,
        )
    }

    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_else(|_| Self::default())
    }

    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        let proto = crate::proto::InstanceRef {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            definition_guid: self.definition_guid.clone(),
            xform: Some(crate::proto::Xform {
                guid: String::new(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
            color: Some(crate::proto::Color {
                guid: String::new(),
                r: self.color.r,
                g: self.color.g,
                b: self.color.b,
                a: self.color.a,
                name: String::new(),
            }),
            flags: self.flags,
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, prost::DecodeError> {
        use prost::Message;
        let proto = crate::proto::InstanceRef::decode(data)?;
        let mut ref_ = Self::new(&proto.definition_guid, Xform::identity());
        ref_.set_guid(proto.guid);
        ref_.name = proto.name;
        if let Some(x) = proto.xform {
            ref_.xform.name = x.name;
            for (i, v) in x.matrix.iter().enumerate() {
                if i < 16 {
                    ref_.xform.m[i] = *v;
                }
            }
        }
        if let Some(c) = proto.color {
            ref_.color.r = c.r;
            ref_.color.g = c.g;
            ref_.color.b = c.b;
            ref_.color.a = c.a;
        }
        ref_.flags = proto.flags;
        Ok(ref_)
    }

    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }
}

impl Index<usize> for InstanceRef {
    type Output = f64;

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

impl fmt::Display for InstanceRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

impl PartialEq for InstanceRef {
    fn eq(&self, other: &Self) -> bool {
        self.definition_guid == other.definition_guid
            && self.xform == other.xform
            && self.color == other.color
            && self.flags == other.flags
    }
}

#[path = "instance_ref_test.rs"]
#[cfg(test)]
mod instance_ref_test;
