use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt;

/// A joint element feature — metadata for a joint between two elements.
///
/// Uniform shape for all joint types; the `type_` code discriminates
/// (10=face_to_face, 20=border_to_face, 30=plane_to_face/cross, ...).
/// Geometry (joint_area, joint_lines, joint_volumes) lives in
/// `Session::lookup` as Polylines and is referenced here by GUID.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "JointElementFeature")]
pub struct JointElementFeature {
    /// The unique identifier of the element feature.
    #[serde(serialize_with = "crate::guid_serde::serialize", deserialize_with = "crate::guid_serde::deserialize")]
    guid: std::sync::OnceLock<String>,
    /// Joint type code (10=face_to_face, 20=border_to_face, 30=plane_to_face/cross).
    #[serde(rename = "type")]
    pub type_: i32,
    pub face_ids_a_first: i32,
    pub face_ids_a_second: i32,
    pub face_ids_b_first: i32,
    pub face_ids_b_second: i32,
    /// Polyline GUIDs in `Session::lookup`.
    pub joint_area_guids: Vec<String>,
    /// Polyline GUIDs in `Session::lookup`.
    pub joint_line_guids: Vec<String>,
    /// Polyline GUIDs in `Session::lookup`.
    pub joint_volume_guids: Vec<String>,
    /// Type-specific scalars (insertion direction x/y/z, depth, ...).
    pub extras: HashMap<String, f32>,
}

impl Default for JointElementFeature {
    fn default() -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            type_: 0,
            face_ids_a_first: -1,
            face_ids_a_second: -1,
            face_ids_b_first: -1,
            face_ids_b_second: -1,
            joint_area_guids: Vec::new(),
            joint_line_guids: Vec::new(),
            joint_volume_guids: Vec::new(),
            extras: HashMap::new(),
        }
    }
}

impl PartialEq for JointElementFeature {
    fn eq(&self, other: &Self) -> bool {
        self.type_ == other.type_
            && self.face_ids_a_first == other.face_ids_a_first
            && self.face_ids_a_second == other.face_ids_a_second
            && self.face_ids_b_first == other.face_ids_b_first
            && self.face_ids_b_second == other.face_ids_b_second
            && self.joint_area_guids == other.joint_area_guids
            && self.joint_line_guids == other.joint_line_guids
            && self.joint_volume_guids == other.joint_volume_guids
            && self.extras == other.extras
    }
}

impl fmt::Display for JointElementFeature {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "JointElementFeature(type={}, area={}, lines={}, volumes={})",
            self.type_,
            self.joint_area_guids.len(),
            self.joint_line_guids.len(),
            self.joint_volume_guids.len()
        )
    }
}

impl JointElementFeature {
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    pub fn new() -> Self {
        Self::default()
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        let proto = crate::proto::JointElementFeature {
            guid: self.guid().to_string(),
            r#type: self.type_,
            face_ids_a_first: self.face_ids_a_first,
            face_ids_a_second: self.face_ids_a_second,
            face_ids_b_first: self.face_ids_b_first,
            face_ids_b_second: self.face_ids_b_second,
            joint_area_guids: self.joint_area_guids.clone(),
            joint_line_guids: self.joint_line_guids.clone(),
            joint_volume_guids: self.joint_volume_guids.clone(),
            extras: self.extras.clone(),
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::JointElementFeature::decode(data)?;
        let jf = JointElementFeature {
            guid: { let lock = std::sync::OnceLock::new(); let _ = lock.set(proto.guid); lock },
            type_: proto.r#type,
            face_ids_a_first: proto.face_ids_a_first,
            face_ids_a_second: proto.face_ids_a_second,
            face_ids_b_first: proto.face_ids_b_first,
            face_ids_b_second: proto.face_ids_b_second,
            joint_area_guids: proto.joint_area_guids,
            joint_line_guids: proto.joint_line_guids,
            joint_volume_guids: proto.joint_volume_guids,
            extras: proto.extras,
        };
        Ok(jf)
    }
}

/// Sum type of everything that can live on a graph edge as a structured element feature.
/// New non-joint element feature families become new enum variants.
/// New joint types stay cheap: just a new `type_` code on `JointElementFeature`.
#[derive(Debug, Clone, PartialEq)]
pub enum EdgeElementFeature {
    Empty,
    Joint(JointElementFeature),
}

impl Default for EdgeElementFeature {
    fn default() -> Self {
        EdgeElementFeature::Empty
    }
}

/// Serialize an `EdgeElementFeature` to a pb byte vector. Returns empty for `Empty`.
pub fn edge_elementfeature_pb_dumps(f: &EdgeElementFeature) -> Vec<u8> {
    use prost::Message;
    let proto = match f {
        EdgeElementFeature::Joint(jf) => crate::proto::EdgeElementFeature {
            payload: Some(crate::proto::edge_element_feature::Payload::Joint(
                crate::proto::JointElementFeature::decode(jf.pb_dumps().as_slice()).unwrap_or_default(),
            )),
        },
        EdgeElementFeature::Empty => crate::proto::EdgeElementFeature { payload: None },
    };
    proto.encode_to_vec()
}

/// Deserialize an `EdgeElementFeature` from a pb byte slice. Returns `Empty` on empty/unknown.
pub fn edge_elementfeature_pb_loads(data: &[u8]) -> EdgeElementFeature {
    use prost::Message;
    let proto = match crate::proto::EdgeElementFeature::decode(data) {
        Ok(p) => p,
        Err(_) => return EdgeElementFeature::Empty,
    };
    match proto.payload {
        Some(crate::proto::edge_element_feature::Payload::Joint(jp)) => {
            match JointElementFeature::pb_loads(&jp.encode_to_vec()) {
                Ok(jf) => EdgeElementFeature::Joint(jf),
                Err(_) => EdgeElementFeature::Empty,
            }
        }
        None => EdgeElementFeature::Empty,
    }
}

/// Serialize an `EdgeElementFeature` to a JSON value. Returns null-ish for `Empty`.
pub fn edge_elementfeature_jsondump(f: &EdgeElementFeature) -> serde_json::Value {
    match f {
        EdgeElementFeature::Joint(jf) => serde_json::json!({
            "kind": "JointElementFeature",
            "data": serde_json::to_value(jf).unwrap_or(serde_json::Value::Null),
        }),
        EdgeElementFeature::Empty => serde_json::json!({ "kind": "Empty" }),
    }
}

/// Deserialize an `EdgeElementFeature` from a JSON value. Returns `Empty` on null/unknown.
pub fn edge_elementfeature_jsonload(data: &serde_json::Value) -> EdgeElementFeature {
    let kind = data.get("kind").and_then(|k| k.as_str()).unwrap_or("Empty");
    if kind == "JointElementFeature" {
        if let Some(inner) = data.get("data") {
            match serde_json::from_value::<JointElementFeature>(inner.clone()) {
                Ok(jf) => return EdgeElementFeature::Joint(jf),
                Err(_) => return EdgeElementFeature::Empty,
            }
        }
    }
    EdgeElementFeature::Empty
}
