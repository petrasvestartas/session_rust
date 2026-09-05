use serde::{Deserialize, Deserializer, Serializer};
use std::sync::OnceLock;

pub fn serialize<S>(guid: &OnceLock<String>, s: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    s.serialize_str(guid.get_or_init(|| uuid::Uuid::new_v4().to_string()))
}

pub fn deserialize<'de, D>(d: D) -> Result<OnceLock<String>, D::Error>
where
    D: Deserializer<'de>,
{
    let val = String::deserialize(d)?;
    let cell = OnceLock::new();
    let _ = cell.set(val);
    Ok(cell)
}
