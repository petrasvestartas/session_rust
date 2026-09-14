use serde::Deserialize;
use serde::Serialize;
use std::fs;

/// Write data to a json file
pub fn file_json_dump<T: Serialize>(
    data: &T,
    filepath: &str,
    pretty: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    let json_str = file_json_dumps(data, pretty)?;
    fs::write(filepath, json_str)?;
    Ok(())
}

/// Read data from a json file
pub fn file_json_load<T: for<'de> Deserialize<'de>>(
    filepath: &str,
) -> Result<T, Box<dyn std::error::Error>> {
    let json_str = fs::read_to_string(filepath)?;
    file_json_loads(&json_str)
}

/// Serialize data to a json string with sorted keys
pub fn file_json_dumps<T: Serialize>(
    data: &T,
    pretty: bool,
) -> Result<String, Box<dyn std::error::Error>> {
    let sorted = sort_json_keys(serde_json::to_value(data)?);
    if pretty {
        return Ok(serde_json::to_string_pretty(&sorted)?);
    }
    Ok(serde_json::to_string(&sorted)?)
}

/// Deserialize data from a json string
pub fn file_json_loads<T: for<'de> Deserialize<'de>>(
    json_str: &str,
) -> Result<T, Box<dyn std::error::Error>> {
    Ok(serde_json::from_str(json_str)?)
}

/// Sort object keys alphabetically at every depth
pub fn sort_json_keys(value: serde_json::Value) -> serde_json::Value {
    match value {
        serde_json::Value::Object(map) => {
            let mut entries: Vec<(String, serde_json::Value)> = map.into_iter().collect();
            entries.sort_by(|a, b| a.0.cmp(&b.0));
            let mut sorted = serde_json::Map::new();
            for (key, item) in entries {
                sorted.insert(key, sort_json_keys(item));
            }
            serde_json::Value::Object(sorted)
        }
        serde_json::Value::Array(arr) => {
            serde_json::Value::Array(arr.into_iter().map(sort_json_keys).collect())
        }
        other => other,
    }
}

/// Serialize data to a pretty json string with sorted keys and four-space indent
pub fn sorted_json_string<T: Serialize>(data: &T) -> Result<String, Box<dyn std::error::Error>> {
    let sorted = sort_json_keys(serde_json::to_value(data)?);
    let mut buf = Vec::new();
    let formatter = serde_json::ser::PrettyFormatter::with_indent(b"    ");
    let mut ser = serde_json::Serializer::with_formatter(&mut buf, formatter);
    sorted.serialize(&mut ser)?;
    Ok(String::from_utf8(buf)?)
}
