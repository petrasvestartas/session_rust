use serde::Deserialize;
use serde::Serialize;
use std::fs;
use std::rc::Rc;

// ═══════════════════════════════════════════════════════════════════════════
// JSON string
// ═══════════════════════════════════════════════════════════════════════════
/// Serialize an object to a JSON string.
pub fn file_json_dumps<T: Serialize>(
    data: &T,
    pretty: bool,
) -> Result<String, Box<dyn std::error::Error>> {
    let json = sort_json_keys(serde_json::to_value(data)?);

    if pretty {
        return Ok(serde_json::to_string_pretty(&json)?);
    }

    Ok(serde_json::to_string(&json)?)
}

/// Deserialize an object from a JSON string.
pub fn file_json_loads<T: for<'de> Deserialize<'de>>(
    json_str: &str,
) -> Result<T, Box<dyn std::error::Error>> {
    Ok(serde_json::from_str(json_str)?)
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON file
// ═══════════════════════════════════════════════════════════════════════════
/// Write an object to a JSON file.
pub fn file_json_dump<T: Serialize>(
    data: &T,
    filepath: &str,
    pretty: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    fs::write(filepath, file_json_dumps(data, pretty)?)?;

    Ok(())
}

/// Read a JSON value from a file.
pub fn file_json_load_data(
    filepath: &str,
) -> Result<serde_json::Value, Box<dyn std::error::Error>> {
    let json_str = fs::read_to_string(filepath)?;

    Ok(serde_json::from_str(&json_str)?)
}

/// Read an object from a JSON file.
pub fn file_json_load<T: for<'de> Deserialize<'de>>(
    filepath: &str,
) -> Result<T, Box<dyn std::error::Error>> {
    let json_str = fs::read_to_string(filepath)?;

    file_json_loads(&json_str)
}

// ═══════════════════════════════════════════════════════════════════════════
// Collections
// ═══════════════════════════════════════════════════════════════════════════
/// Encode a collection of objects or shared pointers to a JSON array.
pub fn file_encode_collection<T: Serialize>(
    collection: &[T],
) -> Result<serde_json::Value, Box<dyn std::error::Error>> {
    let mut array = Vec::new();

    for item in collection {
        array.push(sort_json_keys(serde_json::to_value(item)?));
    }

    Ok(serde_json::Value::Array(array))
}

/// Decode a JSON array to a collection of objects.
pub fn file_decode_collection<T: for<'de> Deserialize<'de>>(
    data: &serde_json::Value,
) -> Result<Vec<T>, Box<dyn std::error::Error>> {
    let mut result = Vec::new();

    let Some(items) = data.as_array() else {
        return Ok(result);
    };

    for item in items {
        result.push(T::deserialize(item)?);
    }

    Ok(result)
}

/// Decode a JSON array to a collection of shared pointers.
pub fn file_decode_collection_ptr<T: for<'de> Deserialize<'de>>(
    data: &serde_json::Value,
) -> Result<Vec<Rc<T>>, Box<dyn std::error::Error>> {
    let mut result = Vec::new();

    let Some(items) = data.as_array() else {
        return Ok(result);
    };

    for item in items {
        result.push(Rc::new(T::deserialize(item)?));
    }

    Ok(result)
}

// ═══════════════════════════════════════════════════════════════════════════
// Sorted keys
// ═══════════════════════════════════════════════════════════════════════════
/// Sort object keys alphabetically at every depth.
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

        serde_json::Value::Array(array) => {
            let mut sorted = Vec::new();

            for item in array {
                sorted.push(sort_json_keys(item));
            }

            serde_json::Value::Array(sorted)
        }

        other => other,
    }
}

/// Serialize an object to a pretty JSON string with sorted keys and four-space indent.
pub fn sorted_json_string<T: Serialize>(data: &T) -> Result<String, Box<dyn std::error::Error>> {
    let sorted = sort_json_keys(serde_json::to_value(data)?);
    let mut buffer = Vec::new();
    let formatter = serde_json::ser::PrettyFormatter::with_indent(b"    ");
    let mut serializer = serde_json::Serializer::with_formatter(&mut buffer, formatter);
    sorted.serialize(&mut serializer)?;

    Ok(String::from_utf8(buffer)?)
}
