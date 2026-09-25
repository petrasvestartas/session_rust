use std::collections::BTreeMap;
use std::fmt;
use std::sync::Mutex;
use std::sync::OnceLock;

// ═══════════════════════════════════════════════════════════════════════════
// Hex encoding
// ═══════════════════════════════════════════════════════════════════════════
/// Encode bytes as hex text, since interaction_data is opaque and JSON carries no bytes.
fn to_hex(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);

    for b in bytes {
        out.push_str(&format!("{:02x}", b));
    }

    out
}

/// Decode hex text back to bytes; bad hex is an error.
fn from_hex(hex: &str) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let mut out = Vec::with_capacity(hex.len() / 2);

    for i in (0..hex.len().saturating_sub(1)).step_by(2) {
        let pair = hex.get(i..i + 2).ok_or("Invalid hex")?;

        out.push(u8::from_str_radix(pair, 16)?);
    }

    Ok(out)
}

// ═══════════════════════════════════════════════════════════════════════════
// Registry lookup
// ═══════════════════════════════════════════════════════════════════════════
/// The factory an implementor registers for its interaction_type: builds it from its interaction_data, or declines with None and loads an InteractionUnknown.
pub type InteractionFactory = fn(&[u8]) -> Option<Box<dyn Interaction>>;

/// Function-local so a package registering from a static initializer finds it built.
fn interaction_registry() -> &'static Mutex<BTreeMap<String, InteractionFactory>> {
    static REGISTRY: OnceLock<Mutex<BTreeMap<String, InteractionFactory>>> = OnceLock::new();

    REGISTRY.get_or_init(|| Mutex::new(BTreeMap::new()))
}

/// The registered type built from its data, an InteractionUnknown when the type is unknown or its factory declines, then given the guid and the name.
fn build_registered(type_name: &str, data: &[u8], guid: &str, name: &str) -> Box<dyn Interaction> {
    let mut factory: Option<InteractionFactory> = None;

    if let Ok(registry) = interaction_registry().lock() {
        factory = registry.get(type_name).copied();
    }

    let built = match factory {
        Some(factory) => factory(data),
        None => None,
    };
    let mut interaction = match built {
        Some(interaction) => interaction,
        None => Box::new(InteractionUnknown::new(type_name, data, "")),
    };

    if !guid.is_empty() {
        interaction.set_guid(guid.to_string());
    }

    interaction.set_name(name);

    interaction
}

// ═══════════════════════════════════════════════════════════════════════════
// Interaction
// ═══════════════════════════════════════════════════════════════════════════
/// What joins two elements, stored on their graph edge: every interaction type implements it and registers a factory under its type name.
pub trait Interaction: fmt::Debug {
    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    fn has_guid(&self) -> bool;

    /// Return the guid, creating it on first access.
    fn guid(&self) -> &str;

    /// Set the guid.
    fn set_guid(&mut self, guid: String);

    /// Return what joins the pair, e.g. "glue"; empty when unnamed.
    fn name(&self) -> &str;

    /// Set the name.
    fn set_name(&mut self, name: &str);

    /// Return the type name the implementor registered its factory under.
    fn interaction_type_name(&self) -> &str;

    /// Return the implementor's own state, opaque to the kernel; its factory reads it back.
    fn interaction_data_dumps(&self) -> Vec<u8>;

    // ═══════════════════════════════════════════════════════════════════════════
    // Utilities
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a polymorphic copy of the same type with the same guid.
    fn clone_box(&self) -> Box<dyn Interaction>;
}

impl Clone for Box<dyn Interaction> {
    /// Copy the same type with the same guid.
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

impl PartialEq for dyn Interaction {
    /// Compare name, type and data; guid ignored.
    fn eq(&self, other: &Self) -> bool {
        self.name() == other.name()
            && self.interaction_type_name() == other.interaction_type_name()
            && self.interaction_data_dumps() == other.interaction_data_dumps()
    }
}

impl dyn Interaction {
    // ═══════════════════════════════════════════════════════════════════════════
    // Polymorphic registry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Register factory for type_name; re-registering the same name replaces it.
    pub fn register_type(type_name: &str, factory: InteractionFactory) {
        if type_name.is_empty() {
            return;
        }

        if let Ok(mut registry) = interaction_registry().lock() {
            registry.insert(type_name.to_string(), factory);
        }
    }

    /// Return whether a factory is registered for type_name.
    pub fn is_registered(type_name: &str) -> bool {
        if let Ok(registry) = interaction_registry().lock() {
            return registry.contains_key(type_name);
        }

        false
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    pub fn jsondump(&self) -> serde_json::Value {
        serde_json::json!({
            "guid": self.guid(),
            "interaction_data": to_hex(&self.interaction_data_dumps()),
            "interaction_type": self.interaction_type_name(),
            "name": self.name(),
            "type": "Interaction",
        })
    }

    /// Deserialize from a JSON object through the registry; an unregistered type loads as an InteractionUnknown.
    pub fn jsonload(data: &serde_json::Value) -> Box<dyn Interaction> {
        build_registered(
            data["interaction_type"].as_str().unwrap_or(""),
            &from_hex(data["interaction_data"].as_str().unwrap_or(""))
                .expect("Invalid interaction_data hex"),
            data["guid"].as_str().unwrap_or(""),
            data["name"].as_str().unwrap_or(""),
        )
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        crate::file_encoders::file_json_dumps(&self.jsondump(), false)
            .expect("Failed to serialize Interaction JSON")
    }

    /// Deserialize from a JSON string through the registry; an unregistered type loads as an InteractionUnknown.
    pub fn file_json_loads(json_string: &str) -> Box<dyn Interaction> {
        let data: serde_json::Value =
            serde_json::from_str(json_string).expect("Failed to parse Interaction JSON");

        Self::jsonload(&data)
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        crate::file_encoders::file_json_dump(&self.jsondump(), filename, true)
    }

    /// Read from a JSON file through the registry; an unregistered type loads as an InteractionUnknown.
    pub fn file_json_load(
        filename: &str,
    ) -> Result<Box<dyn Interaction>, Box<dyn std::error::Error>> {
        let data: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(filename)?)?;

        Ok(Self::jsonload(&data))
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::Interaction {
        crate::proto::Interaction {
            guid: self.guid().to_string(),
            name: self.name().to_string(),
            interaction_type: self.interaction_type_name().to_string(),
            interaction_data: self.interaction_data_dumps(),
        }
    }

    /// Construct from the protobuf message through the registry; an unregistered type loads as an InteractionUnknown.
    pub fn from_proto(proto: crate::proto::Interaction) -> Box<dyn Interaction> {
        build_registered(
            &proto.interaction_type,
            &proto.interaction_data,
            &proto.guid,
            &proto.name,
        )
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes through the registry; an unregistered type loads as an InteractionUnknown.
    pub fn pb_loads(data: &[u8]) -> Result<Box<dyn Interaction>, Box<dyn std::error::Error>> {
        use prost::Message;

        Ok(Self::from_proto(crate::proto::Interaction::decode(data)?))
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filename, self.pb_dumps())?;

        Ok(())
    }

    /// Read from a protobuf file through the registry; an unregistered type loads as an InteractionUnknown.
    pub fn pb_load(filename: &str) -> Result<Box<dyn Interaction>, Box<dyn std::error::Error>> {
        Self::pb_loads(&std::fs::read(filename)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "Type(name)".
    pub fn str(&self) -> String {
        format!("{}({})", self.interaction_type_name(), self.name())
    }

    /// Return "Type(guid, name)".
    pub fn repr(&self) -> String {
        format!(
            "{}({}, {})",
            self.interaction_type_name(),
            self.guid(),
            self.name()
        )
    }
}

impl fmt::Display for dyn Interaction {
    /// Write the interaction string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// InteractionUnknown
// ═══════════════════════════════════════════════════════════════════════════
/// An interaction whose type has no registered factory: a load keeps its type name and data so a save writes them back unchanged.
#[derive(Debug, Clone)]
pub struct InteractionUnknown {
    guid: OnceLock<String>, // Lazily minted guid.
    pub type_name: String,  // The type name it was written under.
    pub data: Vec<u8>,      // Its opaque state.
    pub name: String,       // What joins the pair; empty when unnamed.
}

impl InteractionUnknown {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from a type name, its data and a name.
    pub fn new(type_name: &str, data: &[u8], name: &str) -> Self {
        Self {
            guid: OnceLock::new(),
            type_name: type_name.to_string(),
            data: data.to_vec(),
            name: name.to_string(),
        }
    }
}

impl Interaction for InteractionUnknown {
    /// Return whether the lazy guid has been created.
    fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the guid, creating it on first access.
    fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid.
    fn set_guid(&mut self, guid: String) {
        self.guid = OnceLock::from(guid);
    }

    /// Return the name.
    fn name(&self) -> &str {
        &self.name
    }

    /// Set the name.
    fn set_name(&mut self, name: &str) {
        self.name = name.to_string();
    }

    /// Return the type name it was written under.
    fn interaction_type_name(&self) -> &str {
        &self.type_name
    }

    /// Return its opaque state.
    fn interaction_data_dumps(&self) -> Vec<u8> {
        self.data.clone()
    }

    /// Return a copy with the same guid.
    fn clone_box(&self) -> Box<dyn Interaction> {
        self.guid();

        Box::new(self.clone())
    }
}
