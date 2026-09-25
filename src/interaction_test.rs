use crate::mini_test::TestResult;
use crate::Interaction;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;
use std::sync::OnceLock;

/// A test-only implementor: a named interaction with no state of its own.
#[derive(Debug, Clone, Default)]
struct NamedInteraction {
    guid: OnceLock<String>, // Lazily minted guid.
    name: String,           // What joins the pair.
}

impl NamedInteraction {
    /// Construct from a name.
    fn new(name: &str) -> Self {
        Self {
            guid: OnceLock::new(),
            name: name.to_string(),
        }
    }
}

impl Interaction for NamedInteraction {
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

    /// Return the registered type name.
    fn interaction_type_name(&self) -> &str {
        "NamedInteraction"
    }

    /// Return no state.
    fn interaction_data_dumps(&self) -> Vec<u8> {
        Vec::new()
    }

    /// Return a copy with the same guid.
    fn clone_box(&self) -> Box<dyn Interaction> {
        self.guid();

        Box::new(self.clone())
    }
}

/// Build a NamedInteraction from its data.
fn named_interaction(_data: &[u8]) -> Option<Box<dyn Interaction>> {
    Some(Box::new(NamedInteraction::default()))
}

pub fn run_interaction_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        let unnamed: Box<dyn Interaction> = Box::new(NamedInteraction::default());
        let glue: Box<dyn Interaction> = Box::new(NamedInteraction::new("glue"));
        let duplicate = glue.clone();
        let cloned = glue.clone_box();

        MINI_CHECK!(unnamed.name().is_empty());
        MINI_CHECK!(glue.name() == "glue");
        MINI_CHECK!(glue.interaction_type_name() == "NamedInteraction");
        MINI_CHECK!(*duplicate == *glue);
        MINI_CHECK!(duplicate.guid() == glue.guid());
        MINI_CHECK!(*cloned == *glue);
        MINI_CHECK!(cloned.guid() == glue.guid());
        MINI_CHECK!(*unnamed != *glue);
        MINI_CHECK!(unnamed.guid() != glue.guid());
        MINI_CHECK!(glue.str() == "NamedInteraction(glue)");
        MINI_CHECK!(glue.repr() == format!("NamedInteraction({}, glue)", glue.guid()));
    })
}

pub fn run_interaction_abstract_base() -> TestResult {
    MINI_TEST!("Abstract Base", {
        let glue: Box<dyn Interaction> = Box::new(NamedInteraction::new("glue"));

        MINI_CHECK!(std::mem::size_of::<&dyn Interaction>() == 2 * std::mem::size_of::<usize>());
        MINI_CHECK!(glue.interaction_type_name() == "NamedInteraction");
    })
}

pub fn run_interaction_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        <dyn Interaction>::register_type("NamedInteraction", named_interaction);
        let glue: Box<dyn Interaction> = Box::new(NamedInteraction::new("glue"));

        let data = glue.jsondump();
        let loaded_j = <dyn Interaction>::jsonload(&data);
        let loaded_s = <dyn Interaction>::file_json_loads(&glue.file_json_dumps());

        let filename = "serialization/test_interaction.json";
        glue.file_json_dump(filename).unwrap();
        let loaded = <dyn Interaction>::file_json_load(filename).unwrap();

        MINI_CHECK!(data["type"] == "Interaction");
        MINI_CHECK!(*loaded_j == *glue);
        MINI_CHECK!(*loaded_s == *glue);
        MINI_CHECK!(*loaded == *glue);
        MINI_CHECK!(loaded.guid() == glue.guid());
    })
}

pub fn run_interaction_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        <dyn Interaction>::register_type("NamedInteraction", named_interaction);
        let glue: Box<dyn Interaction> = Box::new(NamedInteraction::new("glue"));

        let proto = glue.to_proto();
        let converted = <dyn Interaction>::from_proto(proto.clone());
        let loaded_b = <dyn Interaction>::pb_loads(&glue.pb_dumps()).unwrap();

        let filename = "serialization/test_interaction.bin";
        glue.pb_dump(filename).unwrap();
        let loaded = <dyn Interaction>::pb_load(filename).unwrap();

        MINI_CHECK!(proto.interaction_type == "NamedInteraction");
        MINI_CHECK!(*converted == *glue);
        MINI_CHECK!(*loaded_b == *glue);
        MINI_CHECK!(*loaded == *glue);
        MINI_CHECK!(loaded.guid() == glue.guid());
    })
}

pub fn run_interaction_registry_unknown_type() -> TestResult {
    MINI_TEST!("Registry Unknown Type", {
        let mystery: Box<dyn Interaction> = Box::new(NamedInteraction::new("mystery"));
        let mut proto = mystery.to_proto();
        proto.interaction_type = "NeverRegistered".to_string();
        proto.interaction_data = b"whatever this package meant".to_vec();
        let loaded = <dyn Interaction>::pb_loads(&prost::Message::encode_to_vec(&proto)).unwrap();
        let saved = <dyn Interaction>::pb_loads(&loaded.pb_dumps()).unwrap();

        MINI_CHECK!(!<dyn Interaction>::is_registered("NeverRegistered"));
        MINI_CHECK!(format!("{:?}", loaded).starts_with("InteractionUnknown"));
        MINI_CHECK!(loaded.name() == "mystery");
        MINI_CHECK!(loaded.interaction_type_name() == "NeverRegistered");
        MINI_CHECK!(loaded.interaction_data_dumps() == b"whatever this package meant");
        MINI_CHECK!(*saved == *loaded);
    })
}

REGISTER_MINI_TEST!(
    "Interaction",
    "Constructor",
    crate::interaction_test::run_interaction_constructor
);
REGISTER_MINI_TEST!(
    "Interaction",
    "Abstract Base",
    crate::interaction_test::run_interaction_abstract_base
);
REGISTER_MINI_TEST!(
    "Interaction",
    "Json Roundtrip",
    crate::interaction_test::run_interaction_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Interaction",
    "Protobuf Roundtrip",
    crate::interaction_test::run_interaction_protobuf_roundtrip
);
REGISTER_MINI_TEST!(
    "Interaction",
    "Registry Unknown Type",
    crate::interaction_test::run_interaction_registry_unknown_type
);
