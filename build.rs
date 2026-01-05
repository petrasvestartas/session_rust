fn main() {
    let proto_dir = "../session_proto";

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", proto_dir);

    #[cfg(feature = "protobuf")]
    {
        // Use bundled protoc from protobuf-src
        std::env::set_var("PROTOC", protobuf_src::protoc());

        let proto_files: Vec<String> = std::fs::read_dir(proto_dir)
            .expect("Failed to read proto directory")
            .filter_map(|entry| {
                let entry = entry.ok()?;
                let path = entry.path();
                if path.extension()? == "proto" {
                    Some(path.to_string_lossy().into_owned())
                } else {
                    None
                }
            })
            .collect();

        if !proto_files.is_empty() {
            prost_build::compile_protos(
                &proto_files.iter().map(|s| s.as_str()).collect::<Vec<_>>(),
                &[proto_dir],
            )
            .expect("Failed to compile protobuf files");
        }
    }
}
