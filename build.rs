fn main() {
    // Use shared proto files from root session_proto/
    let proto_dir = "../session_proto";
    
    // Tell Cargo to re-run this build script if build.rs or any proto file changes
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", proto_dir);

    #[cfg(feature = "protobuf")]
    {
        // Auto-discover all .proto files in the proto directory
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
