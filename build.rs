fn main() {
    // Use shared proto files from root session_proto/
    let proto_dir = "../session_proto";
    
    // Tell Cargo to only re-run this build script if these files change
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}/point.proto", proto_dir);
    println!("cargo:rerun-if-changed={}/color.proto", proto_dir);
    println!("cargo:rerun-if-changed={}/xform.proto", proto_dir);

    #[cfg(feature = "protobuf")]
    {
        prost_build::compile_protos(
            &[
                &format!("{}/point.proto", proto_dir),
                &format!("{}/color.proto", proto_dir),
                &format!("{}/xform.proto", proto_dir),
            ],
            &[proto_dir],
        )
        .expect("Failed to compile protobuf files");
    }
}
