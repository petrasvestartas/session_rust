fn main() {
    #[cfg(feature = "protobuf")]
    {
        prost_build::compile_protos(
            &["proto/point.proto", "proto/color.proto", "proto/xform.proto"],
            &["proto/"],
        )
        .expect("Failed to compile protobuf files");
    }
}
