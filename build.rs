// Generates protobuf Rust bindings into src/proto/ (committed).
//
// Option C: the generated file is checked into the repo. Consumers of
// session_rust never need protoc to build. During development, edits to
// ../session_proto/*.proto cause this script to rewrite src/proto/session_proto.rs
// so the diff shows up in `git status` and gets committed alongside the schema.
//
// Set REGEN_PROTO=0 to skip (useful for offline end-user builds without protoc).

use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=REGEN_PROTO");

    let proto_dir = PathBuf::from("../session_proto");
    let out_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/proto");

    if std::env::var("REGEN_PROTO").as_deref() == Ok("0") {
        return;
    }

    if !proto_dir.exists() {
        return;
    }

    let mut proto_files: Vec<PathBuf> = std::fs::read_dir(&proto_dir)
        .expect("read session_proto")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("proto"))
        .collect();
    // Deterministic ordering — read_dir returns FS-order which differs
    // between NTFS / ext4 / APFS and would cause prost-build output to
    // diverge across platforms.
    proto_files.sort();

    if proto_files.is_empty() {
        return;
    }

    for f in &proto_files {
        println!("cargo:rerun-if-changed={}", f.display());
    }

    std::fs::create_dir_all(&out_dir).expect("create src/proto");

    let protoc = protoc_bin_vendored::protoc_bin_path()
        .expect("protoc-bin-vendored failed to locate bundled protoc");
    std::env::set_var("PROTOC", &protoc);

    let staging = PathBuf::from(std::env::var("OUT_DIR").unwrap()).join("proto_staging");
    std::fs::create_dir_all(&staging).expect("create staging");

    let mut cfg = prost_build::Config::new();
    cfg.out_dir(&staging);
    // Map fields as BTreeMap, so the encoding is reproducible: prost writes a map in ITERATION
    // order and Rust seeds every HashMap differently, so `pb_dumps()` of one unchanged object
    // produced different bytes on every run - a golden file or a content hash over saved output
    // would flake.
    //
    // Mesh's four BIG maps (vertices, faces, halfedges, triangulation) are deliberately NOT in
    // this list. A B-tree there cost 55% on every DECODE - 218 -> 338 ms on one 52 MB sheet,
    // paid by every file load - to fix an order that only matters when WRITING. They stay
    // HashMap for the read path, and `MeshEnc` in mesh.rs sorts them at encode time instead.
    cfg.btree_map([
        ".session_proto.Graph.vertices",
        ".session_proto.VertexData.attributes",
        ".session_proto.FaceData.attributes",
        ".session_proto.EdgeData.attributes",
        ".session_proto.HalfedgeMap.neighbors",
        ".session_proto.Mesh.default_vertex_attributes",
        ".session_proto.Mesh.default_face_attributes",
        ".session_proto.Mesh.default_edge_attributes",
    ]);
    cfg.compile_protos(&proto_files, &[&proto_dir])
        .expect("prost-build: compile_protos");

    // Copy to src/proto/ only when content differs so cargo does not
    // mtime-rebuild the library on every run.
    for entry in std::fs::read_dir(&staging).expect("read staging") {
        let entry = entry.expect("entry");
        let src = entry.path();
        let dst = out_dir.join(entry.file_name());
        let new_bytes = std::fs::read(&src).expect("read staging file");
        let changed = match std::fs::read(&dst) {
            Ok(old) => old != new_bytes,
            Err(_) => true,
        };
        if changed {
            std::fs::write(&dst, &new_bytes).expect("write committed proto");
        }
    }
}
