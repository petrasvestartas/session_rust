use std::path::PathBuf;

fn main() {
    let proto_dir = "../session_proto";

    println!("cargo:rerun-if-changed=build.rs");

    let protoc_path = download_protoc();
    std::env::set_var("PROTOC", &protoc_path);

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

    for proto_file in &proto_files {
        println!("cargo:rerun-if-changed={}", proto_file);
    }

    if !proto_files.is_empty() {
        prost_build::compile_protos(
            &proto_files.iter().map(|s| s.as_str()).collect::<Vec<_>>(),
            &[proto_dir],
        )
        .expect("Failed to compile protobuf files");
    }
}

fn download_protoc() -> PathBuf {
    use std::io::Read;

    let version = "29.0";
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let protoc_dir = out_dir.join("protoc");

    #[cfg(target_os = "windows")]
    let (platform, exe_name) = ("win64", "protoc.exe");

    #[cfg(all(target_os = "linux", target_arch = "x86_64"))]
    let (platform, exe_name) = ("linux-x86_64", "protoc");

    #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
    let (platform, exe_name) = ("linux-aarch_64", "protoc");

    #[cfg(all(target_os = "macos", target_arch = "x86_64"))]
    let (platform, exe_name) = ("osx-x86_64", "protoc");

    #[cfg(all(target_os = "macos", target_arch = "aarch64"))]
    let (platform, exe_name) = ("osx-aarch_64", "protoc");

    let protoc_exe = protoc_dir.join("bin").join(exe_name);

    if protoc_exe.exists() {
        return protoc_exe;
    }

    let url = format!(
        "https://github.com/protocolbuffers/protobuf/releases/download/v{}/protoc-{}-{}.zip",
        version, version, platform
    );

    eprintln!("Downloading protoc from {}...", url);

    let response = ureq::get(&url).call().expect("Failed to download protoc");
    let mut bytes = Vec::new();
    response.into_reader().read_to_end(&mut bytes).expect("Failed to read response");

    let mut archive = zip::ZipArchive::new(std::io::Cursor::new(bytes)).expect("Failed to open zip");

    for i in 0..archive.len() {
        let mut file = archive.by_index(i).unwrap();
        let outpath = protoc_dir.join(file.name());

        if file.name().ends_with('/') {
            std::fs::create_dir_all(&outpath).ok();
        } else {
            if let Some(parent) = outpath.parent() {
                std::fs::create_dir_all(parent).ok();
            }
            let mut outfile = std::fs::File::create(&outpath).unwrap();
            std::io::copy(&mut file, &mut outfile).unwrap();

            #[cfg(unix)]
            {
                use std::os::unix::fs::PermissionsExt;
                if outpath.to_string_lossy().contains("bin/protoc") {
                    std::fs::set_permissions(&outpath, std::fs::Permissions::from_mode(0o755)).ok();
                }
            }
        }
    }

    protoc_exe
}
