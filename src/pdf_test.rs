// ═══════════════════════════════════════════════════════════════════════════
// Rust-only PDF import
// ═══════════════════════════════════════════════════════════════════════════
#[cfg(test)]
mod pdf_tests {
    use crate::pdf::import_pdf;
    use crate::tolerance::TOLERANCE;
    use crate::Session;
    use std::path::PathBuf;

    #[test]
    fn import_minimal() {
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let pdf_path = src_dir
            .parent()
            .unwrap()
            .join("session_data")
            .join("minimal.pdf");
        if !pdf_path.exists() {
            return;
        }
        let stem = src_dir.join("serialization").join("test_temp_pdf");
        import_pdf(pdf_path.to_str().unwrap(), stem.to_str().unwrap(), 0);
        let out = PathBuf::from(format!("{}.pb", stem.to_str().unwrap()));
        assert!(out.exists());
        let session = Session::pb_load(out.to_str().unwrap());

        assert!(session.objects.lines.len() == 1);
        assert!(session.objects.meshes.len() == 1);
        assert!(session.objects.polylines.len() == 1);

        let line = &session.objects.lines[0];
        assert!(TOLERANCE.is_close(line[0], 10.0));
        assert!(TOLERANCE.is_close(line[3], 90.0));
        assert!(TOLERANCE.is_close(line[2], 0.0));
        assert!(TOLERANCE.is_close(line[5], 0.0));
        assert!(TOLERANCE.is_close(line.linecolor.b as f64, 1.0));
        assert!(TOLERANCE.is_close(line.linecolor.r as f64, 0.0));
        assert!(TOLERANCE.is_close(line.width, 1.0));

        let mesh = &session.objects.meshes[0];
        assert!(mesh.number_of_vertices() == 4);
        assert!(mesh.number_of_faces() == 2);

        std::fs::remove_file(&out).unwrap();
    }
}
