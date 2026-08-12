use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_pdf_import_minimal() -> TestResult {
    MINI_TEST!("Import Minimal", {
        // session_data/minimal.pdf is hand-built and 463 bytes: a 100x100 page holding one blue
        // stroked line (10,10)->(90,10) at width 1, and one red filled 60x40 rectangle at (20,30).
        use std::path::PathBuf;
        use crate::pdf::import_pdf;
        use crate::Session;
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let pdf_path = src_dir.parent().unwrap().join("session_data").join("minimal.pdf");
        if !pdf_path.exists() {
            return Ok(());
        }
        let stem = src_dir.join("serialization").join("test_temp_pdf");
        import_pdf(pdf_path.to_str().unwrap(), stem.to_str().unwrap(), 0);
        let out = PathBuf::from(format!("{}.pb", stem.to_str().unwrap()));
        MINI_CHECK!(out.exists());
        let session = Session::pb_load(out.to_str().unwrap());

        // stroke -> Line, fill -> Mesh, page box -> Polyline
        MINI_CHECK!(session.objects.lines.len() == 1);
        MINI_CHECK!(session.objects.meshes.len() == 1);
        MINI_CHECK!(session.objects.polylines.len() == 1);

        // PDF y points down and is flipped on import; 1 pt = 1 mm, z = 0
        let line = &session.objects.lines[0];
        MINI_CHECK!(TOLERANCE.is_close(line[0], 10.0));
        MINI_CHECK!(TOLERANCE.is_close(line[3], 90.0));
        MINI_CHECK!(TOLERANCE.is_close(line[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(line[5], 0.0));
        // "0 0 1 RG" -> blue, "1 w" -> absolute mm width
        MINI_CHECK!(TOLERANCE.is_close(line.linecolor.b as f64, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(line.linecolor.r as f64, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(line.width, 1.0));

        // the rectangle earcuts into 2 triangles over its 4 corners
        let mesh = &session.objects.meshes[0];
        MINI_CHECK!(mesh.number_of_vertices() == 4);
        MINI_CHECK!(mesh.number_of_faces() == 2);

        let _ = std::fs::remove_file(&out);
    })
}

REGISTER_MINI_TEST!("Pdf", "Import Minimal", crate::pdf_test::run_pdf_import_minimal);
