use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_io_read_bunny() -> TestResult {
    MINI_TEST!("Read Bunny", {
        // load Stanford Bunny (real-world XYZ point cloud: 397 points)
        use std::path::PathBuf;
        use crate::io::read_xyz;
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let bunny_path = src_dir.parent().unwrap().join("session_data").join("bunny.xyz");
        if !bunny_path.exists() {
            return Ok(());
        }
        let cloud = read_xyz(bunny_path.to_str().unwrap()).unwrap();

        MINI_CHECK!(cloud.point_count() == 397);
        let points = cloud.get_points();
        MINI_CHECK!(points.len() == 397);
        let has_non_zero = points.iter().any(|p| p[0] != 0.0 || p[1] != 0.0 || p[2] != 0.0);
        MINI_CHECK!(has_non_zero);
    })
}

pub fn run_io_write_read_roundtrip() -> TestResult {
    MINI_TEST!("Write Read Roundtrip", {
        // build a small cloud (4 points), write to XYZ, read back, compare counts
        use crate::{Point, PointCloud};
        use crate::io::{read_xyz, write_xyz};
        use std::path::PathBuf;
        let mut original = PointCloud::default();
        original.add_point(&Point::new(0.0, 0.0, 0.0));
        original.add_point(&Point::new(1.0, 0.0, 0.0));
        original.add_point(&Point::new(0.0, 1.0, 0.0));
        original.add_point(&Point::new(0.0, 0.0, 1.0));

        MINI_CHECK!(original.point_count() == 4);
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let temp_file = src_dir.join("serialization").join("test_temp_roundtrip.xyz");
        let temp_str = temp_file.to_str().unwrap();
        write_xyz(&original, temp_str).unwrap();
        MINI_CHECK!(temp_file.exists());
        let loaded = read_xyz(temp_str).unwrap();
        MINI_CHECK!(loaded.point_count() == original.point_count());
        let _ = std::fs::remove_file(&temp_file);
    })
}

pub fn run_io_string_roundtrip() -> TestResult {
    MINI_TEST!("String Roundtrip", {
        use crate::{Point, PointCloud};
        use crate::io::{read_xyz_from_str, write_xyz_to_string};
        let mut original = PointCloud::default();
        original.add_point(&Point::new(0.0, 0.0, 0.0));
        original.add_point(&Point::new(1.0, 0.0, 0.0));
        original.add_point(&Point::new(0.0, 1.0, 0.0));
        original.add_point(&Point::new(0.0, 0.0, 1.0));
        let s = write_xyz_to_string(&original);
        let loaded = read_xyz_from_str(&s);

        MINI_CHECK!(loaded.point_count() == original.point_count());
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[1][0], 1.0));
    })
}

pub fn run_io_read_colors() -> TestResult {
    MINI_TEST!("Read Colors", {
        use crate::io::read_xyz_from_str;
        // "x y z r g b" (0-255): point 0 red, point 1 green
        let cloud = read_xyz_from_str("0 0 0 255 0 0\n1 0 0 0 255 0\n");
        MINI_CHECK!(cloud.point_count() == 2);
        MINI_CHECK!(cloud.color_count() == 2);
        let c0 = cloud.get_color(0);
        MINI_CHECK!(TOLERANCE.is_close(c0.r as f64, 1.0) && TOLERANCE.is_close(c0.g as f64, 0.0));
        let c1 = cloud.get_color(1);
        MINI_CHECK!(TOLERANCE.is_close(c1.g as f64, 1.0) && TOLERANCE.is_close(c1.r as f64, 0.0));
    })
}

REGISTER_MINI_TEST!("Io", "Read Bunny", crate::io_test::run_io_read_bunny);
REGISTER_MINI_TEST!("Io", "String Roundtrip", crate::io_test::run_io_string_roundtrip);
REGISTER_MINI_TEST!("Io", "Write Read Roundtrip", crate::io_test::run_io_write_read_roundtrip);
REGISTER_MINI_TEST!("Io", "Read Colors", crate::io_test::run_io_read_colors);

// The PDF importer test lives in the Io suite (file-IO is io). Feature-gated like the pdf
// module itself: pdf-less builds (macOS/Windows CI, wasm) simply run the four xyz tests above.
#[cfg(all(feature = "pdf", not(target_arch = "wasm32")))]
pub fn run_io_pdf_import_minimal() -> TestResult {
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

#[cfg(all(feature = "pdf", not(target_arch = "wasm32")))]
REGISTER_MINI_TEST!("Io", "Import Minimal", crate::io_test::run_io_pdf_import_minimal);
